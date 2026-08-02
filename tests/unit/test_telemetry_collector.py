import json
import sqlite3
import time
import unittest
from unittest import mock

from tools import telemetry_collector as collector


class TelemetryCollectorTests(unittest.TestCase):
    def setUp(self):
        self.connection = sqlite3.connect(":memory:")
        self.connection.row_factory = sqlite3.Row
        self.connection.execute("PRAGMA foreign_keys = ON")
        self.connection.executescript(collector.SCHEMA_SQL)
        self.board = collector.BoardConfig(
            board_id="inverter-board-1",
            hostname="inverter-board-1",
            mac="58:8c:81:3a:d6:90",
        )
        self.source = collector.BmsSourceConfig(
            source_id="inverter-board-1-bms1",
            board_id=self.board.board_id,
            name="JK BMS",
            endpoint="/api/telemetry",
        )
        collector.register_boards(self.connection, [self.board])
        collector.register_sources(self.connection, [self.source])
        self.inverter = collector.InverterConfig(
            inverter_id="inverter-easun",
            name="EASUN",
            protocol="easun_qpigs",
            mac="c4:d8:d5:1c:6a:06",
            ip="192.168.1.185",
            local_ip="192.168.1.44",
            local_port=8899,
            linked_board_id=self.board.board_id,
        )
        collector.register_inverters(self.connection, [self.inverter])

    def tearDown(self):
        self.connection.close()

    def test_resolves_live_lease_by_mac(self):
        leases = collector.parse_dnsmasq_leases(
            f"{int(time.time()) + 3600} 58:8C:81:3A:D6:90 "
            "192.168.50.216 different-hostname *"
        )

        result = collector.resolve_board_ip(self.board, leases, {})

        self.assertEqual(result, "192.168.50.216")

    def test_static_ip_overrides_stale_lease_and_cached_address(self):
        board = collector.BoardConfig(
            board_id=self.board.board_id,
            hostname=self.board.hostname,
            mac=self.board.mac,
            static_ip="192.168.1.5",
        )
        leases = collector.parse_dnsmasq_leases(
            f"{int(time.time()) + 3600} {self.board.mac} "
            "192.168.50.216 inverter-board-1 *"
        )

        self.assertEqual(
            collector.resolve_board_ip(
                board, leases, {self.board.board_id: "192.168.50.111"}
            ),
            "192.168.1.5",
        )

    def test_current_configuration_defines_all_three_bms_sources(self):
        config = collector.load_config(collector.DEFAULT_CONFIG)

        self.assertEqual(len(config.boards), 2)
        self.assertEqual(
            {source.source_id: source.endpoint for source in config.sources},
            {
                "inverter-board-1-bms1": "/api/telemetry",
                "inverter-board-1-bms2": "/api/telemetry2",
                "inverter-board-2-bms1": "/api/telemetry",
            },
        )
        self.assertEqual(
            {
                inverter.inverter_id: (
                    inverter.protocol,
                    inverter.ip,
                    inverter.local_port,
                )
                for inverter in config.inverters
            },
            {
                "inverter-anenji": ("anenji_modbus", "192.168.1.18", 8899),
                "inverter-easun": ("easun_qpigs", "192.168.1.185", 8899),
            },
        )

    def test_missing_board_does_not_create_sample(self):
        result = collector.fetch_telemetry(self.board, self.source, None, 0.1)

        stored, skipped = collector.store_results(self.connection, [result])

        self.assertEqual((stored, skipped), (0, 1))
        self.assertEqual(
            self.connection.execute(
                "SELECT COUNT(*) FROM telemetry_samples"
            ).fetchone()[0],
            0,
        )
        self.assertIn(
            "no current DHCP lease",
            self.connection.execute(
                "SELECT last_error FROM bms_sources WHERE source_id = ?",
                (self.source.source_id,),
            ).fetchone()[0],
        )

    def test_successful_sample_preserves_typed_fields_and_json(self):
        payload = {
            "valid": True,
            "protocol": "RS485_PYLON",
            "pack_voltage_v": 51.2,
            "soc_pct": 75,
            "cells_v": [3.2, 3.21],
            "future_field": "preserved",
        }
        result = collector.CollectionResult(
            board=self.board,
            source=self.source,
            ip="192.168.50.216",
            payload=payload,
            sampled_at_utc="2026-07-26T12:00:00.000Z",
            sampled_at_unix_ms=1_785_066_000_000,
            error=None,
        )

        stored, skipped = collector.store_results(self.connection, [result])
        row = self.connection.execute(
            "SELECT * FROM telemetry_samples"
        ).fetchone()

        self.assertEqual((stored, skipped), (1, 0))
        self.assertEqual(row["valid"], 1)
        self.assertEqual(row["source_id"], self.source.source_id)
        self.assertEqual(row["protocol"], "RS485_PYLON")
        self.assertAlmostEqual(row["pack_voltage_v"], 51.2)
        self.assertEqual(json.loads(row["cells_v_json"]), [3.2, 3.21])
        self.assertEqual(json.loads(row["payload_json"])["future_field"], "preserved")

    def test_invalid_payload_is_not_stored(self):
        class FakeResponse:
            status = 200

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def read(self, _limit):
                return b'{"valid":false,"stale":false,"cells_v":[]}'

        with mock.patch.object(
            collector.urllib.request, "urlopen", return_value=FakeResponse()
        ):
            result = collector.fetch_telemetry(
                self.board, self.source, "192.168.1.5", 0.1
            )

        self.assertFalse(result.succeeded)
        self.assertIn("not valid", result.error)
        self.assertEqual(collector.store_results(self.connection, [result]), (0, 1))
        self.assertEqual(
            self.connection.execute(
                "SELECT COUNT(*) FROM telemetry_samples"
            ).fetchone()[0],
            0,
        )

    def test_two_sources_on_one_board_are_stored_independently(self):
        second = collector.BmsSourceConfig(
            source_id="inverter-board-1-bms2",
            board_id=self.board.board_id,
            name="Daly BMS",
            endpoint="/api/telemetry2",
        )
        collector.register_sources(self.connection, [second])
        results = [
            collector.CollectionResult(
                board=self.board,
                source=source,
                ip="192.168.1.5",
                payload={
                    "valid": True,
                    "stale": False,
                    "protocol": protocol,
                    "cells_v": cells,
                    "future_data": {"kept": True},
                },
                sampled_at_utc="2026-08-02T12:00:00.000Z",
                sampled_at_unix_ms=1_785_645_600_000 + offset,
                error=None,
            )
            for source, protocol, cells, offset in (
                (self.source, "JKBMS_MODBUS", [3.38] * 8, 0),
                (second, "DALY_RS485", [3.39] * 8, 1),
            )
        ]

        self.assertEqual(collector.store_results(self.connection, results), (2, 0))
        rows = self.connection.execute(
            "SELECT source_id, protocol, cells_v_json FROM latest_telemetry ORDER BY source_id"
        ).fetchall()
        self.assertEqual(
            [(row["source_id"], row["protocol"]) for row in rows],
            [
                ("inverter-board-1-bms1", "JKBMS_MODBUS"),
                ("inverter-board-1-bms2", "DALY_RS485"),
            ],
        )

    def test_successful_inverter_sample_keeps_only_live_fields_and_raw_queries(self):
        payload = {
            "protocol": "EASUN_VOLTRONIC_QPIGS",
            "working_mode": "BATTERY",
            "output_voltage_v": 230.1,
            "output_power_w": 123,
            "battery_voltage_v": 26.8,
            "battery_current_a": -4.5,
            "pv_voltage_v": 302.4,
            "battery_voltage_scc_v": 26.9,
            "device_status_bits": "10101010",
            "rating_fields": ["configuration", "must", "not", "persist"],
            "raw": {
                "responses": {
                    "QPIGS": "full raw response",
                    "QPIRI": "configuration response",
                },
                "frames_hex": {"QPIGS": "live", "QPIRI": "settings"},
            },
            "future_inverter_field": 42,
        }
        result = collector.InverterCollectionResult(
            inverter=self.inverter,
            payload=payload,
            sampled_at_utc="2026-08-02T12:00:00.000Z",
            sampled_at_unix_ms=1_785_645_600_000,
            error=None,
        )

        self.assertEqual(
            collector.store_inverter_results(self.connection, [result]), (1, 0)
        )
        row = self.connection.execute(
            "SELECT * FROM latest_inverter_telemetry WHERE inverter_id = ?",
            (self.inverter.inverter_id,),
        ).fetchone()
        self.assertEqual(row["protocol"], "EASUN_VOLTRONIC_QPIGS")
        self.assertAlmostEqual(row["output_voltage_v"], 230.1)
        self.assertEqual(row["output_power_w"], 123)
        self.assertAlmostEqual(row["battery_current_a"], -4.5)
        self.assertAlmostEqual(row["battery_voltage_scc_v"], 26.9)
        self.assertEqual(row["device_status_bits"], "10101010")
        stored_payload = json.loads(row["payload_json"])
        self.assertNotIn("future_inverter_field", stored_payload)
        self.assertNotIn("rating_fields", stored_payload)
        self.assertEqual(
            stored_payload["raw"]["responses"]["QPIGS"], "full raw response"
        )
        self.assertNotIn("QPIRI", stored_payload["raw"]["responses"])
        self.assertNotIn("QPIRI", stored_payload["raw"]["frames_hex"])

    def test_two_inverters_can_share_standard_callback_port(self):
        second = collector.InverterConfig(
            inverter_id="inverter-anenji",
            name="Anenji",
            protocol="anenji_modbus",
            mac="34:5f:45:48:cf:15",
            ip="192.168.1.18",
            local_ip=self.inverter.local_ip,
            local_port=self.inverter.local_port,
            linked_board_id=self.board.board_id,
        )

        collector.register_inverters(self.connection, [second])

        self.assertEqual(
            self.connection.execute("SELECT COUNT(*) FROM inverters").fetchone()[0],
            2,
        )

    def test_v1_to_v4_database_migration_preserves_samples(self):
        connection = sqlite3.connect(":memory:")
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA foreign_keys = ON")
        connection.executescript(
            """
            PRAGMA user_version = 1;
            CREATE TABLE boards (
                board_id TEXT PRIMARY KEY,
                hostname TEXT NOT NULL UNIQUE,
                mac TEXT NOT NULL UNIQUE COLLATE NOCASE,
                last_ip TEXT,
                last_seen_utc TEXT,
                last_error TEXT,
                updated_at_utc TEXT NOT NULL
            );
            CREATE TABLE telemetry_samples (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                board_id TEXT NOT NULL,
                sampled_at_unix_ms INTEGER NOT NULL
            );
            INSERT INTO boards (
                board_id, hostname, mac, updated_at_utc
            ) VALUES (
                'inverter-board-1', 'inverter-board-1',
                '58:8c:81:3a:d6:90', 'old'
            );
            INSERT INTO telemetry_samples (
                board_id, sampled_at_unix_ms
            ) VALUES ('inverter-board-1', 123);
            """
        )

        collector.migrate_database_v4(connection)

        row = connection.execute(
            "SELECT board_id, source_id FROM telemetry_samples"
        ).fetchone()
        self.assertEqual(row["board_id"], "inverter-board-1")
        self.assertEqual(row["source_id"], "inverter-board-1-bms1")
        self.assertEqual(connection.execute("PRAGMA user_version").fetchone()[0], 4)
        self.assertEqual(
            connection.execute("SELECT COUNT(*) FROM latest_telemetry").fetchone()[0],
            1,
        )
        self.assertIsNotNone(
            connection.execute(
                "SELECT 1 FROM sqlite_master WHERE type='table' AND name='inverter_samples'"
            ).fetchone()
        )
        connection.close()

    def test_v3_to_v4_migration_preserves_inverter_samples_and_shares_port(self):
        connection = sqlite3.connect(":memory:")
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA foreign_keys = ON")
        connection.executescript(
            """
            PRAGMA user_version = 3;
            CREATE TABLE boards (board_id TEXT PRIMARY KEY);
            INSERT INTO boards VALUES ('inverter-board-1');
            CREATE TABLE inverters (
                inverter_id TEXT PRIMARY KEY,
                name TEXT NOT NULL,
                protocol TEXT NOT NULL,
                mac TEXT NOT NULL UNIQUE COLLATE NOCASE,
                linked_board_id TEXT,
                configured_ip TEXT NOT NULL,
                local_ip TEXT NOT NULL,
                local_port INTEGER NOT NULL,
                last_ip TEXT,
                last_seen_utc TEXT,
                last_error TEXT,
                updated_at_utc TEXT NOT NULL,
                UNIQUE (local_ip, local_port),
                FOREIGN KEY (linked_board_id) REFERENCES boards(board_id)
            );
            CREATE TABLE inverter_samples (
                id INTEGER PRIMARY KEY,
                inverter_id TEXT NOT NULL,
                FOREIGN KEY (inverter_id) REFERENCES inverters(inverter_id)
            );
            INSERT INTO inverters (
                inverter_id, name, protocol, mac, linked_board_id,
                configured_ip, local_ip, local_port, updated_at_utc
            ) VALUES (
                'inverter-easun', 'EASUN', 'easun_qpigs',
                'c4:d8:d5:1c:6a:06', 'inverter-board-1',
                '192.168.1.185', '192.168.1.44', 8899, 'old'
            );
            INSERT INTO inverter_samples VALUES (1, 'inverter-easun');
            """
        )

        collector.migrate_database_v4(connection)
        connection.execute(
            """
            INSERT INTO inverters (
                inverter_id, name, protocol, mac, linked_board_id,
                configured_ip, local_ip, local_port, updated_at_utc
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                "inverter-anenji",
                "Anenji",
                "anenji_modbus",
                "34:5f:45:48:cf:15",
                "inverter-board-1",
                "192.168.1.18",
                "192.168.1.44",
                8899,
                "new",
            ),
        )

        self.assertEqual(connection.execute("PRAGMA user_version").fetchone()[0], 4)
        self.assertEqual(
            connection.execute("SELECT COUNT(*) FROM inverter_samples").fetchone()[0],
            1,
        )
        self.assertEqual(
            connection.execute("SELECT COUNT(*) FROM inverters").fetchone()[0], 2
        )
        self.assertEqual(connection.execute("PRAGMA foreign_key_check").fetchall(), [])
        connection.close()

    def test_v4_to_v5_migration_backfills_live_fields_and_removes_settings(self):
        connection = sqlite3.connect(":memory:")
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA foreign_keys = ON")
        connection.executescript(
            """
            PRAGMA user_version = 4;
            CREATE TABLE inverters (inverter_id TEXT PRIMARY KEY);
            INSERT INTO inverters VALUES ('inverter-easun');
            CREATE TABLE inverter_samples (
                id INTEGER PRIMARY KEY,
                inverter_id TEXT NOT NULL,
                payload_json TEXT NOT NULL,
                FOREIGN KEY (inverter_id) REFERENCES inverters(inverter_id)
            );
            """
        )
        old_payload = {
            "protocol": "EASUN_VOLTRONIC_QPIGS",
            "battery_voltage_scc_v": 27.1,
            "device_status_bits": "10101010",
            "rating_fields": ["settings"],
            "raw": {
                "responses": {"QPIGS": "live", "QPIRI": "settings"},
                "frames_hex": {"QPIGS": "aa", "QPIRI": "bb"},
            },
        }
        connection.execute(
            "INSERT INTO inverter_samples VALUES (?, ?, ?)",
            (1, "inverter-easun", json.dumps(old_payload)),
        )

        collector.migrate_database_v5(connection)

        row = connection.execute("SELECT * FROM inverter_samples").fetchone()
        payload = json.loads(row["payload_json"])
        self.assertEqual(connection.execute("PRAGMA user_version").fetchone()[0], 5)
        self.assertAlmostEqual(row["battery_voltage_scc_v"], 27.1)
        self.assertEqual(row["device_status_bits"], "10101010")
        self.assertNotIn("rating_fields", payload)
        self.assertNotIn("QPIRI", payload["raw"]["responses"])
        self.assertNotIn("QPIRI", payload["raw"]["frames_hex"])
        self.assertEqual(connection.execute("PRAGMA foreign_key_check").fetchall(), [])
        connection.close()


if __name__ == "__main__":
    unittest.main()
