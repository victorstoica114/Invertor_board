import json
import sqlite3
import time
import unittest

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
        collector.register_boards(self.connection, [self.board])

    def tearDown(self):
        self.connection.close()

    def test_resolves_live_lease_by_mac(self):
        leases = collector.parse_dnsmasq_leases(
            f"{int(time.time()) + 3600} 58:8C:81:3A:D6:90 "
            "192.168.50.216 different-hostname *"
        )

        result = collector.resolve_board_ip(self.board, leases, {})

        self.assertEqual(result, "192.168.50.216")

    def test_missing_board_does_not_create_sample(self):
        result = collector.fetch_telemetry(self.board, None, 0.1)

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
                "SELECT last_error FROM boards WHERE board_id = ?",
                (self.board.board_id,),
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
        self.assertEqual(row["protocol"], "RS485_PYLON")
        self.assertAlmostEqual(row["pack_voltage_v"], 51.2)
        self.assertEqual(json.loads(row["cells_v_json"]), [3.2, 3.21])
        self.assertEqual(json.loads(row["payload_json"])["future_field"], "preserved")


if __name__ == "__main__":
    unittest.main()
