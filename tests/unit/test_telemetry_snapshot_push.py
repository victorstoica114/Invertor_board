import json
from pathlib import Path
import sqlite3
import tempfile

from tools import telemetry_collector as collector
from tools import telemetry_snapshot_push as publisher


def create_source_database(path: Path) -> None:
    connection = collector.open_database(path)
    board = collector.BoardConfig(
        board_id="inverter-board-1",
        hostname="inverter-board-1",
        mac="58:8c:81:3a:d6:90",
        static_ip="192.168.1.5",
    )
    source = collector.BmsSourceConfig(
        source_id="inverter-board-1-bms1",
        board_id=board.board_id,
        name="JK BMS",
        endpoint="/api/telemetry",
    )
    collector.register_boards(connection, [board])
    collector.register_sources(connection, [source])
    collector.store_results(
        connection,
        [
            collector.CollectionResult(
                board=board,
                source=source,
                ip="192.168.1.5",
                payload={
                    "valid": True,
                    "stale": False,
                    "protocol": "JKBMS_MODBUS",
                    "cell_count": 2,
                    "cells_v": [3.38, 3.39],
                    "future_field": {"preserved": True},
                },
                sampled_at_utc="2026-08-02T12:00:00.000Z",
                sampled_at_unix_ms=1_785_645_600_000,
                error=None,
            )
        ],
    )
    connection.close()


def test_consistent_snapshot_preserves_complete_database_and_metadata():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        source = root / "source.sqlite3"
        snapshot = root / "snapshot.sqlite3"
        create_source_database(source)

        metadata = publisher.create_snapshot(source, snapshot)

        assert metadata["schema_version"] == 2
        assert metadata["sample_count"] == 1
        assert metadata["sources"] == [
            {
                "source_id": "inverter-board-1-bms1",
                "name": "JK BMS",
                "board_id": "inverter-board-1",
                "endpoint": "/api/telemetry",
                "sample_count": 1,
                "latest_sample_utc": "2026-08-02T12:00:00.000Z",
                "latest_protocol": "JKBMS_MODBUS",
                "latest_cell_count": 2,
            }
        ]
        assert metadata["sha256"] == publisher.sha256_file(snapshot)
        with sqlite3.connect(f"{snapshot.resolve().as_uri()}?mode=ro", uri=True) as db:
            payload = json.loads(
                db.execute("SELECT payload_json FROM telemetry_samples").fetchone()[0]
            )
            assert payload["future_field"] == {"preserved": True}
            assert db.execute("PRAGMA integrity_check").fetchone()[0] == "ok"


def test_logical_key_ignores_snapshot_timestamp_but_detects_new_samples():
    base = {
        "schema_version": 2,
        "sample_count": 10,
        "latest_sample_utc": "2026-08-02T12:00:00Z",
        "generated_at_utc": "first",
        "sha256": "one",
        "sources": [
            {
                "source_id": "bms1",
                "sample_count": 10,
                "latest_sample_utc": "2026-08-02T12:00:00Z",
                "latest_protocol": "JKBMS_MODBUS",
                "latest_cell_count": 8,
            }
        ],
    }
    same_data = {**base, "generated_at_utc": "later", "sha256": "two"}
    new_data = {**same_data, "sample_count": 11}

    assert publisher.logical_snapshot_key(base) == publisher.logical_snapshot_key(
        same_data
    )
    assert publisher.logical_snapshot_key(base) != publisher.logical_snapshot_key(
        new_data
    )
