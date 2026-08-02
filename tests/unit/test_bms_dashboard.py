import argparse
import asyncio
import importlib.util
import inspect
import json
from pathlib import Path
import sqlite3
import sys
import tempfile


MODULE_PATH = Path(__file__).parents[2] / "tools" / "bms_dashboard.py"
SPEC = importlib.util.spec_from_file_location("bms_dashboard", MODULE_PATH)
dashboard = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = dashboard
SPEC.loader.exec_module(dashboard)


def test_history_sample_keeps_only_chart_values():
    result = dashboard.history_sample(
        "2026-08-01T20:00:00Z",
        {"voltage": 26.5, "current": -12.3, "power": -325.0, "battery_level": 84, "temperature": 28.2, "cells": [1]},
    )
    assert result == {
        "timestamp": "2026-08-01T20:00:00Z",
        "voltage": 26.5,
        "current": -12.3,
        "power": -325.0,
        "soc": 84,
        "temperature": 28.2,
    }


def test_lan_control_handlers_do_not_require_an_additional_token():
    handlers = (
        dashboard.inverter_configuration_handler,
        dashboard.inverter_setting_handler,
        dashboard.bms_configuration_handler,
        dashboard.bms_setting_handler,
        dashboard.jk_protocol_handler,
        dashboard.seplos_protocol_handler,
    )
    assert all("require_control" not in inspect.getsource(handler) for handler in handlers)


def test_config_validates_intervals():
    args = argparse.Namespace(host="127.0.0.1", port=9000, poll_interval=12.0)
    config = dashboard.DashboardConfig.from_env_and_args(args)
    assert config.host == "127.0.0.1"
    assert config.port == 9000
    assert config.poll_interval_seconds == 12.0


def test_polling_preserves_last_data_when_one_device_disappears():
    calls = {"daly": 0, "seplos": 0, "jk": 0}

    async def telemetry_reader(alias):
        calls[alias] += 1
        if alias == "seplos":
            raise TimeoutError("not in range")
        return {"info": {"model": alias}, "telemetry": {"voltage": 26.5, "current": -1.2}}

    async def protocol_reader():
        return {}

    config = dashboard.DashboardConfig(host="127.0.0.1", poll_interval_seconds=20)
    state = dashboard.DashboardState(
        config,
        telemetry_reader=telemetry_reader,
        jk_protocol_reader=protocol_reader,
        seplos_protocol_reader=protocol_reader,
    )
    assert asyncio.run(state.poll_once()) is True
    snapshot = state.public_snapshot()
    assert calls == {"daly": 1, "seplos": 1, "jk": 1}
    assert snapshot["devices"]["daly"]["online"] is True
    assert snapshot["devices"]["jk"]["online"] is True
    assert snapshot["devices"]["seplos"]["online"] is False
    assert "not in range" in snapshot["devices"]["seplos"]["error"]
    assert len(snapshot["history"]["daly"]) == 1


def test_record_error_retains_previous_telemetry():
    state = dashboard.DashboardState(dashboard.DashboardConfig())
    state.record_success("daly", {"info": {}, "telemetry": {"voltage": 26.4}}, "one")
    state.record_error("daly", TimeoutError("gone"), "two")
    assert state.devices["daly"]["online"] is False
    assert state.devices["daly"]["telemetry"]["voltage"] == 26.4


def test_bms_configuration_write_is_serialized_and_requests_telemetry_refresh():
    calls = []

    async def reader(alias):
        calls.append(("read", alias))
        return {"device": alias, "groups": []}

    async def writer(alias, setting, value, confirmation):
        calls.append(("write", alias, setting, value, confirmation))
        return {
            "written": True,
            "verified": True,
            "before": 10,
            "after": 11,
            "after_configuration": {"device": alias, "groups": []},
        }

    state = dashboard.DashboardState(
        dashboard.DashboardConfig(),
        bms_configuration_reader=reader,
        bms_setting_writer=writer,
    )
    assert asyncio.run(state.read_bms_configuration("daly"))["device"] == "daly"
    result = asyncio.run(state.write_bms_setting("jk", "cell_count", 8, "serial"))

    assert result["verified"] is True
    assert calls == [
        ("read", "daly"),
        ("write", "jk", "cell_count", 8, "serial"),
    ]
    assert state.refresh_event.is_set()


def test_seplos_protocol_change_updates_cached_state_and_capability():
    calls = []

    async def writer(profile, confirmation):
        calls.append((profile, confirmation))
        return {
            "changed": True,
            "identity": {"bms_serial_number": confirmation},
            "before": {"selector_profile": "pylon_485"},
            "after": {"selector_profile": profile, "selector_index": 10},
        }

    state = dashboard.DashboardState(
        dashboard.DashboardConfig(),
        seplos_protocol_writer=writer,
    )
    result = asyncio.run(state.change_seplos_protocol("growatt_485", "SP144B-C2506260009"))
    snapshot = state.public_snapshot()
    assert result["changed"] is True
    assert calls == [("growatt_485", "SP144B-C2506260009")]
    assert snapshot["protocols"]["seplos"]["data"]["inverter_protocol"]["selector_profile"] == "growatt_485"
    assert snapshot["capabilities"]["seplos"]["protocol_write"] is True
    assert {profile["name"] for profile in snapshot["seplos_protocol_profiles"]} == {
        "srne_485", "growatt_485", "pylon_485"
    }


def test_static_dashboard_is_english_and_renders_individual_cells():
    index = (dashboard.STATIC_ROOT / "index.html").read_text(encoding="utf-8")
    script = (dashboard.STATIC_ROOT / "app.js").read_text(encoding="utf-8")
    assert '<html lang="en">' in index
    assert 'id="cell-voltage-grid"' in index
    assert "Individual cell voltages" in index
    assert "function renderCellVoltages(cells)" in script
    assert 'data-tab="inverters"' in index
    assert 'id="inverter-grid"' in index
    assert "function renderInverters(data)" in script
    assert "Anenji and EASUN inverters" in index
    assert 'data-tab="inverter-control"' in index
    assert 'id="inverter-config-content"' in index
    assert "function renderInverterConfiguration(configuration)" in script
    assert "function applyInverterSetting(key, button)" in script
    assert 'id="inverter-config-refresh"' not in index
    assert "const INVERTER_CONFIG_REFRESH_MS = 30_000" in script
    assert "function startInverterConfigPolling()" in script
    assert 'data-tab="bms-control"' in index
    assert ">BMS control<" in index
    assert 'id="bms-config-content"' in index
    assert "Bluetooth is used only to configure the selected BMS" in index
    assert "operational telemetry continues through the ESP32 wired links" in index
    assert "function renderBmsConfiguration(configuration)" in script
    assert "function applyBmsSetting(key, button)" in script
    assert "const BMS_CONFIG_REFRESH_MS = 30_000" in script
    assert "function startBmsConfigPolling()" in script
    assert "Protocols and control" not in index
    assert "Live inverter configuration loaded" not in script
    assert "X-Control-Token" not in script
    assert 'id="inverter-control-token"' not in index
    assert 'id="control-token"' not in index
    assert 'id="seplos-control-token"' not in index
    for romanian_text in ("Aștept", "Actualizează", "Tensiune", "Curent", "Celule", "Aplică"):
        assert romanian_text not in index
        assert romanian_text not in script


def test_inverter_snapshot_reads_latest_payload_and_marks_stale_data():
    with tempfile.TemporaryDirectory() as directory:
        database = Path(directory) / "telemetry.sqlite3"
        with sqlite3.connect(database) as connection:
            connection.executescript(
                """
                PRAGMA user_version = 4;
                CREATE TABLE inverters (
                    inverter_id TEXT PRIMARY KEY,
                    name TEXT NOT NULL,
                    protocol TEXT NOT NULL,
                    mac TEXT NOT NULL,
                    linked_board_id TEXT,
                    configured_ip TEXT NOT NULL,
                    last_seen_utc TEXT,
                    last_error TEXT
                );
                CREATE TABLE inverter_samples (
                    id INTEGER PRIMARY KEY,
                    inverter_id TEXT NOT NULL,
                    sampled_at_utc TEXT NOT NULL,
                    sampled_at_unix_ms INTEGER NOT NULL,
                    source_ip TEXT NOT NULL,
                    payload_json TEXT NOT NULL
                );
                CREATE VIEW latest_inverter_telemetry AS
                SELECT samples.*
                FROM inverter_samples AS samples
                JOIN (
                    SELECT inverter_id, MAX(id) AS latest_id
                    FROM inverter_samples
                    GROUP BY inverter_id
                ) AS latest ON latest.latest_id = samples.id;
                INSERT INTO inverters VALUES (
                    'inverter-easun', 'EASUN', 'easun_qpigs',
                    'c4:d8:d5:1c:6a:06', 'inverter-board-1',
                    '192.168.1.185', '2026-08-02T12:00:00Z', NULL
                );
                """
            )
            connection.execute(
                "INSERT INTO inverter_samples VALUES (?, ?, ?, ?, ?, ?)",
                (
                    1,
                    "inverter-easun",
                    "2026-08-02T12:00:00Z",
                    170_000,
                    "192.168.1.185",
                    json.dumps(
                        {
                            "protocol": "EASUN_VOLTRONIC_QPIGS",
                            "output_power_w": 69,
                            "battery_voltage_v": 27.0,
                        }
                    ),
                ),
            )

        fresh = dashboard.read_inverter_snapshot(database, 90, now_unix_ms=200_000)
        device = fresh["devices"]["inverter-easun"]
        assert fresh["available"] is True
        assert fresh["schema_version"] == 4
        assert device["online"] is True
        assert device["age_seconds"] == 30
        assert device["telemetry"]["output_power_w"] == 69

        stale = dashboard.read_inverter_snapshot(database, 90, now_unix_ms=300_001)
        stale_device = stale["devices"]["inverter-easun"]
        assert stale_device["online"] is False
        assert stale_device["stale"] is True
        assert "stale" in stale_device["error"]


def test_inverter_snapshot_keeps_dashboard_available_when_database_is_missing():
    snapshot = dashboard.read_inverter_snapshot(Path("/definitely/missing/telemetry.sqlite3"))

    assert snapshot["available"] is False
    assert snapshot["devices"] == {}
    assert "OperationalError" in snapshot["error"]


def test_inverter_target_resolves_control_network_fields():
    with tempfile.TemporaryDirectory() as directory:
        database = Path(directory) / "telemetry.sqlite3"
        with sqlite3.connect(database) as connection:
            connection.executescript(
                """
                CREATE TABLE inverters (
                    inverter_id TEXT PRIMARY KEY,
                    name TEXT NOT NULL,
                    protocol TEXT NOT NULL,
                    configured_ip TEXT NOT NULL,
                    local_ip TEXT NOT NULL,
                    local_port INTEGER NOT NULL
                );
                INSERT INTO inverters VALUES (
                    'inverter-anenji', 'Anenji', 'anenji_modbus',
                    '192.168.1.18', '192.168.1.44', 8899
                );
                """
            )

        assert dashboard.read_inverter_target(database, "inverter-anenji") == {
            "id": "inverter-anenji",
            "name": "Anenji",
            "protocol": "anenji_modbus",
            "ip": "192.168.1.18",
            "local_ip": "192.168.1.44",
            "local_port": 8899,
        }
