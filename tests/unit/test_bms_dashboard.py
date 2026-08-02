import argparse
import asyncio
import importlib.util
from pathlib import Path
import sys


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


def test_control_auth_is_disabled_without_token_and_constant_time_with_token():
    assert dashboard.control_authorized({"X-Control-Token": "secret"}, "") is False
    assert dashboard.control_authorized({"X-Control-Token": "secret"}, "secret") is True
    assert dashboard.control_authorized({"X-Control-Token": "wrong"}, "secret") is False


def test_config_validates_intervals(monkeypatch):
    monkeypatch.setenv("BMS_DASHBOARD_CONTROL_TOKEN", "secret")
    args = argparse.Namespace(host="127.0.0.1", port=9000, poll_interval=12.0)
    config = dashboard.DashboardConfig.from_env_and_args(args)
    assert config.host == "127.0.0.1"
    assert config.port == 9000
    assert config.poll_interval_seconds == 12.0
    assert config.control_token == "secret"


def test_polling_preserves_last_data_when_one_device_disappears():
    calls = {"daly": 0, "seplos": 0, "jk": 0}

    async def telemetry_reader(alias):
        calls[alias] += 1
        if alias == "seplos":
            raise TimeoutError("not in range")
        return {"info": {"model": alias}, "telemetry": {"voltage": 26.5, "current": -1.2}}

    async def protocol_reader():
        return {}

    config = dashboard.DashboardConfig(host="127.0.0.1", poll_interval_seconds=20, control_token="secret")
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
        dashboard.DashboardConfig(control_token="secret"),
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
    for romanian_text in ("Aștept", "Actualizează", "Tensiune", "Curent", "Celule", "Aplică"):
        assert romanian_text not in index
        assert romanian_text not in script
