#!/usr/bin/env python3
"""LAN dashboard for local Bluetooth BMS devices and Wi-Fi inverters."""

from __future__ import annotations

import argparse
import asyncio
from collections import deque
import copy
import datetime as dt
from dataclasses import dataclass
import json
import logging
import os
from pathlib import Path
import re
import sqlite3
import time
from typing import Any, Awaitable, Callable, Final, Mapping

from aiohttp import web

try:
    from tools import bms_ble, bms_config, inverter_protocols, miot_plugs, tuya_ac
except ModuleNotFoundError:  # Direct execution: python tools/bms_dashboard.py
    import bms_ble  # type: ignore[no-redef]
    import bms_config  # type: ignore[no-redef]
    import inverter_protocols  # type: ignore[no-redef]
    import miot_plugs  # type: ignore[no-redef]
    import tuya_ac  # type: ignore[no-redef]


LOGGER = logging.getLogger("bms_dashboard")
PROJECT_ROOT: Final[Path] = Path(__file__).resolve().parents[1]
STATIC_ROOT: Final[Path] = PROJECT_ROOT / "tools" / "bms_dashboard_static"
DEFAULT_TELEMETRY_DATABASE: Final[Path] = PROJECT_ROOT / "data" / "telemetry.sqlite3"
DEFAULT_INVERTER_INVENTORY: Final[Path] = PROJECT_ROOT / "tools" / "telemetry_boards.json"
HISTORY_LIMIT: Final[int] = 180
IOT_HISTORY_LIMIT: Final[int] = 360
MIN_REFRESH_SECONDS: Final[float] = 3.0
DEFAULT_INVERTER_STALE_SECONDS: Final[float] = 90.0
DEFAULT_INVERTER_POLL_SECONDS: Final[float] = 30.0
INVERTER_VOLTAGE_MAX_AGE_SECONDS: Final[float] = 300.0
INVERTER_CONTROL_TIMEOUT_SECONDS: Final[float] = 12.0
INVERTER_ID_PATTERN: Final[re.Pattern[str]] = re.compile(r"^[a-z0-9][a-z0-9_-]{0,63}$")

TelemetryReader = Callable[[str], Awaitable[dict[str, Any]]]
ProtocolReader = Callable[[], Awaitable[dict[str, Any]]]
JkProtocolWriter = Callable[[str, str, str], Awaitable[dict[str, Any]]]
SeplosProtocolWriter = Callable[[str, str], Awaitable[dict[str, Any]]]
BmsConfigurationReader = Callable[[str], Awaitable[dict[str, Any]]]
BmsSettingWriter = Callable[[str, str, Any, str], Awaitable[dict[str, Any]]]
PlugReader = Callable[[miot_plugs.PlugDefinition], dict[str, Any]]
PlugPowerWriter = Callable[[miot_plugs.PlugDefinition, bool], dict[str, Any]]
AcReader = Callable[[tuya_ac.AcDefinition], dict[str, Any]]
AcPowerWriter = Callable[[tuya_ac.AcDefinition, bool], dict[str, Any]]
AcSettingWriter = Callable[[tuya_ac.AcDefinition, str, Any], dict[str, Any]]
InverterReader = Callable[[str, str, str, int, float], dict[str, Any]]


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def finite_number(value: Any) -> float | int | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return value if value == value and value not in (float("inf"), float("-inf")) else None


def history_sample(timestamp: str, telemetry: Mapping[str, Any]) -> dict[str, Any]:
    """Select the small numeric subset needed for the dashboard charts."""
    return {
        "timestamp": timestamp,
        "voltage": finite_number(telemetry.get("voltage")),
        "current": finite_number(telemetry.get("current")),
        "power": finite_number(telemetry.get("power")),
        "soc": finite_number(telemetry.get("battery_level")),
        "temperature": finite_number(telemetry.get("temperature")),
    }


def read_inverter_snapshot(
    database: Path,
    stale_after_seconds: float = DEFAULT_INVERTER_STALE_SECONDS,
    now_unix_ms: int | None = None,
) -> dict[str, Any]:
    """Read the latest inverter samples without competing with the collector."""
    current_ms = time.time_ns() // 1_000_000 if now_unix_ms is None else now_unix_ms
    try:
        database_uri = f"{database.expanduser().resolve().as_uri()}?mode=ro"
        with sqlite3.connect(database_uri, uri=True, timeout=2) as connection:
            connection.row_factory = sqlite3.Row
            schema_version = int(connection.execute("PRAGMA user_version").fetchone()[0])
            rows = connection.execute(
                """
                SELECT
                    inverters.inverter_id,
                    inverters.name,
                    inverters.protocol AS configured_protocol,
                    inverters.mac,
                    inverters.linked_board_id,
                    inverters.configured_ip,
                    inverters.last_seen_utc,
                    inverters.last_error,
                    latest_inverter_telemetry.sampled_at_utc,
                    latest_inverter_telemetry.sampled_at_unix_ms,
                    latest_inverter_telemetry.source_ip,
                    latest_inverter_telemetry.payload_json
                FROM inverters
                LEFT JOIN latest_inverter_telemetry
                    ON latest_inverter_telemetry.inverter_id = inverters.inverter_id
                ORDER BY inverters.inverter_id
                """
            ).fetchall()
    except (OSError, sqlite3.Error) as exc:
        return {
            "available": False,
            "schema_version": None,
            "stale_after_seconds": stale_after_seconds,
            "error": f"{type(exc).__name__}: {exc}",
            "devices": {},
        }

    devices: dict[str, dict[str, Any]] = {}
    for row in rows:
        payload: dict[str, Any] = {}
        payload_error: str | None = None
        if row["payload_json"]:
            try:
                parsed = json.loads(str(row["payload_json"]))
                if not isinstance(parsed, dict):
                    raise ValueError("payload is not a JSON object")
                payload = parsed
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                payload_error = f"{type(exc).__name__}: {exc}"

        sampled_ms = row["sampled_at_unix_ms"]
        age_seconds = (
            max(0.0, (current_ms - int(sampled_ms)) / 1000.0)
            if sampled_ms is not None
            else None
        )
        stale = age_seconds is None or age_seconds > stale_after_seconds
        error = str(row["last_error"]) if row["last_error"] else payload_error
        if stale and error is None:
            error = (
                "No inverter sample is available"
                if age_seconds is None
                else f"Latest sample is stale ({age_seconds:.0f}s old)"
            )
        inverter_id = str(row["inverter_id"])
        devices[inverter_id] = {
            "id": inverter_id,
            "name": str(row["name"]),
            "mac": str(row["mac"]),
            "linked_board_id": row["linked_board_id"],
            "configured_protocol": str(row["configured_protocol"]),
            "configured_ip": str(row["configured_ip"]),
            "source_ip": row["source_ip"] or row["configured_ip"],
            "online": not stale and error is None,
            "stale": stale,
            "age_seconds": age_seconds,
            "last_sample": row["sampled_at_utc"],
            "last_seen": row["last_seen_utc"],
            "error": error,
            "telemetry": payload,
        }
    return {
        "available": True,
        "schema_version": schema_version,
        "stale_after_seconds": stale_after_seconds,
        "error": None,
        "devices": devices,
    }


def load_inverter_inventory(path: Path) -> dict[str, dict[str, Any]]:
    """Load the non-secret inverter connection inventory used for live polling."""
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot load inverter inventory {path}: {exc}") from exc
    rows = document.get("inverters") if isinstance(document, dict) else None
    if not isinstance(rows, list):
        raise ValueError("inverter inventory must contain an inverters array")
    inventory: dict[str, dict[str, Any]] = {}
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("each inverter inventory entry must be an object")
        inverter_id = str(row.get("id", "")).strip()
        protocol = str(row.get("protocol", "")).strip()
        inverter_ip = str(row.get("ip", "")).strip()
        local_ip = str(row.get("local_ip", "")).strip()
        try:
            local_port = int(row.get("local_port"))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{inverter_id or 'inverter'} has an invalid local port") from exc
        if not INVERTER_ID_PATTERN.fullmatch(inverter_id):
            raise ValueError(f"invalid inverter id: {inverter_id!r}")
        if inverter_id in inventory:
            raise ValueError(f"duplicate inverter id: {inverter_id}")
        if protocol not in inverter_protocols.READERS:
            raise ValueError(f"{inverter_id} uses unsupported protocol: {protocol}")
        if not inverter_ip or not local_ip:
            raise ValueError(f"{inverter_id} requires ip and local_ip")
        if not 1 <= local_port <= 65535:
            raise ValueError(f"{inverter_id} local_port must be between 1 and 65535")
        inventory[inverter_id] = {
            "id": inverter_id,
            "name": str(row.get("name") or inverter_id),
            "protocol": protocol,
            "mac": str(row.get("mac") or ""),
            "ip": inverter_ip,
            "local_ip": local_ip,
            "local_port": local_port,
            "linked_board_id": row.get("linked_board_id"),
        }
    return inventory


def inverter_voltage_reference_from_snapshot(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    """Select the freshest valid AC-output voltage from an inverter snapshot."""
    if not snapshot["available"]:
        return {
            "available": False,
            "voltage_v": None,
            "source_id": None,
            "source_name": None,
            "sampled_at": None,
            "age_seconds": None,
            "measurement": None,
            "error": snapshot["error"],
        }
    candidates = []
    for device in snapshot["devices"].values():
        telemetry = device.get("telemetry", {})
        voltage = finite_number(telemetry.get("output_voltage_v"))
        measurement = "output_voltage_v"
        if voltage is None or not 100 <= float(voltage) <= 260:
            voltage = finite_number(telemetry.get("grid_voltage_v"))
            measurement = "grid_voltage_v"
        age = finite_number(device.get("age_seconds"))
        if (
            voltage is not None
            and 100 <= float(voltage) <= 260
            and age is not None
            and float(age) <= INVERTER_VOLTAGE_MAX_AGE_SECONDS
        ):
            candidates.append((float(age), float(voltage), measurement, device))
    if not candidates:
        return {
            "available": False,
            "voltage_v": None,
            "source_id": None,
            "source_name": None,
            "sampled_at": None,
            "age_seconds": None,
            "measurement": None,
            "error": "No fresh inverter AC-voltage sample is available",
        }
    age, voltage, measurement, device = min(candidates, key=lambda item: item[0])
    return {
        "available": True,
        "voltage_v": voltage,
        "source_id": device["id"],
        "source_name": device["name"],
        "sampled_at": device["last_sample"],
        "age_seconds": age,
        "measurement": measurement,
        "error": None,
    }


def read_inverter_voltage_reference(database: Path) -> dict[str, Any]:
    """Compatibility wrapper for reports/tests that still inspect stored samples."""
    return inverter_voltage_reference_from_snapshot(
        read_inverter_snapshot(
            database,
            stale_after_seconds=INVERTER_VOLTAGE_MAX_AGE_SECONDS,
        )
    )


def read_inverter_target(database: Path, inverter_id: str) -> dict[str, Any]:
    """Resolve one configured inverter from the collector's local inventory."""
    if not INVERTER_ID_PATTERN.fullmatch(inverter_id):
        raise ValueError("invalid inverter id")
    database_uri = f"{database.expanduser().resolve().as_uri()}?mode=ro"
    with sqlite3.connect(database_uri, uri=True, timeout=2) as connection:
        connection.row_factory = sqlite3.Row
        row = connection.execute(
            """
            SELECT inverter_id, name, protocol, configured_ip, local_ip, local_port
            FROM inverters
            WHERE inverter_id = ?
            """,
            (inverter_id,),
        ).fetchone()
    if row is None:
        raise KeyError(inverter_id)
    return {
        "id": str(row["inverter_id"]),
        "name": str(row["name"]),
        "protocol": str(row["protocol"]),
        "ip": str(row["configured_ip"]),
        "local_ip": str(row["local_ip"]),
        "local_port": int(row["local_port"]),
    }


@dataclass(frozen=True)
class DashboardConfig:
    host: str = "0.0.0.0"
    port: int = 8765
    poll_interval_seconds: float = 20.0
    protocol_interval_seconds: float = 300.0
    telemetry_database: Path = DEFAULT_TELEMETRY_DATABASE
    inverter_stale_seconds: float = DEFAULT_INVERTER_STALE_SECONDS
    inverter_inventory: Path = DEFAULT_INVERTER_INVENTORY
    inverter_poll_interval_seconds: float = DEFAULT_INVERTER_POLL_SECONDS
    iot_poll_interval_seconds: float = 10.0
    plug_inventory: Path = Path.home() / ".config" / "iot-keys" / "xiaomi_plugs.json"
    ac_configuration: Path = Path.home() / ".config" / "iot-keys" / "tuya_ac.json"

    @classmethod
    def from_env_and_args(cls, args: argparse.Namespace) -> "DashboardConfig":
        host = args.host or os.environ.get("BMS_DASHBOARD_HOST", cls.host)
        port = args.port or int(os.environ.get("BMS_DASHBOARD_PORT", cls.port))
        poll_interval = args.poll_interval or float(
            os.environ.get("BMS_DASHBOARD_POLL_INTERVAL", cls.poll_interval_seconds)
        )
        protocol_interval = float(
            os.environ.get("BMS_DASHBOARD_PROTOCOL_INTERVAL", cls.protocol_interval_seconds)
        )
        database_value = os.environ.get(
            "BMS_DASHBOARD_TELEMETRY_DATABASE", str(DEFAULT_TELEMETRY_DATABASE)
        )
        database = Path(database_value).expanduser()
        if not database.is_absolute():
            database = PROJECT_ROOT / database
        inverter_stale_seconds = float(
            os.environ.get(
                "BMS_DASHBOARD_INVERTER_STALE_SECONDS",
                DEFAULT_INVERTER_STALE_SECONDS,
            )
        )
        inverter_inventory = Path(
            os.environ.get(
                "BMS_DASHBOARD_INVERTER_INVENTORY", str(cls.inverter_inventory)
            )
        ).expanduser()
        inverter_poll_interval_seconds = float(
            os.environ.get(
                "BMS_DASHBOARD_INVERTER_POLL_INTERVAL",
                cls.inverter_poll_interval_seconds,
            )
        )
        iot_poll_interval_seconds = float(
            os.environ.get("BMS_DASHBOARD_IOT_POLL_INTERVAL", cls.iot_poll_interval_seconds)
        )
        plug_inventory = Path(
            os.environ.get("BMS_DASHBOARD_PLUG_INVENTORY", str(cls.plug_inventory))
        ).expanduser()
        ac_configuration = Path(
            os.environ.get("BMS_DASHBOARD_AC_CONFIGURATION", str(cls.ac_configuration))
        ).expanduser()
        if not 1 <= port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if poll_interval < 5:
            raise ValueError("poll interval must be at least 5 seconds")
        if protocol_interval < 30:
            raise ValueError("protocol interval must be at least 30 seconds")
        if inverter_stale_seconds < 30:
            raise ValueError("inverter stale interval must be at least 30 seconds")
        if inverter_poll_interval_seconds < 10:
            raise ValueError("inverter poll interval must be at least 10 seconds")
        if iot_poll_interval_seconds < 5:
            raise ValueError("IoT poll interval must be at least 5 seconds")
        return cls(
            host=host,
            port=port,
            poll_interval_seconds=poll_interval,
            protocol_interval_seconds=protocol_interval,
            telemetry_database=database.resolve(),
            inverter_stale_seconds=inverter_stale_seconds,
            inverter_inventory=inverter_inventory.resolve(),
            inverter_poll_interval_seconds=inverter_poll_interval_seconds,
            iot_poll_interval_seconds=iot_poll_interval_seconds,
            plug_inventory=plug_inventory.resolve(),
            ac_configuration=ac_configuration.resolve(),
        )


class DashboardState:
    """Own the single BLE polling loop and cache shared by all web clients."""

    def __init__(
        self,
        config: DashboardConfig,
        telemetry_reader: TelemetryReader = bms_ble.read_telemetry,
        jk_protocol_reader: ProtocolReader = bms_ble.read_jk_protocols,
        seplos_protocol_reader: ProtocolReader = bms_ble.read_seplos_protocol,
        jk_protocol_writer: JkProtocolWriter = bms_ble.set_jk_protocol,
        seplos_protocol_writer: SeplosProtocolWriter = bms_ble.set_seplos_protocol,
        bms_configuration_reader: BmsConfigurationReader = bms_config.read_bms_configuration,
        bms_setting_writer: BmsSettingWriter = bms_config.write_bms_setting,
        inverter_reader: InverterReader = inverter_protocols.read_inverter,
        plug_reader: PlugReader = miot_plugs.read_plug,
        plug_power_writer: PlugPowerWriter = miot_plugs.write_power,
        ac_reader: AcReader = tuya_ac.read_ac,
        ac_power_writer: AcPowerWriter = tuya_ac.write_power,
        ac_setting_writer: AcSettingWriter = tuya_ac.write_setting,
        inverter_inventory: Mapping[str, Mapping[str, Any]] | None = None,
        inverter_inventory_error: str | None = None,
        plug_inventory: Mapping[str, miot_plugs.PlugDefinition] | None = None,
        ac_definition: tuya_ac.AcDefinition | None = None,
    ) -> None:
        self.config = config
        self.telemetry_reader = telemetry_reader
        self.jk_protocol_reader = jk_protocol_reader
        self.seplos_protocol_reader = seplos_protocol_reader
        self.jk_protocol_writer = jk_protocol_writer
        self.seplos_protocol_writer = seplos_protocol_writer
        self.bms_configuration_reader = bms_configuration_reader
        self.bms_setting_writer = bms_setting_writer
        self.inverter_reader = inverter_reader
        self.plug_reader = plug_reader
        self.plug_power_writer = plug_power_writer
        self.ac_reader = ac_reader
        self.ac_power_writer = ac_power_writer
        self.ac_setting_writer = ac_setting_writer
        self.operation_lock = asyncio.Lock()
        self.inverter_operation_lock = asyncio.Lock()
        self.iot_operation_lock = asyncio.Lock()
        self.refresh_event = asyncio.Event()
        self.iot_refresh_event = asyncio.Event()
        self.poll_task: asyncio.Task[None] | None = None
        self.iot_poll_task: asyncio.Task[None] | None = None
        self.polling = False
        self.iot_polling = False
        self.generation = 0
        self.iot_generation = 0
        self.last_poll_started: str | None = None
        self.last_poll_finished: str | None = None
        self.iot_last_poll_started: str | None = None
        self.iot_last_poll_finished: str | None = None
        self.last_protocol_monotonic = 0.0
        self.last_inverter_poll_monotonic = 0.0
        self.last_refresh_request_monotonic = 0.0
        self.last_iot_refresh_request_monotonic = 0.0
        self.devices: dict[str, dict[str, Any]] = {}
        self.history: dict[str, deque[dict[str, Any]]] = {}
        self.protocols: dict[str, Any] = {}
        for alias, inventory in bms_ble.DEVICE_INVENTORY.items():
            self.devices[alias] = {
                "alias": alias,
                "address": inventory["address"],
                "advertised_name": inventory["advertised_name"],
                "online": False,
                "updating": False,
                "last_attempt": None,
                "last_success": None,
                "error": "Waiting for first Bluetooth poll",
                "info": {},
                "telemetry": {},
            }
            self.history[alias] = deque(maxlen=HISTORY_LIMIT)

        self.inverter_inventory = {
            inverter_id: dict(target)
            for inverter_id, target in (inverter_inventory or {}).items()
        }
        self.inverter_inventory_error = inverter_inventory_error
        self.inverters: dict[str, dict[str, Any]] = {}
        for inverter_id, target in self.inverter_inventory.items():
            self.inverters[inverter_id] = {
                "id": inverter_id,
                "name": target["name"],
                "mac": target.get("mac", ""),
                "linked_board_id": target.get("linked_board_id"),
                "configured_protocol": target["protocol"],
                "configured_ip": target["ip"],
                "source_ip": target["ip"],
                "online": False,
                "stale": True,
                "updating": False,
                "age_seconds": None,
                "last_attempt": None,
                "last_sample": None,
                "error": "Waiting for first direct inverter poll",
                "telemetry": {},
            }

        self.plug_config_error: str | None = None
        if plug_inventory is None:
            try:
                plug_inventory = miot_plugs.load_inventory(config.plug_inventory)
            except miot_plugs.MiotPlugError as exc:
                LOGGER.warning("smart-plug inventory unavailable: %s", exc)
                self.plug_config_error = str(exc)
                plug_inventory = {}
        self.plug_inventory = dict(plug_inventory)
        self.inverter_voltage_reference: dict[str, Any] = {
            "available": False,
            "voltage_v": None,
            "source_id": None,
            "source_name": None,
            "sampled_at": None,
            "age_seconds": None,
            "measurement": None,
            "error": "Waiting for inverter AC-voltage telemetry",
        }
        self.plugs: dict[str, dict[str, Any]] = {}
        self.plug_history: dict[str, deque[dict[str, Any]]] = {}
        for plug_id, definition in self.plug_inventory.items():
            self.plugs[plug_id] = {
                **definition.public_identity(),
                "online": False,
                "updating": False,
                "last_attempt": None,
                "last_success": None,
                "error": "Waiting for first local-network poll",
                "telemetry": {},
            }
            self.plug_history[plug_id] = deque(maxlen=IOT_HISTORY_LIMIT)

        self.ac_config_error: str | None = None
        if ac_definition is None:
            try:
                ac_definition = tuya_ac.load_config(config.ac_configuration)
            except tuya_ac.TuyaAcError as exc:
                LOGGER.warning("air-conditioner configuration unavailable: %s", exc)
                self.ac_config_error = str(exc)
        self.ac_definition = ac_definition
        self.air_conditioner: dict[str, Any] = {
            **(ac_definition.public_identity() if ac_definition else {
                "id": "air-conditioner",
                "name": "Air conditioner",
                "ip": None,
                "product_id": None,
                "version": None,
                "controls": {
                    "modes": list(tuya_ac.DEFAULT_MODE_VALUES),
                    "fan_speeds": list(tuya_ac.DEFAULT_FAN_VALUES),
                    "temperature": {
                        "minimum_c": 16.0,
                        "maximum_c": 30.0,
                        "step_c": 1.0,
                    },
                },
            }),
            "configured": ac_definition is not None,
            "online": False,
            "updating": False,
            "last_attempt": None,
            "last_success": None,
            "error": self.ac_config_error or "Waiting for first local-network poll",
            "telemetry": {},
        }

    def record_success(self, alias: str, result: Mapping[str, Any], timestamp: str) -> None:
        device = self.devices[alias]
        telemetry = copy.deepcopy(result.get("telemetry", {}))
        device.update(
            {
                "online": True,
                "updating": False,
                "last_attempt": timestamp,
                "last_success": timestamp,
                "error": None,
                "info": copy.deepcopy(result.get("info", {})),
                "telemetry": telemetry,
            }
        )
        self.history[alias].append(history_sample(timestamp, telemetry))

    def record_error(self, alias: str, error: BaseException, timestamp: str) -> None:
        self.devices[alias].update(
            {
                "online": False,
                "updating": False,
                "last_attempt": timestamp,
                "error": f"{type(error).__name__}: {error}",
            }
        )

    async def _poll_telemetry_locked(self) -> None:
        for alias in self.devices:
            timestamp = utc_now()
            self.devices[alias]["updating"] = True
            self.devices[alias]["last_attempt"] = timestamp
            try:
                result = await self.telemetry_reader(alias)
            except Exception as exc:  # BLE absence must not stop polling the other BMS devices.
                LOGGER.warning("%s telemetry failed: %s", alias, exc)
                self.record_error(alias, exc, utc_now())
            else:
                self.record_success(alias, result, utc_now())

    async def _poll_protocols_locked(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_protocol_monotonic < self.config.protocol_interval_seconds:
            return
        for alias, reader in (("jk", self.jk_protocol_reader), ("seplos", self.seplos_protocol_reader)):
            try:
                self.protocols[alias] = {"data": await reader(), "updated_at": utc_now(), "error": None}
            except Exception as exc:
                LOGGER.warning("%s protocol read failed: %s", alias, exc)
                previous = self.protocols.get(alias, {})
                self.protocols[alias] = {
                    "data": previous.get("data"),
                    "updated_at": previous.get("updated_at"),
                    "error": f"{type(exc).__name__}: {exc}",
                }
        self.protocols["daly"] = {
            "data": None,
            "updated_at": utc_now(),
            "error": None,
            "note": "No verified inverter-protocol selector is exposed by this Daly BLE firmware.",
        }
        self.last_protocol_monotonic = time.monotonic()

    async def poll_once(self, include_protocols: bool = True) -> bool:
        if self.operation_lock.locked():
            return False
        async with self.operation_lock:
            self.polling = True
            self.last_poll_started = utc_now()
            try:
                await self._poll_telemetry_locked()
                if include_protocols:
                    await self._poll_protocols_locked()
            finally:
                self.polling = False
                self.last_poll_finished = utc_now()
                self.generation += 1
        return True

    async def polling_loop(self) -> None:
        while True:
            await self.poll_once()
            try:
                await asyncio.wait_for(self.refresh_event.wait(), timeout=self.config.poll_interval_seconds)
            except asyncio.TimeoutError:
                pass
            self.refresh_event.clear()

    def request_refresh(self) -> bool:
        now = time.monotonic()
        if now - self.last_refresh_request_monotonic < MIN_REFRESH_SECONDS:
            return False
        self.last_refresh_request_monotonic = now
        self.refresh_event.set()
        return True

    async def refresh_protocols(self) -> bool:
        if self.operation_lock.locked():
            return False
        async with self.operation_lock:
            await self._poll_protocols_locked(force=True)
            self.generation += 1
        return True

    async def change_jk_protocol(self, interface: str, protocol: str, confirmation: str) -> dict[str, Any]:
        async with self.operation_lock:
            result = await self.jk_protocol_writer(interface, protocol, confirmation)
            self.protocols["jk"] = {
                "data": {"device": "jk", "address": bms_ble.DEVICE_INVENTORY["jk"]["address"], **result["after"]},
                "updated_at": utc_now(),
                "error": None,
            }
            self.generation += 1
        self.refresh_event.set()
        return result

    async def change_seplos_protocol(self, profile: str, confirmation: str) -> dict[str, Any]:
        async with self.operation_lock:
            result = await self.seplos_protocol_writer(profile, confirmation)
            self.protocols["seplos"] = {
                "data": {
                    "device": "seplos",
                    "address": bms_ble.DEVICE_INVENTORY["seplos"]["address"],
                    "identity": copy.deepcopy(result["identity"]),
                    "inverter_protocol": copy.deepcopy(result["after"]),
                },
                "updated_at": utc_now(),
                "error": None,
            }
            self.generation += 1
        self.refresh_event.set()
        return result

    async def read_bms_configuration(self, alias: str) -> dict[str, Any]:
        async with self.operation_lock:
            return await self.bms_configuration_reader(alias)

    async def write_bms_setting(
        self, alias: str, setting: str, value: Any, confirmation: str
    ) -> dict[str, Any]:
        async with self.operation_lock:
            result = await self.bms_setting_writer(alias, setting, value, confirmation)
            self.generation += 1
        self.refresh_event.set()
        return result

    def record_inverter_success(
        self, inverter_id: str, telemetry: Mapping[str, Any], timestamp: str
    ) -> None:
        self.inverters[inverter_id].update(
            {
                "online": True,
                "stale": False,
                "updating": False,
                "age_seconds": 0.0,
                "last_attempt": timestamp,
                "last_sample": timestamp,
                "error": None,
                "telemetry": copy.deepcopy(dict(telemetry)),
            }
        )

    def record_inverter_error(
        self, inverter_id: str, error: BaseException, timestamp: str
    ) -> None:
        self.inverters[inverter_id].update(
            {
                "online": False,
                "updating": False,
                "last_attempt": timestamp,
                "error": f"{type(error).__name__}: {error}",
            }
        )

    async def poll_inverters_once(self, force: bool = False) -> bool:
        if not self.inverter_inventory or self.inverter_operation_lock.locked():
            return False
        now = time.monotonic()
        if (
            not force
            and now - self.last_inverter_poll_monotonic
            < self.config.inverter_poll_interval_seconds
        ):
            return False
        async with self.inverter_operation_lock:
            for inverter_id, target in self.inverter_inventory.items():
                timestamp = utc_now()
                self.inverters[inverter_id]["updating"] = True
                self.inverters[inverter_id]["last_attempt"] = timestamp
                try:
                    telemetry = await asyncio.to_thread(
                        self.inverter_reader,
                        target["protocol"],
                        target["ip"],
                        target["local_ip"],
                        target["local_port"],
                        INVERTER_CONTROL_TIMEOUT_SECONDS,
                    )
                except Exception as exc:
                    LOGGER.warning("%s direct telemetry failed: %s", inverter_id, exc)
                    self.record_inverter_error(inverter_id, exc, utc_now())
                else:
                    self.record_inverter_success(inverter_id, telemetry, utc_now())
        self.last_inverter_poll_monotonic = time.monotonic()
        return True

    def public_inverter_snapshot(self) -> dict[str, Any]:
        devices = copy.deepcopy(self.inverters)
        now = dt.datetime.now(dt.timezone.utc)
        for device in devices.values():
            sampled_at = device.get("last_sample")
            age_seconds: float | None = None
            if sampled_at:
                try:
                    sampled = dt.datetime.fromisoformat(
                        str(sampled_at).replace("Z", "+00:00")
                    )
                    age_seconds = max(0.0, (now - sampled).total_seconds())
                except ValueError:
                    pass
            stale = age_seconds is None or age_seconds > self.config.inverter_stale_seconds
            device["age_seconds"] = age_seconds
            device["stale"] = stale
            device["online"] = bool(device["online"] and not stale and not device["error"])
            if stale and device["error"] is None:
                device["error"] = (
                    "No direct inverter sample is available"
                    if age_seconds is None
                    else f"Latest direct sample is stale ({age_seconds:.0f}s old)"
                )
        return {
            "available": bool(self.inverter_inventory),
            "source": "direct_lan",
            "schema_version": None,
            "stale_after_seconds": self.config.inverter_stale_seconds,
            "poll_interval_seconds": self.config.inverter_poll_interval_seconds,
            "error": self.inverter_inventory_error,
            "devices": devices,
        }

    def record_plug_success(
        self, plug_id: str, result: Mapping[str, Any], timestamp: str
    ) -> None:
        telemetry = copy.deepcopy(result.get("telemetry", {}))
        self.plugs[plug_id].update(
            {
                "online": True,
                "updating": False,
                "last_attempt": timestamp,
                "last_success": timestamp,
                "error": None,
                "telemetry": telemetry,
            }
        )
        self.plug_history[plug_id].append(
            {
                "timestamp": timestamp,
                "electric_power_w": finite_number(telemetry.get("electric_power_w")),
                "energy_counter": finite_number(telemetry.get("energy_counter")),
                "estimated_current_a": finite_number(
                    telemetry.get("estimated_current_a")
                ),
                "voltage_v": finite_number(telemetry.get("voltage_v")),
                "voltage_source": telemetry.get("voltage_source"),
                "voltage_source_name": telemetry.get("voltage_source_name"),
                "on": telemetry.get("on") if isinstance(telemetry.get("on"), bool) else None,
            }
        )

    def record_plug_error(self, plug_id: str, error: BaseException, timestamp: str) -> None:
        self.plugs[plug_id].update(
            {
                "online": False,
                "updating": False,
                "last_attempt": timestamp,
                "error": f"{type(error).__name__}: {error}",
            }
        )

    def apply_inverter_voltage_reference(
        self, plug_id: str, result: dict[str, Any]
    ) -> dict[str, Any]:
        definition = self.plug_inventory[plug_id]
        reference = self.inverter_voltage_reference
        if reference["available"]:
            miot_plugs.apply_voltage_reference(
                result.setdefault("telemetry", {}),
                reference["voltage_v"],
                "inverter_output" if reference["measurement"] == "output_voltage_v" else "inverter_grid",
                reference["source_name"],
                reference["sampled_at"],
            )
        else:
            miot_plugs.apply_voltage_reference(
                result.setdefault("telemetry", {}),
                definition.reference_voltage_v,
                "configured_fallback",
            )
        return result

    async def _poll_one_plug(self, plug_id: str) -> None:
        timestamp = utc_now()
        self.plugs[plug_id]["updating"] = True
        self.plugs[plug_id]["last_attempt"] = timestamp
        try:
            result = await asyncio.to_thread(self.plug_reader, self.plug_inventory[plug_id])
        except Exception as exc:
            LOGGER.warning("%s smart-plug telemetry failed: %s", plug_id, exc)
            self.record_plug_error(plug_id, exc, utc_now())
        else:
            self.apply_inverter_voltage_reference(plug_id, result)
            self.record_plug_success(plug_id, result, utc_now())

    async def _poll_air_conditioner(self) -> None:
        if self.ac_definition is None:
            return
        timestamp = utc_now()
        self.air_conditioner["updating"] = True
        self.air_conditioner["last_attempt"] = timestamp
        try:
            result = await asyncio.to_thread(self.ac_reader, self.ac_definition)
        except Exception as exc:
            LOGGER.warning("air-conditioner telemetry failed: %s", exc)
            self.air_conditioner.update(
                {
                    "online": False,
                    "updating": False,
                    "last_attempt": utc_now(),
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )
        else:
            timestamp = utc_now()
            self.air_conditioner.update(
                {
                    **result,
                    "configured": True,
                    "online": True,
                    "updating": False,
                    "last_attempt": timestamp,
                    "last_success": timestamp,
                    "error": None,
                }
            )

    async def poll_iot_once(self) -> bool:
        if self.iot_operation_lock.locked():
            return False
        async with self.iot_operation_lock:
            self.iot_polling = True
            self.iot_last_poll_started = utc_now()
            try:
                await self.poll_inverters_once()
                self.inverter_voltage_reference = inverter_voltage_reference_from_snapshot(
                    self.public_inverter_snapshot()
                )
                operations = [self._poll_one_plug(plug_id) for plug_id in self.plugs]
                if self.ac_definition is not None:
                    operations.append(self._poll_air_conditioner())
                if operations:
                    await asyncio.gather(*operations)
            finally:
                self.iot_polling = False
                self.iot_last_poll_finished = utc_now()
                self.iot_generation += 1
        return True

    async def iot_polling_loop(self) -> None:
        while True:
            await self.poll_iot_once()
            try:
                await asyncio.wait_for(
                    self.iot_refresh_event.wait(),
                    timeout=self.config.iot_poll_interval_seconds,
                )
            except asyncio.TimeoutError:
                pass
            self.iot_refresh_event.clear()

    def request_iot_refresh(self) -> bool:
        now = time.monotonic()
        if now - self.last_iot_refresh_request_monotonic < MIN_REFRESH_SECONDS:
            return False
        self.last_iot_refresh_request_monotonic = now
        self.iot_refresh_event.set()
        return True

    async def set_plug_power(self, plug_id: str, enabled: bool) -> dict[str, Any]:
        definition = self.plug_inventory.get(plug_id)
        if definition is None:
            raise KeyError(plug_id)
        async with self.iot_operation_lock:
            result = await asyncio.to_thread(self.plug_power_writer, definition, enabled)
            self.apply_inverter_voltage_reference(plug_id, result["device"])
            self.record_plug_success(plug_id, result["device"], utc_now())
            self.iot_generation += 1
        self.iot_refresh_event.set()
        return result

    async def set_ac_power(self, enabled: bool) -> dict[str, Any]:
        if self.ac_definition is None:
            raise LookupError("air conditioner is not configured")
        async with self.iot_operation_lock:
            result = await asyncio.to_thread(
                self.ac_power_writer, self.ac_definition, enabled
            )
            timestamp = utc_now()
            self.air_conditioner.update(
                {
                    **result["device"],
                    "configured": True,
                    "online": True,
                    "updating": False,
                    "last_attempt": timestamp,
                    "last_success": timestamp,
                    "error": None,
                }
            )
            self.iot_generation += 1
        self.iot_refresh_event.set()
        return result

    async def set_ac_setting(self, setting: str, value: Any) -> dict[str, Any]:
        if self.ac_definition is None:
            raise LookupError("air conditioner is not configured")
        async with self.iot_operation_lock:
            result = await asyncio.to_thread(
                self.ac_setting_writer, self.ac_definition, setting, value
            )
            timestamp = utc_now()
            self.air_conditioner.update(
                {
                    **result["device"],
                    "configured": True,
                    "online": True,
                    "updating": False,
                    "last_attempt": timestamp,
                    "last_success": timestamp,
                    "error": None,
                }
            )
            self.iot_generation += 1
        self.iot_refresh_event.set()
        return result

    def public_iot_snapshot(self) -> dict[str, Any]:
        return {
            "polling": self.iot_polling,
            "generation": self.iot_generation,
            "poll_interval_seconds": self.config.iot_poll_interval_seconds,
            "last_poll_started": self.iot_last_poll_started,
            "last_poll_finished": self.iot_last_poll_finished,
            "inverter_voltage_reference": copy.deepcopy(self.inverter_voltage_reference),
            "plugs": {
                "available": bool(self.plug_inventory),
                "configured": bool(self.plug_inventory),
                "error": self.plug_config_error,
                "devices": copy.deepcopy(self.plugs),
                "history": {
                    plug_id: list(samples)
                    for plug_id, samples in self.plug_history.items()
                },
            },
            "air_conditioner": copy.deepcopy(self.air_conditioner),
        }

    def public_snapshot(self) -> dict[str, Any]:
        snapshot = {
            "server_time": utc_now(),
            "generation": self.generation,
            "polling": self.polling,
            "poll_interval_seconds": self.config.poll_interval_seconds,
            "last_poll_started": self.last_poll_started,
            "last_poll_finished": self.last_poll_finished,
            "control_enabled": True,
            "control_auth_required": False,
            "seplos_protocol_profiles": [
                {"name": name, **copy.deepcopy(profile)}
                for name, profile in bms_ble.SEPLOS_PROTOCOLS.items()
            ],
            "devices": copy.deepcopy(self.devices),
            "history": {alias: list(samples) for alias, samples in self.history.items()},
            "protocols": copy.deepcopy(self.protocols),
            "capabilities": {
                "daly": {"telemetry": True, "protocol_read": False, "protocol_write": False},
                "seplos": {"telemetry": True, "protocol_read": True, "protocol_write": True},
                "jk": {"telemetry": True, "protocol_read": True, "protocol_write": True},
            },
        }
        snapshot["iot"] = self.public_iot_snapshot()
        snapshot["inverters"] = self.public_inverter_snapshot()
        return snapshot


@web.middleware
async def response_headers(request: web.Request, handler: Callable[[web.Request], Awaitable[web.StreamResponse]]) -> web.StreamResponse:
    try:
        response = await handler(request)
    except web.HTTPException as exc:
        response = exc
    response.headers["Cache-Control"] = "no-store"
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["X-Frame-Options"] = "DENY"
    response.headers["Referrer-Policy"] = "no-referrer"
    response.headers["Content-Security-Policy"] = (
        "default-src 'self'; style-src 'self' 'unsafe-inline'; script-src 'self'; "
        "img-src 'self' data:; connect-src 'self'"
    )
    return response


def dashboard_state(request: web.Request) -> DashboardState:
    return request.app[STATE_KEY]


async def index_handler(_request: web.Request) -> web.FileResponse:
    return web.FileResponse(STATIC_ROOT / "index.html")


async def status_handler(request: web.Request) -> web.Response:
    return web.json_response(dashboard_state(request).public_snapshot())


async def inverters_handler(request: web.Request) -> web.Response:
    return web.json_response(dashboard_state(request).public_inverter_snapshot())


async def iot_handler(request: web.Request) -> web.Response:
    return web.json_response(dashboard_state(request).public_iot_snapshot())


async def plug_power_handler(request: web.Request) -> web.Response:
    state = dashboard_state(request)
    plug_id = request.match_info.get("plug_id", "")
    if plug_id not in state.plug_inventory:
        raise web.HTTPNotFound(text="unknown smart plug")
    try:
        body = await request.json()
    except (ValueError, TypeError) as exc:
        raise web.HTTPBadRequest(text="invalid JSON body") from exc
    enabled = body.get("on")
    if not isinstance(enabled, bool):
        raise web.HTTPBadRequest(text="on must be a boolean")
    LOGGER.warning(
        "smart-plug power requested: plug=%s on=%s peer=%s",
        plug_id,
        enabled,
        request.remote or "unknown",
    )
    try:
        result = await state.set_plug_power(plug_id, enabled)
    except ValueError as exc:
        raise web.HTTPBadRequest(text=str(exc)) from exc
    except (OSError, miot_plugs.MiotPlugError) as exc:
        LOGGER.error("smart-plug command failed: plug=%s reason=%s", plug_id, exc)
        raise web.HTTPBadGateway(text=f"smart-plug command failed: {exc}") from exc
    return web.json_response({"ok": True, "result": result})


async def ac_power_handler(request: web.Request) -> web.Response:
    state = dashboard_state(request)
    if state.ac_definition is None:
        raise web.HTTPServiceUnavailable(text="air conditioner is not configured")
    try:
        body = await request.json()
    except (ValueError, TypeError) as exc:
        raise web.HTTPBadRequest(text="invalid JSON body") from exc
    enabled = body.get("on")
    if not isinstance(enabled, bool):
        raise web.HTTPBadRequest(text="on must be a boolean")
    LOGGER.warning(
        "air-conditioner power requested: on=%s peer=%s",
        enabled,
        request.remote or "unknown",
    )
    try:
        result = await state.set_ac_power(enabled)
    except ValueError as exc:
        raise web.HTTPBadRequest(text=str(exc)) from exc
    except (OSError, tuya_ac.TuyaAcError) as exc:
        LOGGER.error("air-conditioner command failed: %s", exc)
        raise web.HTTPBadGateway(text=f"air-conditioner command failed: {exc}") from exc
    return web.json_response({"ok": True, "result": result})


async def ac_setting_handler(request: web.Request) -> web.Response:
    state = dashboard_state(request)
    if state.ac_definition is None:
        raise web.HTTPServiceUnavailable(text="air conditioner is not configured")
    try:
        body = await request.json()
    except (ValueError, TypeError) as exc:
        raise web.HTTPBadRequest(text="invalid JSON body") from exc
    setting = str(body.get("setting", "")).strip()
    if setting not in {"mode", "fan", "target_temperature_c"}:
        raise web.HTTPBadRequest(text="unsupported air-conditioner setting")
    if "value" not in body:
        raise web.HTTPBadRequest(text="value is required")
    LOGGER.warning(
        "air-conditioner setting requested: setting=%s peer=%s",
        setting,
        request.remote or "unknown",
    )
    try:
        result = await state.set_ac_setting(setting, body["value"])
    except ValueError as exc:
        raise web.HTTPBadRequest(text=str(exc)) from exc
    except (OSError, tuya_ac.TuyaAcError) as exc:
        LOGGER.error("air-conditioner setting failed: setting=%s reason=%s", setting, exc)
        raise web.HTTPBadGateway(text=f"air-conditioner setting failed: {exc}") from exc
    return web.json_response({"ok": True, "result": result})


def bms_alias_for_request(request: web.Request) -> str:
    alias = request.match_info.get("alias", "")
    if alias not in bms_ble.DEVICE_INVENTORY:
        raise web.HTTPNotFound(text="unknown BMS")
    return alias


async def bms_configuration_handler(request: web.Request) -> web.Response:
    state = dashboard_state(request)
    alias = bms_alias_for_request(request)
    try:
        configuration = await state.read_bms_configuration(alias)
    except ValueError as exc:
        raise web.HTTPBadRequest(text=str(exc)) from exc
    except TimeoutError as exc:
        raise web.HTTPGatewayTimeout(text=f"Bluetooth configuration read timed out: {exc}") from exc
    except Exception as exc:
        LOGGER.error("BMS configuration read failed: alias=%s reason=%s", alias, exc)
        raise web.HTTPBadGateway(text=f"Bluetooth configuration read failed: {exc}") from exc
    return web.json_response({"ok": True, "configuration": configuration})


async def bms_setting_handler(request: web.Request) -> web.Response:
    state = dashboard_state(request)
    alias = bms_alias_for_request(request)
    try:
        body = await request.json()
    except (ValueError, TypeError) as exc:
        raise web.HTTPBadRequest(text="invalid JSON body") from exc
    setting = str(body.get("setting", "")).strip()
    confirmation = str(body.get("confirmation", "")).strip()
    if not setting or "value" not in body:
        raise web.HTTPBadRequest(text="setting and value are required")
    LOGGER.warning(
        "BMS configuration write requested: alias=%s setting=%s peer=%s",
        alias, setting, request.remote or "unknown",
    )
    try:
        result = await state.write_bms_setting(alias, setting, body["value"], confirmation)
    except ValueError as exc:
        LOGGER.warning("BMS write rejected: alias=%s setting=%s reason=%s", alias, setting, exc)
        raise web.HTTPBadRequest(text=str(exc)) from exc
    except TimeoutError as exc:
        LOGGER.error("BMS write timed out: alias=%s setting=%s", alias, setting)
        raise web.HTTPGatewayTimeout(text=f"Bluetooth configuration write timed out: {exc}") from exc
    except Exception as exc:
        LOGGER.error("BMS write failed: alias=%s setting=%s reason=%s", alias, setting, exc)
        raise web.HTTPBadGateway(text=f"Bluetooth configuration write failed: {exc}") from exc
    LOGGER.warning(
        "BMS configuration write completed: alias=%s setting=%s written=%s verified=%s before=%r after=%r",
        alias, setting, result["written"], result["verified"], result["before"], result["after"],
    )
    return web.json_response({"ok": True, "result": result})


async def inverter_target_for_request(request: web.Request) -> dict[str, Any]:
    state = dashboard_state(request)
    inverter_id = request.match_info.get("inverter_id", "")
    if not INVERTER_ID_PATTERN.fullmatch(inverter_id):
        raise web.HTTPBadRequest(text="invalid inverter id")
    target = state.inverter_inventory.get(inverter_id)
    if target is None:
        raise web.HTTPNotFound(text="unknown inverter")
    return copy.deepcopy(target)


async def inverter_configuration_handler(request: web.Request) -> web.Response:
    state = dashboard_state(request)
    target = await inverter_target_for_request(request)
    if state.inverter_operation_lock.locked():
        raise web.HTTPConflict(text="another inverter configuration operation is in progress")
    try:
        async with state.inverter_operation_lock:
            configuration = await asyncio.to_thread(
                inverter_protocols.read_inverter_configuration,
                target["protocol"],
                target["ip"],
                target["local_ip"],
                target["local_port"],
                INVERTER_CONTROL_TIMEOUT_SECONDS,
            )
    except ValueError as exc:
        raise web.HTTPBadRequest(text=str(exc)) from exc
    except TimeoutError as exc:
        raise web.HTTPGatewayTimeout(text=str(exc)) from exc
    except (OSError, inverter_protocols.InverterProtocolError) as exc:
        raise web.HTTPBadGateway(text=str(exc)) from exc
    configuration["inverter"] = target
    return web.json_response({"ok": True, "configuration": configuration})


async def inverter_setting_handler(request: web.Request) -> web.Response:
    state = dashboard_state(request)
    target = await inverter_target_for_request(request)
    try:
        body = await request.json()
    except (ValueError, TypeError) as exc:
        raise web.HTTPBadRequest(text="invalid JSON body") from exc
    setting = str(body.get("setting", "")).strip()
    confirmation = str(body.get("confirmation", "")).strip()
    if not setting or "value" not in body:
        raise web.HTTPBadRequest(text="setting and value are required")
    if state.inverter_operation_lock.locked():
        raise web.HTTPConflict(text="another inverter configuration operation is in progress")
    LOGGER.warning(
        "inverter configuration write requested: inverter=%s setting=%s peer=%s",
        target["id"], setting, request.remote or "unknown",
    )
    try:
        async with state.inverter_operation_lock:
            result = await asyncio.to_thread(
                inverter_protocols.write_inverter_setting,
                target["protocol"],
                target["ip"],
                target["local_ip"],
                target["local_port"],
                INVERTER_CONTROL_TIMEOUT_SECONDS,
                setting,
                body["value"],
                confirmation,
            )
    except ValueError as exc:
        LOGGER.warning("inverter write rejected: inverter=%s setting=%s reason=%s", target["id"], setting, exc)
        raise web.HTTPBadRequest(text=str(exc)) from exc
    except TimeoutError as exc:
        LOGGER.error("inverter write timed out: inverter=%s setting=%s", target["id"], setting)
        raise web.HTTPGatewayTimeout(text=str(exc)) from exc
    except (OSError, inverter_protocols.InverterProtocolError) as exc:
        LOGGER.error("inverter write failed: inverter=%s setting=%s reason=%s", target["id"], setting, exc)
        raise web.HTTPBadGateway(text=str(exc)) from exc
    result["after_configuration"]["inverter"] = target
    LOGGER.warning(
        "inverter configuration write completed: inverter=%s setting=%s written=%s verified=%s before=%r after=%r",
        target["id"], setting, result["written"], result["verified"], result["before"], result["after"],
    )
    return web.json_response({"ok": True, "result": result})


async def health_handler(request: web.Request) -> web.Response:
    state = dashboard_state(request)
    bms_ok = state.poll_task is not None and not state.poll_task.done()
    iot_ok = state.iot_poll_task is not None and not state.iot_poll_task.done()
    return web.json_response(
        {
            "ok": bms_ok and iot_ok,
            "bms_poller_ok": bms_ok,
            "iot_poller_ok": iot_ok,
            "polling": state.polling,
            "iot_polling": state.iot_polling,
            "generation": state.generation,
            "iot_generation": state.iot_generation,
            "server_time": utc_now(),
        }
    )


async def refresh_handler(request: web.Request) -> web.Response:
    state = dashboard_state(request)
    bms_accepted = state.request_refresh()
    iot_accepted = state.request_iot_refresh()
    accepted = bms_accepted or iot_accepted
    return web.json_response(
        {
            "accepted": accepted,
            "bms_accepted": bms_accepted,
            "iot_accepted": iot_accepted,
            "reason": None if accepted else "refresh rate limited",
        }
    )


async def protocol_refresh_handler(request: web.Request) -> web.Response:
    state = dashboard_state(request)
    if not await state.refresh_protocols():
        raise web.HTTPConflict(text="another Bluetooth operation is in progress")
    return web.json_response({"ok": True, "protocols": state.public_snapshot()["protocols"]})


async def jk_protocol_handler(request: web.Request) -> web.Response:
    state = dashboard_state(request)
    try:
        body = await request.json()
    except (ValueError, TypeError) as exc:
        raise web.HTTPBadRequest(text="invalid JSON body") from exc
    interface = str(body.get("interface", ""))
    protocol = str(body.get("protocol", ""))
    confirmation = str(body.get("confirmation", ""))
    if interface not in bms_ble.JK_PROTOCOL_INTERFACES:
        raise web.HTTPBadRequest(text="invalid JK interface")
    if not protocol:
        raise web.HTTPBadRequest(text="protocol is required")
    try:
        result = await state.change_jk_protocol(interface, protocol, confirmation)
    except ValueError as exc:
        raise web.HTTPBadRequest(text=str(exc)) from exc
    except TimeoutError as exc:
        raise web.HTTPGatewayTimeout(text=str(exc)) from exc
    except RuntimeError as exc:
        raise web.HTTPBadGateway(text=str(exc)) from exc
    return web.json_response({"ok": True, "result": result})


async def seplos_protocol_handler(request: web.Request) -> web.Response:
    state = dashboard_state(request)
    try:
        body = await request.json()
    except (ValueError, TypeError) as exc:
        raise web.HTTPBadRequest(text="invalid JSON body") from exc
    profile = str(body.get("profile", ""))
    confirmation = str(body.get("confirmation", ""))
    if profile not in bms_ble.SEPLOS_PROTOCOLS:
        raise web.HTTPBadRequest(text="invalid Seplos protocol profile")
    try:
        result = await state.change_seplos_protocol(profile, confirmation)
    except ValueError as exc:
        raise web.HTTPBadRequest(text=str(exc)) from exc
    except TimeoutError as exc:
        raise web.HTTPGatewayTimeout(text=str(exc)) from exc
    except RuntimeError as exc:
        raise web.HTTPBadGateway(text=str(exc)) from exc
    return web.json_response({"ok": True, "result": result})


STATE_KEY: Final[web.AppKey[DashboardState]] = web.AppKey("dashboard_state", DashboardState)


async def start_background(app: web.Application) -> None:
    state = app[STATE_KEY]
    state.poll_task = asyncio.create_task(state.polling_loop(), name="bms-dashboard-poller")
    state.iot_poll_task = asyncio.create_task(
        state.iot_polling_loop(), name="iot-dashboard-poller"
    )


async def stop_background(app: web.Application) -> None:
    state = app[STATE_KEY]
    tasks = [task for task in (state.poll_task, state.iot_poll_task) if task is not None]
    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)


def create_app(config: DashboardConfig, state: DashboardState | None = None) -> web.Application:
    if not (STATIC_ROOT / "index.html").is_file():
        raise RuntimeError(f"dashboard static files are missing from {STATIC_ROOT}")
    if state is None:
        inverter_inventory_error: str | None = None
        try:
            inverter_inventory = load_inverter_inventory(config.inverter_inventory)
        except ValueError as exc:
            LOGGER.warning("inverter inventory unavailable: %s", exc)
            inverter_inventory = {}
            inverter_inventory_error = str(exc)
        state = DashboardState(
            config,
            inverter_inventory=inverter_inventory,
            inverter_inventory_error=inverter_inventory_error,
        )
    app = web.Application(middlewares=[response_headers], client_max_size=16 * 1024)
    app[STATE_KEY] = state
    app.router.add_get("/", index_handler)
    app.router.add_get("/api/status", status_handler)
    app.router.add_get("/api/inverters", inverters_handler)
    app.router.add_get("/api/iot", iot_handler)
    app.router.add_post("/api/iot/plugs/{plug_id}/power", plug_power_handler)
    app.router.add_post("/api/iot/air-conditioner/power", ac_power_handler)
    app.router.add_post("/api/iot/air-conditioner/setting", ac_setting_handler)
    app.router.add_get("/api/bms/{alias}/configuration", bms_configuration_handler)
    app.router.add_post("/api/bms/{alias}/setting", bms_setting_handler)
    app.router.add_get("/api/inverters/{inverter_id}/configuration", inverter_configuration_handler)
    app.router.add_post("/api/inverters/{inverter_id}/setting", inverter_setting_handler)
    app.router.add_get("/api/health", health_handler)
    app.router.add_post("/api/refresh", refresh_handler)
    app.router.add_post("/api/protocols/refresh", protocol_refresh_handler)
    app.router.add_post("/api/jk/protocol", jk_protocol_handler)
    app.router.add_post("/api/seplos/protocol", seplos_protocol_handler)
    app.router.add_static("/static", STATIC_ROOT, show_index=False)
    app.on_startup.append(start_background)
    app.on_cleanup.append(stop_background)
    return app


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Serve the local Bluetooth BMS dashboard over HTTP")
    parser.add_argument("--host", help="bind address (default: BMS_DASHBOARD_HOST or 0.0.0.0)")
    parser.add_argument("--port", type=int, help="HTTP port (default: BMS_DASHBOARD_PORT or 8765)")
    parser.add_argument("--poll-interval", type=float, help="telemetry polling interval in seconds")
    return parser


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    config = DashboardConfig.from_env_and_args(build_parser().parse_args())
    LOGGER.info(
        "starting on http://%s:%d (poll %.1fs, controls trusted on LAN)",
        config.host,
        config.port,
        config.poll_interval_seconds,
    )
    web.run_app(create_app(config), host=config.host, port=config.port, print=None)


if __name__ == "__main__":
    main()
