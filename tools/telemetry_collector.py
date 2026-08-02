#!/usr/bin/env python3
"""Collect ESP32 bridge telemetry into a local SQLite database."""

from __future__ import annotations

import argparse
import concurrent.futures
import dataclasses
import datetime as dt
import ipaddress
import json
import re
import signal
import sqlite3
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Iterable

try:
    import inverter_protocols
except ModuleNotFoundError:  # Imported as tools.telemetry_collector by the tests.
    from tools import inverter_protocols


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "tools" / "telemetry_boards.json"
MAC_PATTERN = re.compile(r"^(?:[0-9a-f]{2}:){5}[0-9a-f]{2}$")
MAX_RESPONSE_BYTES = 1024 * 1024

BOOLEAN_FIELDS = {
    "valid",
    "stale",
    "fake_override",
    "pack_power_valid",
    "balance_current_valid",
}

TELEMETRY_FIELDS = (
    "valid",
    "stale",
    "fake_override",
    "age_ms",
    "updated_ms",
    "source",
    "protocol",
    "current_a",
    "pack_voltage_v",
    "pack_power_w",
    "pack_power_valid",
    "balance_current_a",
    "balance_current_valid",
    "remaining_ah",
    "full_ah",
    "soc_pct",
    "soh_pct",
    "cycles",
    "cell_max_v",
    "cell_min_v",
    "cell_max_idx",
    "cell_min_idx",
    "delta_v",
    "temp_mos_c",
    "temp_t1_c",
    "temp_t2_c",
    "temp_t4_c",
    "temp_t5_c",
    "temp_count",
    "status_63",
    "deye_status_35c",
    "deye_temp_max_sensor",
    "deye_temp_min_sensor",
    "state_flags",
    "cell_count",
    "cell_avg_v",
    "cell_diff_v",
    "alarm_raw",
    "precharge_state",
    "protections",
    "alarms",
    "warnings",
)

SAMPLE_COLUMNS = (
    "board_id",
    "source_id",
    "sampled_at_utc",
    "sampled_at_unix_ms",
    "source_ip",
    *TELEMETRY_FIELDS,
    "cells_v_json",
    "payload_json",
)

INVERTER_SAMPLE_FIELDS = (
    "protocol",
    "working_mode",
    "working_mode_code",
    "grid_voltage_v",
    "grid_frequency_hz",
    "grid_power_w",
    "grid_power_source",
    "inverter_voltage_v",
    "inverter_current_a",
    "inverter_frequency_hz",
    "inverter_power_w",
    "inverter_charging_average_current_a",
    "inverter_charging_power_w",
    "output_voltage_v",
    "output_current_a",
    "output_frequency_hz",
    "output_power_w",
    "output_apparent_power_va",
    "load_pct",
    "bus_voltage_v",
    "battery_voltage_v",
    "battery_current_a",
    "battery_charge_current_a",
    "battery_discharge_current_a",
    "battery_power_w",
    "battery_soc_pct",
    "pv_voltage_v",
    "pv_current_a",
    "pv_power_w",
    "pv_charging_power_w",
    "pv_charging_average_current_a",
    "pv2_voltage_v",
    "pv2_current_a",
    "pv2_power_w",
    "inverter_temperature_c",
    "dcdc_temperature_c",
    "pv_temperature_c",
    "power_flow_status",
    "warning_bits",
    "battery_voltage_scc_v",
    "battery_voltage_offset",
    "device_status_bits",
    "device_status_bits_2",
    "eeprom_version",
)

INVERTER_SAMPLE_COLUMNS = (
    "inverter_id",
    "sampled_at_utc",
    "sampled_at_unix_ms",
    "source_ip",
    *INVERTER_SAMPLE_FIELDS,
    "payload_json",
)

INVERTER_V5_COLUMN_TYPES = {
    "grid_power_source": "TEXT",
    "inverter_voltage_v": "REAL",
    "inverter_current_a": "REAL",
    "inverter_frequency_hz": "REAL",
    "inverter_power_w": "REAL",
    "inverter_charging_average_current_a": "REAL",
    "inverter_charging_power_w": "REAL",
    "pv_charging_average_current_a": "REAL",
    "battery_voltage_scc_v": "REAL",
    "battery_voltage_offset": "TEXT",
    "device_status_bits": "TEXT",
    "device_status_bits_2": "TEXT",
    "eeprom_version": "TEXT",
}

SCHEMA_SQL = """
PRAGMA user_version = 5;

CREATE TABLE IF NOT EXISTS boards (
    board_id TEXT PRIMARY KEY,
    hostname TEXT NOT NULL UNIQUE,
    mac TEXT NOT NULL UNIQUE COLLATE NOCASE,
    last_ip TEXT,
    last_seen_utc TEXT,
    last_error TEXT,
    updated_at_utc TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS bms_sources (
    source_id TEXT PRIMARY KEY,
    board_id TEXT NOT NULL,
    name TEXT NOT NULL,
    endpoint TEXT NOT NULL,
    last_seen_utc TEXT,
    last_error TEXT,
    updated_at_utc TEXT NOT NULL,
    UNIQUE (board_id, endpoint),
    FOREIGN KEY (board_id) REFERENCES boards(board_id)
);

CREATE TABLE IF NOT EXISTS telemetry_samples (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    board_id TEXT NOT NULL,
    source_id TEXT NOT NULL,
    sampled_at_utc TEXT NOT NULL,
    sampled_at_unix_ms INTEGER NOT NULL,
    source_ip TEXT NOT NULL,
    valid INTEGER,
    stale INTEGER,
    fake_override INTEGER,
    age_ms INTEGER,
    updated_ms INTEGER,
    source TEXT,
    protocol TEXT,
    current_a REAL,
    pack_voltage_v REAL,
    pack_power_w REAL,
    pack_power_valid INTEGER,
    balance_current_a REAL,
    balance_current_valid INTEGER,
    remaining_ah REAL,
    full_ah REAL,
    soc_pct INTEGER,
    soh_pct INTEGER,
    cycles INTEGER,
    cell_max_v REAL,
    cell_min_v REAL,
    cell_max_idx INTEGER,
    cell_min_idx INTEGER,
    delta_v REAL,
    temp_mos_c REAL,
    temp_t1_c REAL,
    temp_t2_c REAL,
    temp_t4_c REAL,
    temp_t5_c REAL,
    temp_count INTEGER,
    status_63 INTEGER,
    deye_status_35c INTEGER,
    deye_temp_max_sensor INTEGER,
    deye_temp_min_sensor INTEGER,
    state_flags TEXT,
    cell_count INTEGER,
    cell_avg_v REAL,
    cell_diff_v REAL,
    alarm_raw INTEGER,
    precharge_state INTEGER,
    protections TEXT,
    alarms TEXT,
    warnings TEXT,
    cells_v_json TEXT NOT NULL,
    payload_json TEXT NOT NULL,
    FOREIGN KEY (board_id) REFERENCES boards(board_id),
    FOREIGN KEY (source_id) REFERENCES bms_sources(source_id)
);

CREATE INDEX IF NOT EXISTS idx_telemetry_source_time
    ON telemetry_samples(source_id, sampled_at_unix_ms DESC);

CREATE INDEX IF NOT EXISTS idx_telemetry_time
    ON telemetry_samples(sampled_at_unix_ms DESC);

CREATE VIEW IF NOT EXISTS latest_telemetry AS
SELECT samples.*
FROM telemetry_samples AS samples
JOIN (
    SELECT source_id, MAX(id) AS latest_id
    FROM telemetry_samples
    GROUP BY source_id
) AS latest
ON samples.id = latest.latest_id;

CREATE TABLE IF NOT EXISTS inverters (
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
    FOREIGN KEY (linked_board_id) REFERENCES boards(board_id)
);

CREATE TABLE IF NOT EXISTS inverter_samples (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    inverter_id TEXT NOT NULL,
    sampled_at_utc TEXT NOT NULL,
    sampled_at_unix_ms INTEGER NOT NULL,
    source_ip TEXT NOT NULL,
    protocol TEXT NOT NULL,
    working_mode TEXT,
    working_mode_code TEXT,
    grid_voltage_v REAL,
    grid_frequency_hz REAL,
    grid_power_w REAL,
    grid_power_source TEXT,
    inverter_voltage_v REAL,
    inverter_current_a REAL,
    inverter_frequency_hz REAL,
    inverter_power_w REAL,
    inverter_charging_average_current_a REAL,
    inverter_charging_power_w REAL,
    output_voltage_v REAL,
    output_current_a REAL,
    output_frequency_hz REAL,
    output_power_w REAL,
    output_apparent_power_va REAL,
    load_pct REAL,
    bus_voltage_v REAL,
    battery_voltage_v REAL,
    battery_current_a REAL,
    battery_charge_current_a REAL,
    battery_discharge_current_a REAL,
    battery_power_w REAL,
    battery_soc_pct REAL,
    pv_voltage_v REAL,
    pv_current_a REAL,
    pv_power_w REAL,
    pv_charging_power_w REAL,
    pv_charging_average_current_a REAL,
    pv2_voltage_v REAL,
    pv2_current_a REAL,
    pv2_power_w REAL,
    inverter_temperature_c REAL,
    dcdc_temperature_c REAL,
    pv_temperature_c REAL,
    power_flow_status INTEGER,
    warning_bits TEXT,
    battery_voltage_scc_v REAL,
    battery_voltage_offset TEXT,
    device_status_bits TEXT,
    device_status_bits_2 TEXT,
    eeprom_version TEXT,
    payload_json TEXT NOT NULL,
    FOREIGN KEY (inverter_id) REFERENCES inverters(inverter_id)
);

CREATE INDEX IF NOT EXISTS idx_inverter_samples_device_time
    ON inverter_samples(inverter_id, sampled_at_unix_ms DESC);

CREATE INDEX IF NOT EXISTS idx_inverter_samples_time
    ON inverter_samples(sampled_at_unix_ms DESC);

CREATE VIEW IF NOT EXISTS latest_inverter_telemetry AS
SELECT samples.*
FROM inverter_samples AS samples
JOIN (
    SELECT inverter_id, MAX(id) AS latest_id
    FROM inverter_samples
    GROUP BY inverter_id
) AS latest
ON samples.id = latest.latest_id;
"""

SOURCES_SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS bms_sources (
    source_id TEXT PRIMARY KEY,
    board_id TEXT NOT NULL,
    name TEXT NOT NULL,
    endpoint TEXT NOT NULL,
    last_seen_utc TEXT,
    last_error TEXT,
    updated_at_utc TEXT NOT NULL,
    UNIQUE (board_id, endpoint),
    FOREIGN KEY (board_id) REFERENCES boards(board_id)
);
"""

INVERTER_SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS inverters (
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
    FOREIGN KEY (linked_board_id) REFERENCES boards(board_id)
);

CREATE TABLE IF NOT EXISTS inverter_samples (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    inverter_id TEXT NOT NULL,
    sampled_at_utc TEXT NOT NULL,
    sampled_at_unix_ms INTEGER NOT NULL,
    source_ip TEXT NOT NULL,
    protocol TEXT NOT NULL,
    working_mode TEXT,
    working_mode_code TEXT,
    grid_voltage_v REAL,
    grid_frequency_hz REAL,
    grid_power_w REAL,
    grid_power_source TEXT,
    inverter_voltage_v REAL,
    inverter_current_a REAL,
    inverter_frequency_hz REAL,
    inverter_power_w REAL,
    inverter_charging_average_current_a REAL,
    inverter_charging_power_w REAL,
    output_voltage_v REAL,
    output_current_a REAL,
    output_frequency_hz REAL,
    output_power_w REAL,
    output_apparent_power_va REAL,
    load_pct REAL,
    bus_voltage_v REAL,
    battery_voltage_v REAL,
    battery_current_a REAL,
    battery_charge_current_a REAL,
    battery_discharge_current_a REAL,
    battery_power_w REAL,
    battery_soc_pct REAL,
    pv_voltage_v REAL,
    pv_current_a REAL,
    pv_power_w REAL,
    pv_charging_power_w REAL,
    pv_charging_average_current_a REAL,
    pv2_voltage_v REAL,
    pv2_current_a REAL,
    pv2_power_w REAL,
    inverter_temperature_c REAL,
    dcdc_temperature_c REAL,
    pv_temperature_c REAL,
    power_flow_status INTEGER,
    warning_bits TEXT,
    battery_voltage_scc_v REAL,
    battery_voltage_offset TEXT,
    device_status_bits TEXT,
    device_status_bits_2 TEXT,
    eeprom_version TEXT,
    payload_json TEXT NOT NULL,
    FOREIGN KEY (inverter_id) REFERENCES inverters(inverter_id)
);

CREATE INDEX IF NOT EXISTS idx_inverter_samples_device_time
    ON inverter_samples(inverter_id, sampled_at_unix_ms DESC);

CREATE INDEX IF NOT EXISTS idx_inverter_samples_time
    ON inverter_samples(sampled_at_unix_ms DESC);

CREATE VIEW IF NOT EXISTS latest_inverter_telemetry AS
SELECT samples.*
FROM inverter_samples AS samples
JOIN (
    SELECT inverter_id, MAX(id) AS latest_id
    FROM inverter_samples
    GROUP BY inverter_id
) AS latest
ON samples.id = latest.latest_id;
"""


@dataclasses.dataclass(frozen=True)
class BoardConfig:
    board_id: str
    hostname: str
    mac: str
    static_ip: str | None = None


@dataclasses.dataclass(frozen=True)
class BmsSourceConfig:
    source_id: str
    board_id: str
    name: str
    endpoint: str


@dataclasses.dataclass(frozen=True)
class InverterConfig:
    inverter_id: str
    name: str
    protocol: str
    mac: str
    ip: str
    local_ip: str
    local_port: int
    linked_board_id: str | None = None


@dataclasses.dataclass(frozen=True)
class CollectorConfig:
    interval_seconds: float
    request_timeout_seconds: float
    lease_file: Path
    database: Path
    boards: tuple[BoardConfig, ...]
    sources: tuple[BmsSourceConfig, ...]
    inverters: tuple[InverterConfig, ...]


@dataclasses.dataclass(frozen=True)
class Lease:
    expires_at: int
    mac: str
    ip: str
    hostname: str


@dataclasses.dataclass(frozen=True)
class CollectionResult:
    board: BoardConfig
    source: BmsSourceConfig
    ip: str | None
    payload: dict[str, Any] | None
    sampled_at_utc: str | None
    sampled_at_unix_ms: int | None
    error: str | None

    @property
    def succeeded(self) -> bool:
        return self.payload is not None


@dataclasses.dataclass(frozen=True)
class InverterCollectionResult:
    inverter: InverterConfig
    payload: dict[str, Any] | None
    sampled_at_utc: str | None
    sampled_at_unix_ms: int | None
    error: str | None

    @property
    def succeeded(self) -> bool:
        return self.payload is not None


def utc_now() -> str:
    return (
        dt.datetime.now(dt.timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def normalize_mac(value: str) -> str:
    return value.strip().lower().replace("-", ":")


def resolve_project_path(value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else PROJECT_ROOT / path


def load_config(config_path: Path, database_override: str | None = None) -> CollectorConfig:
    with config_path.open("r", encoding="utf-8") as handle:
        raw = json.load(handle)

    interval = float(raw.get("interval_seconds", 30))
    timeout = float(raw.get("request_timeout_seconds", 5))
    if interval <= 0:
        raise ValueError("interval_seconds must be greater than zero")
    if timeout <= 0:
        raise ValueError("request_timeout_seconds must be greater than zero")

    boards: list[BoardConfig] = []
    seen_ids: set[str] = set()
    seen_hostnames: set[str] = set()
    seen_macs: set[str] = set()
    for item in raw.get("boards", []):
        static_ip = str(item.get("ip", "")).strip() or None
        if static_ip is not None:
            try:
                ipaddress.ip_address(static_ip)
            except ValueError as exc:
                raise ValueError(
                    f"invalid static IP for {item.get('id', 'unknown board')}: {static_ip}"
                ) from exc
        board = BoardConfig(
            board_id=str(item["id"]).strip(),
            hostname=str(item["hostname"]).strip(),
            mac=normalize_mac(str(item["mac"])),
            static_ip=static_ip,
        )
        if not board.board_id or not board.hostname:
            raise ValueError("board id and hostname cannot be empty")
        if not MAC_PATTERN.fullmatch(board.mac):
            raise ValueError(f"invalid MAC address for {board.board_id}: {board.mac}")
        if board.board_id in seen_ids:
            raise ValueError(f"duplicate board id: {board.board_id}")
        if board.hostname in seen_hostnames:
            raise ValueError(f"duplicate hostname: {board.hostname}")
        if board.mac in seen_macs:
            raise ValueError(f"duplicate MAC address: {board.mac}")
        seen_ids.add(board.board_id)
        seen_hostnames.add(board.hostname)
        seen_macs.add(board.mac)
        boards.append(board)

    if not boards:
        raise ValueError("at least one board must be configured")

    board_ids = {board.board_id for board in boards}
    raw_sources = raw.get("sources")
    if raw_sources is None:
        raw_sources = [
            {
                "id": f"{board.board_id}-bms1",
                "board_id": board.board_id,
                "name": f"{board.hostname} BMS 1",
                "endpoint": "/api/telemetry",
            }
            for board in boards
        ]

    sources: list[BmsSourceConfig] = []
    seen_source_ids: set[str] = set()
    seen_board_endpoints: set[tuple[str, str]] = set()
    for item in raw_sources:
        source = BmsSourceConfig(
            source_id=str(item["id"]).strip(),
            board_id=str(item["board_id"]).strip(),
            name=str(item["name"]).strip(),
            endpoint=str(item["endpoint"]).strip(),
        )
        if not source.source_id or not source.name:
            raise ValueError("BMS source id and name cannot be empty")
        if source.board_id not in board_ids:
            raise ValueError(
                f"unknown board_id for {source.source_id}: {source.board_id}"
            )
        if not source.endpoint.startswith("/api/") or any(
            character in source.endpoint for character in ("?", "#", " ")
        ):
            raise ValueError(
                f"invalid telemetry endpoint for {source.source_id}: {source.endpoint}"
            )
        if source.source_id in seen_source_ids:
            raise ValueError(f"duplicate BMS source id: {source.source_id}")
        board_endpoint = (source.board_id, source.endpoint)
        if board_endpoint in seen_board_endpoints:
            raise ValueError(
                f"duplicate endpoint {source.endpoint} on board {source.board_id}"
            )
        seen_source_ids.add(source.source_id)
        seen_board_endpoints.add(board_endpoint)
        sources.append(source)

    if not sources:
        raise ValueError("at least one BMS source must be configured")

    inverters: list[InverterConfig] = []
    seen_inverter_ids: set[str] = set()
    seen_inverter_macs: set[str] = set()
    for item in raw.get("inverters", []):
        linked_board_id = str(item.get("linked_board_id", "")).strip() or None
        inverter = InverterConfig(
            inverter_id=str(item["id"]).strip(),
            name=str(item["name"]).strip(),
            protocol=str(item["protocol"]).strip(),
            mac=normalize_mac(str(item["mac"])),
            ip=str(item["ip"]).strip(),
            local_ip=str(item["local_ip"]).strip(),
            local_port=int(item["local_port"]),
            linked_board_id=linked_board_id,
        )
        if not inverter.inverter_id or not inverter.name:
            raise ValueError("inverter id and name cannot be empty")
        if inverter.protocol not in inverter_protocols.READERS:
            raise ValueError(
                f"unsupported protocol for {inverter.inverter_id}: {inverter.protocol}"
            )
        if not MAC_PATTERN.fullmatch(inverter.mac):
            raise ValueError(
                f"invalid MAC address for {inverter.inverter_id}: {inverter.mac}"
            )
        for label, address in (("IP", inverter.ip), ("local IP", inverter.local_ip)):
            try:
                parsed_address = ipaddress.ip_address(address)
            except ValueError as exc:
                raise ValueError(
                    f"invalid {label} for {inverter.inverter_id}: {address}"
                ) from exc
            if parsed_address.version != 4:
                raise ValueError(
                    f"{label} for {inverter.inverter_id} must be IPv4: {address}"
                )
        if not 1 <= inverter.local_port <= 65535:
            raise ValueError(
                f"invalid local port for {inverter.inverter_id}: {inverter.local_port}"
            )
        if linked_board_id is not None and linked_board_id not in board_ids:
            raise ValueError(
                f"unknown linked_board_id for {inverter.inverter_id}: {linked_board_id}"
            )
        if inverter.inverter_id in seen_inverter_ids:
            raise ValueError(f"duplicate inverter id: {inverter.inverter_id}")
        if inverter.mac in seen_inverter_macs:
            raise ValueError(f"duplicate inverter MAC address: {inverter.mac}")
        seen_inverter_ids.add(inverter.inverter_id)
        seen_inverter_macs.add(inverter.mac)
        inverters.append(inverter)

    database_value = database_override or str(raw.get("database", "data/telemetry.sqlite3"))
    return CollectorConfig(
        interval_seconds=interval,
        request_timeout_seconds=timeout,
        lease_file=resolve_project_path(str(raw["lease_file"])),
        database=resolve_project_path(database_value),
        boards=tuple(boards),
        sources=tuple(sources),
        inverters=tuple(inverters),
    )


def parse_dnsmasq_leases(text: str, now: int | None = None) -> list[Lease]:
    current_time = int(time.time()) if now is None else now
    leases: list[Lease] = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) < 4:
            continue
        try:
            expires_at = int(parts[0])
            ipaddress.ip_address(parts[2])
        except (ValueError, TypeError):
            continue
        if expires_at != 0 and expires_at < current_time:
            continue
        leases.append(
            Lease(
                expires_at=expires_at,
                mac=normalize_mac(parts[1]),
                ip=parts[2],
                hostname=parts[3],
            )
        )
    return leases


def read_leases(path: Path) -> list[Lease]:
    return parse_dnsmasq_leases(path.read_text(encoding="utf-8"))


def open_database(path: Path) -> sqlite3.Connection:
    path.parent.mkdir(parents=True, exist_ok=True)
    connection = sqlite3.connect(path, timeout=10)
    connection.row_factory = sqlite3.Row
    connection.execute("PRAGMA journal_mode = WAL")
    connection.execute("PRAGMA synchronous = NORMAL")
    connection.execute("PRAGMA foreign_keys = ON")
    connection.execute("PRAGMA busy_timeout = 5000")
    have_samples = connection.execute(
        "SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = 'telemetry_samples'"
    ).fetchone()
    if have_samples is None:
        connection.executescript(SCHEMA_SQL)
    else:
        migrate_database_v5(connection)
    return connection


def migrate_database_v2(connection: sqlite3.Connection) -> None:
    """Add independent BMS sources while preserving every v1 sample."""
    if connection.execute("PRAGMA user_version").fetchone()[0] >= 2:
        return
    with connection:
        connection.executescript(SOURCES_SCHEMA_SQL)
        columns = {
            str(row[1]) for row in connection.execute("PRAGMA table_info(telemetry_samples)")
        }
        if "source_id" not in columns:
            connection.execute("ALTER TABLE telemetry_samples ADD COLUMN source_id TEXT")

        timestamp = utc_now()
        connection.execute(
            """
            INSERT OR IGNORE INTO bms_sources (
                source_id, board_id, name, endpoint, updated_at_utc
            )
            SELECT
                board_id || '-bms1',
                board_id,
                hostname || ' BMS 1',
                '/api/telemetry',
                ?
            FROM boards
            """,
            (timestamp,),
        )
        connection.execute(
            """
            UPDATE telemetry_samples
            SET source_id = board_id || '-bms1'
            WHERE source_id IS NULL OR source_id = ''
            """
        )
        connection.execute("DROP VIEW IF EXISTS latest_telemetry")
        connection.execute("DROP INDEX IF EXISTS idx_telemetry_board_time")
        connection.execute(
            """
            CREATE INDEX IF NOT EXISTS idx_telemetry_source_time
            ON telemetry_samples(source_id, sampled_at_unix_ms DESC)
            """
        )
        connection.execute(
            """
            CREATE INDEX IF NOT EXISTS idx_telemetry_time
            ON telemetry_samples(sampled_at_unix_ms DESC)
            """
        )
        connection.execute(
            """
            CREATE VIEW latest_telemetry AS
            SELECT samples.*
            FROM telemetry_samples AS samples
            JOIN (
                SELECT source_id, MAX(id) AS latest_id
                FROM telemetry_samples
                GROUP BY source_id
            ) AS latest
            ON samples.id = latest.latest_id
            """
        )
        connection.execute("PRAGMA user_version = 2")


def migrate_database_v3(connection: sqlite3.Connection) -> None:
    """Add inverter inventory/samples while preserving all BMS history."""
    version = connection.execute("PRAGMA user_version").fetchone()[0]
    if version >= 3:
        return
    if version < 2:
        migrate_database_v2(connection)
    with connection:
        connection.executescript(INVERTER_SCHEMA_SQL)
        connection.execute("PRAGMA user_version = 3")


def migrate_database_v4(connection: sqlite3.Connection) -> None:
    """Allow multiple dongles to share their required local TCP port."""
    version = connection.execute("PRAGMA user_version").fetchone()[0]
    if version >= 4:
        return
    if version < 3:
        migrate_database_v3(connection)

    table_sql_row = connection.execute(
        "SELECT sql FROM sqlite_master WHERE type='table' AND name='inverters'"
    ).fetchone()
    table_sql = str(table_sql_row[0]) if table_sql_row is not None else ""
    if "UNIQUE (local_ip, local_port)" in table_sql:
        connection.commit()
        connection.execute("PRAGMA foreign_keys = OFF")
        try:
            with connection:
                connection.execute("DROP TABLE IF EXISTS inverters_v4")
                connection.execute(
                    """
                    CREATE TABLE inverters_v4 (
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
                        FOREIGN KEY (linked_board_id) REFERENCES boards(board_id)
                    )
                    """
                )
                connection.execute(
                    """
                    INSERT INTO inverters_v4
                    SELECT
                        inverter_id, name, protocol, mac, linked_board_id,
                        configured_ip, local_ip, local_port, last_ip,
                        last_seen_utc, last_error, updated_at_utc
                    FROM inverters
                    """
                )
                connection.execute("DROP TABLE inverters")
                connection.execute("ALTER TABLE inverters_v4 RENAME TO inverters")
        finally:
            connection.execute("PRAGMA foreign_keys = ON")
    with connection:
        connection.execute("PRAGMA user_version = 4")
    foreign_key_errors = connection.execute("PRAGMA foreign_key_check").fetchall()
    if foreign_key_errors:
        raise sqlite3.IntegrityError(
            f"foreign key errors after v4 migration: {foreign_key_errors}"
        )


def migrate_database_v5(connection: sqlite3.Connection) -> None:
    """Store every live inverter field explicitly and remove settings from JSON."""
    version = connection.execute("PRAGMA user_version").fetchone()[0]
    if version >= 5:
        return
    if version < 4:
        migrate_database_v4(connection)

    columns = {
        str(row[1]) for row in connection.execute("PRAGMA table_info(inverter_samples)")
    }
    with connection:
        for field, sql_type in INVERTER_V5_COLUMN_TYPES.items():
            if field not in columns:
                connection.execute(
                    f"ALTER TABLE inverter_samples ADD COLUMN {field} {sql_type}"
                )

        if "payload_json" in columns:
            rows = connection.execute(
                "SELECT id, payload_json FROM inverter_samples"
            ).fetchall()
            assignments = ", ".join(
                f"{field} = ?" for field in INVERTER_V5_COLUMN_TYPES
            )
            for row in rows:
                try:
                    payload = json.loads(str(row["payload_json"]))
                except (TypeError, ValueError, json.JSONDecodeError):
                    continue
                if not isinstance(payload, dict):
                    continue
                live_payload = inverter_live_payload(payload)
                values = [
                    sqlite_value(field, live_payload.get(field))
                    for field in INVERTER_V5_COLUMN_TYPES
                ]
                values.extend(
                    (
                        json.dumps(
                            live_payload,
                            separators=(",", ":"),
                            ensure_ascii=False,
                            sort_keys=True,
                        ),
                        row["id"],
                    )
                )
                connection.execute(
                    f"UPDATE inverter_samples SET {assignments}, payload_json = ? WHERE id = ?",
                    values,
                )

        connection.execute("DROP VIEW IF EXISTS latest_inverter_telemetry")
        connection.execute(
            """
            CREATE VIEW latest_inverter_telemetry AS
            SELECT samples.*
            FROM inverter_samples AS samples
            JOIN (
                SELECT inverter_id, MAX(id) AS latest_id
                FROM inverter_samples
                GROUP BY inverter_id
            ) AS latest
            ON samples.id = latest.latest_id
            """
        )
        connection.execute("PRAGMA user_version = 5")

    foreign_key_errors = connection.execute("PRAGMA foreign_key_check").fetchall()
    if foreign_key_errors:
        raise sqlite3.IntegrityError(
            f"foreign key errors after v5 migration: {foreign_key_errors}"
        )


def register_boards(connection: sqlite3.Connection, boards: Iterable[BoardConfig]) -> None:
    timestamp = utc_now()
    for board in boards:
        connection.execute(
            """
            INSERT INTO boards (board_id, hostname, mac, updated_at_utc)
            VALUES (?, ?, ?, ?)
            ON CONFLICT(board_id) DO UPDATE SET
                hostname = excluded.hostname,
                mac = excluded.mac,
                updated_at_utc = excluded.updated_at_utc
            """,
            (board.board_id, board.hostname, board.mac, timestamp),
        )
    connection.commit()


def register_sources(
    connection: sqlite3.Connection, sources: Iterable[BmsSourceConfig]
) -> None:
    timestamp = utc_now()
    for source in sources:
        connection.execute(
            """
            INSERT INTO bms_sources (
                source_id, board_id, name, endpoint, updated_at_utc
            )
            VALUES (?, ?, ?, ?, ?)
            ON CONFLICT(source_id) DO UPDATE SET
                board_id = excluded.board_id,
                name = excluded.name,
                endpoint = excluded.endpoint,
                updated_at_utc = excluded.updated_at_utc
            """,
            (
                source.source_id,
                source.board_id,
                source.name,
                source.endpoint,
                timestamp,
            ),
        )
    connection.commit()


def register_inverters(
    connection: sqlite3.Connection, inverters: Iterable[InverterConfig]
) -> None:
    timestamp = utc_now()
    for inverter in inverters:
        connection.execute(
            """
            INSERT INTO inverters (
                inverter_id, name, protocol, mac, linked_board_id,
                configured_ip, local_ip, local_port, updated_at_utc
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT(inverter_id) DO UPDATE SET
                name = excluded.name,
                protocol = excluded.protocol,
                mac = excluded.mac,
                linked_board_id = excluded.linked_board_id,
                configured_ip = excluded.configured_ip,
                local_ip = excluded.local_ip,
                local_port = excluded.local_port,
                updated_at_utc = excluded.updated_at_utc
            """,
            (
                inverter.inverter_id,
                inverter.name,
                inverter.protocol,
                inverter.mac,
                inverter.linked_board_id,
                inverter.ip,
                inverter.local_ip,
                inverter.local_port,
                timestamp,
            ),
        )
    connection.commit()


def load_cached_ips(connection: sqlite3.Connection) -> dict[str, str]:
    rows = connection.execute(
        "SELECT board_id, last_ip FROM boards WHERE last_ip IS NOT NULL"
    ).fetchall()
    return {str(row["board_id"]): str(row["last_ip"]) for row in rows}


def resolve_board_ip(
    board: BoardConfig, leases: Iterable[Lease], cached_ips: dict[str, str]
) -> str | None:
    if board.static_ip:
        return board.static_ip

    matching_mac = [lease for lease in leases if lease.mac == board.mac]
    if matching_mac:
        return max(matching_mac, key=lambda lease: lease.expires_at).ip

    matching_hostname = [
        lease for lease in leases if lease.hostname.lower() == board.hostname.lower()
    ]
    if matching_hostname:
        return max(matching_hostname, key=lambda lease: lease.expires_at).ip

    cached_ip = cached_ips.get(board.board_id)
    if cached_ip:
        try:
            ipaddress.ip_address(cached_ip)
            return cached_ip
        except ValueError:
            return None
    return None


def fetch_telemetry(
    board: BoardConfig,
    source: BmsSourceConfig,
    ip: str | None,
    timeout_seconds: float,
) -> CollectionResult:
    if ip is None:
        return CollectionResult(
            board=board,
            source=source,
            ip=None,
            payload=None,
            sampled_at_utc=None,
            sampled_at_unix_ms=None,
            error="no current DHCP lease or cached IP",
        )

    request = urllib.request.Request(
        f"http://{ip}{source.endpoint}",
        headers={
            "Accept": "application/json",
            "User-Agent": "inverter-telemetry-collector/1",
        },
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout_seconds) as response:
            body = response.read(MAX_RESPONSE_BYTES + 1)
            if len(body) > MAX_RESPONSE_BYTES:
                raise ValueError("telemetry response exceeds 1 MiB")
            if response.status != 200:
                raise ValueError(f"HTTP {response.status}")
        payload = json.loads(body.decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("telemetry response is not a JSON object")
        if payload.get("valid") is not True:
            raise ValueError("telemetry payload is not valid")
        if payload.get("stale") is True:
            raise ValueError("telemetry payload is stale")
    except (OSError, ValueError, UnicodeError, json.JSONDecodeError) as exc:
        return CollectionResult(
            board=board,
            source=source,
            ip=ip,
            payload=None,
            sampled_at_utc=None,
            sampled_at_unix_ms=None,
            error=f"{type(exc).__name__}: {exc}",
        )

    return CollectionResult(
        board=board,
        source=source,
        ip=ip,
        payload=payload,
        sampled_at_utc=utc_now(),
        sampled_at_unix_ms=time.time_ns() // 1_000_000,
        error=None,
    )


def fetch_inverter(
    inverter: InverterConfig, timeout_seconds: float
) -> InverterCollectionResult:
    try:
        payload = inverter_protocols.read_inverter(
            inverter.protocol,
            inverter.ip,
            inverter.local_ip,
            inverter.local_port,
            timeout_seconds,
        )
        if not isinstance(payload, dict) or not payload.get("protocol"):
            raise ValueError("inverter reader returned an invalid payload")
    except (OSError, TimeoutError, ValueError, inverter_protocols.InverterProtocolError) as exc:
        return InverterCollectionResult(
            inverter=inverter,
            payload=None,
            sampled_at_utc=None,
            sampled_at_unix_ms=None,
            error=f"{type(exc).__name__}: {exc}",
        )

    return InverterCollectionResult(
        inverter=inverter,
        payload=payload,
        sampled_at_utc=utc_now(),
        sampled_at_unix_ms=time.time_ns() // 1_000_000,
        error=None,
    )


def sqlite_value(field: str, value: Any) -> Any:
    if value is None:
        return None
    if field in BOOLEAN_FIELDS:
        return int(bool(value))
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, (str, int, float)):
        return value
    return json.dumps(value, separators=(",", ":"), ensure_ascii=False)


def inverter_live_payload(payload: dict[str, Any]) -> dict[str, Any]:
    """Return only operational inverter telemetry and its live raw responses."""
    live = {
        field: payload[field]
        for field in INVERTER_SAMPLE_FIELDS
        if field in payload
    }
    raw_value = payload.get("raw")
    if not isinstance(raw_value, dict):
        return live

    raw = dict(raw_value)
    if payload.get("protocol") == "EASUN_VOLTRONIC_QPIGS":
        # QPIRI is a ratings/configuration response.  Retain only the live
        # QPIGS/QMOD/QPIWS/QPIGS2 response material in telemetry history.
        for bucket_name in ("responses", "frames_hex", "optional_errors"):
            bucket = raw.get(bucket_name)
            if isinstance(bucket, dict):
                raw[bucket_name] = {
                    key: value for key, value in bucket.items() if key != "QPIRI"
                }
    live["raw"] = raw
    return live


def store_results(
    connection: sqlite3.Connection, results: Iterable[CollectionResult]
) -> tuple[int, int]:
    result_list = list(results)
    successes = 0
    failures = 0
    placeholders = ", ".join("?" for _ in SAMPLE_COLUMNS)
    insert_sql = (
        f"INSERT INTO telemetry_samples ({', '.join(SAMPLE_COLUMNS)}) "
        f"VALUES ({placeholders})"
    )

    with connection:
        for result in result_list:
            updated_at = utc_now()
            if not result.succeeded:
                failures += 1
                connection.execute(
                    """
                    UPDATE bms_sources
                    SET last_error = ?,
                        updated_at_utc = ?
                    WHERE source_id = ?
                    """,
                    (
                        (result.error or "unknown error")[:500],
                        updated_at,
                        result.source.source_id,
                    ),
                )
                continue

            assert result.payload is not None
            assert result.ip is not None
            assert result.sampled_at_utc is not None
            assert result.sampled_at_unix_ms is not None
            successes += 1

            values: list[Any] = [
                result.board.board_id,
                result.source.source_id,
                result.sampled_at_utc,
                result.sampled_at_unix_ms,
                result.ip,
            ]
            values.extend(
                sqlite_value(field, result.payload.get(field))
                for field in TELEMETRY_FIELDS
            )
            values.extend(
                (
                    json.dumps(
                        result.payload.get("cells_v", []),
                        separators=(",", ":"),
                        ensure_ascii=False,
                    ),
                    json.dumps(
                        result.payload,
                        separators=(",", ":"),
                        ensure_ascii=False,
                        sort_keys=True,
                    ),
                )
            )
            connection.execute(insert_sql, values)
            connection.execute(
                """
                UPDATE bms_sources
                SET last_seen_utc = ?,
                    last_error = NULL,
                    updated_at_utc = ?
                WHERE source_id = ?
                """,
                (
                    result.sampled_at_utc,
                    updated_at,
                    result.source.source_id,
                ),
            )

        for board_id in {result.board.board_id for result in result_list}:
            board_results = [
                result for result in result_list if result.board.board_id == board_id
            ]
            successful = [result for result in board_results if result.succeeded]
            board_ip = next((result.ip for result in board_results if result.ip), None)
            if successful:
                latest_seen = max(
                    result.sampled_at_utc or "" for result in successful
                )
                board_error = None
            else:
                latest_seen = None
                board_error = "; ".join(
                    f"{result.source.source_id}: {result.error or 'unknown error'}"
                    for result in board_results
                )[:500]
            connection.execute(
                """
                UPDATE boards
                SET last_ip = COALESCE(?, last_ip),
                    last_seen_utc = COALESCE(?, last_seen_utc),
                    last_error = ?,
                    updated_at_utc = ?
                WHERE board_id = ?
                """,
                (board_ip, latest_seen, board_error, utc_now(), board_id),
            )
    return successes, failures


def store_inverter_results(
    connection: sqlite3.Connection,
    results: Iterable[InverterCollectionResult],
) -> tuple[int, int]:
    result_list = list(results)
    successes = 0
    failures = 0
    placeholders = ", ".join("?" for _ in INVERTER_SAMPLE_COLUMNS)
    insert_sql = (
        f"INSERT INTO inverter_samples ({', '.join(INVERTER_SAMPLE_COLUMNS)}) "
        f"VALUES ({placeholders})"
    )

    with connection:
        for result in result_list:
            updated_at = utc_now()
            if not result.succeeded:
                failures += 1
                connection.execute(
                    """
                    UPDATE inverters
                    SET last_ip = ?,
                        last_error = ?,
                        updated_at_utc = ?
                    WHERE inverter_id = ?
                    """,
                    (
                        result.inverter.ip,
                        (result.error or "unknown error")[:500],
                        updated_at,
                        result.inverter.inverter_id,
                    ),
                )
                continue

            assert result.payload is not None
            assert result.sampled_at_utc is not None
            assert result.sampled_at_unix_ms is not None
            successes += 1
            live_payload = inverter_live_payload(result.payload)
            values: list[Any] = [
                result.inverter.inverter_id,
                result.sampled_at_utc,
                result.sampled_at_unix_ms,
                result.inverter.ip,
            ]
            values.extend(
                sqlite_value(field, live_payload.get(field))
                for field in INVERTER_SAMPLE_FIELDS
            )
            values.append(
                json.dumps(
                    live_payload,
                    separators=(",", ":"),
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
            connection.execute(insert_sql, values)
            connection.execute(
                """
                UPDATE inverters
                SET last_ip = ?,
                    last_seen_utc = ?,
                    last_error = NULL,
                    updated_at_utc = ?
                WHERE inverter_id = ?
                """,
                (
                    result.inverter.ip,
                    result.sampled_at_utc,
                    updated_at,
                    result.inverter.inverter_id,
                ),
            )
    return successes, failures


def collect_once(
    config: CollectorConfig, connection: sqlite3.Connection
) -> list[CollectionResult]:
    try:
        leases = read_leases(config.lease_file)
        lease_error = None
    except OSError as exc:
        leases = []
        lease_error = f"cannot read {config.lease_file}: {exc}"

    cached_ips = load_cached_ips(connection)
    board_ips = {
        board: resolve_board_ip(board, leases, cached_ips) for board in config.boards
    }
    boards_by_id = {board.board_id: board for board in config.boards}
    results: list[CollectionResult] = []
    with concurrent.futures.ThreadPoolExecutor(
        max_workers=min(len(config.sources), 8)
    ) as executor:
        futures = {
            executor.submit(
                fetch_telemetry,
                boards_by_id[source.board_id],
                source,
                board_ips[boards_by_id[source.board_id]],
                config.request_timeout_seconds,
            ): source
            for source in config.sources
        }
        for future in concurrent.futures.as_completed(futures):
            results.append(future.result())

    results.sort(key=lambda result: result.source.source_id)
    successes, failures = store_results(connection, results)
    timestamp = utc_now()
    details = ", ".join(
        (
            f"{result.source.source_id}@{result.ip}{result.source.endpoint}=stored"
            if result.succeeded
            else f"{result.source.source_id}@{result.ip or '-'}{result.source.endpoint}=skipped"
        )
        for result in results
    )
    print(
        f"{timestamp} telemetry cycle: {successes} stored, "
        f"{failures} skipped ({details})",
        flush=True,
    )
    if lease_error:
        print(f"{timestamp} warning: {lease_error}", flush=True)
    for result in results:
        if result.error:
            print(
                f"{timestamp} warning: {result.source.source_id}: {result.error}",
                flush=True,
            )
    return results


def collect_inverters_once(
    config: CollectorConfig, connection: sqlite3.Connection
) -> list[InverterCollectionResult]:
    if not config.inverters:
        return []

    # Both installed Eybond dongles only dial back to their standard TCP port
    # 8899. Poll sequentially so they can safely share the same listener.
    results = [
        fetch_inverter(inverter, config.request_timeout_seconds)
        for inverter in config.inverters
    ]

    results.sort(key=lambda result: result.inverter.inverter_id)
    successes, failures = store_inverter_results(connection, results)
    timestamp = utc_now()
    details = ", ".join(
        (
            f"{result.inverter.inverter_id}@{result.inverter.ip}=stored"
            if result.succeeded
            else f"{result.inverter.inverter_id}@{result.inverter.ip}=skipped"
        )
        for result in results
    )
    print(
        f"{timestamp} inverter cycle: {successes} stored, "
        f"{failures} skipped ({details})",
        flush=True,
    )
    for result in results:
        if result.error:
            print(
                f"{timestamp} warning: {result.inverter.inverter_id}: {result.error}",
                flush=True,
            )
    return results


def print_status(connection: sqlite3.Connection, database_path: Path) -> None:
    print(f"Database: {database_path}")
    rows = connection.execute(
        """
        SELECT
            bms_sources.source_id,
            bms_sources.name,
            bms_sources.endpoint,
            bms_sources.last_seen_utc,
            bms_sources.last_error,
            boards.board_id,
            boards.hostname,
            boards.last_ip,
            COALESCE(sample_counts.sample_count, 0) AS sample_count,
            sample_counts.latest_sample,
            latest_telemetry.protocol AS latest_protocol,
            latest_telemetry.cell_count AS latest_cell_count
        FROM bms_sources
        JOIN boards ON boards.board_id = bms_sources.board_id
        LEFT JOIN (
            SELECT
                source_id,
                COUNT(*) AS sample_count,
                MAX(sampled_at_utc) AS latest_sample
            FROM telemetry_samples
            GROUP BY source_id
        ) AS sample_counts ON sample_counts.source_id = bms_sources.source_id
        LEFT JOIN latest_telemetry
            ON latest_telemetry.source_id = bms_sources.source_id
        ORDER BY bms_sources.source_id
        """
    ).fetchall()
    for row in rows:
        print(
            f"{row['source_id']} ({row['name']}): board={row['board_id']} "
            f"endpoint={row['endpoint']} samples={row['sample_count']} "
            f"latest={row['latest_sample'] or '-'} ip={row['last_ip'] or '-'} "
            f"protocol={row['latest_protocol'] or '-'} "
            f"cells={row['latest_cell_count'] if row['latest_cell_count'] is not None else '-'} "
            f"error={row['last_error'] or '-'}"
        )

    inverter_rows = connection.execute(
        """
        SELECT
            inverters.inverter_id,
            inverters.name,
            inverters.protocol AS configured_protocol,
            inverters.linked_board_id,
            inverters.last_ip,
            inverters.last_seen_utc,
            inverters.last_error,
            COALESCE(sample_counts.sample_count, 0) AS sample_count,
            sample_counts.latest_sample,
            latest_inverter_telemetry.protocol AS latest_protocol,
            latest_inverter_telemetry.working_mode,
            latest_inverter_telemetry.output_power_w,
            latest_inverter_telemetry.battery_voltage_v,
            latest_inverter_telemetry.pv_power_w
        FROM inverters
        LEFT JOIN (
            SELECT
                inverter_id,
                COUNT(*) AS sample_count,
                MAX(sampled_at_utc) AS latest_sample
            FROM inverter_samples
            GROUP BY inverter_id
        ) AS sample_counts ON sample_counts.inverter_id = inverters.inverter_id
        LEFT JOIN latest_inverter_telemetry
            ON latest_inverter_telemetry.inverter_id = inverters.inverter_id
        ORDER BY inverters.inverter_id
        """
    ).fetchall()
    for row in inverter_rows:
        print(
            f"{row['inverter_id']} ({row['name']}): "
            f"linked_board={row['linked_board_id'] or '-'} "
            f"samples={row['sample_count']} latest={row['latest_sample'] or '-'} "
            f"ip={row['last_ip'] or '-'} "
            f"protocol={row['latest_protocol'] or row['configured_protocol']} "
            f"mode={row['working_mode'] or '-'} "
            f"output={row['output_power_w'] if row['output_power_w'] is not None else '-'}W "
            f"battery={row['battery_voltage_v'] if row['battery_voltage_v'] is not None else '-'}V "
            f"pv={row['pv_power_w'] if row['pv_power_w'] is not None else '-'}W "
            f"error={row['last_error'] or '-'}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"collector configuration (default: {DEFAULT_CONFIG})",
    )
    parser.add_argument(
        "--database",
        help="override the SQLite database path from the configuration",
    )
    parser.add_argument(
        "--interval",
        type=float,
        help="override the polling interval in seconds",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--once", action="store_true", help="collect one cycle and exit")
    mode.add_argument(
        "--init-only",
        action="store_true",
        help="initialize the database and exit",
    )
    mode.add_argument(
        "--status",
        action="store_true",
        help="show sample counts and last collector status",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        config = load_config(args.config.resolve(), args.database)
        if args.interval is not None:
            if args.interval <= 0:
                raise ValueError("--interval must be greater than zero")
            config = dataclasses.replace(config, interval_seconds=args.interval)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"Configuration error: {exc}", flush=True)
        return 2

    try:
        connection = open_database(config.database)
        register_boards(connection, config.boards)
        register_sources(connection, config.sources)
        register_inverters(connection, config.inverters)
    except (OSError, sqlite3.Error) as exc:
        print(f"Database error: {exc}", flush=True)
        return 3

    if args.init_only:
        print(f"Initialized {config.database}")
        connection.close()
        return 0
    if args.status:
        print_status(connection, config.database)
        connection.close()
        return 0
    if args.once:
        collect_once(config, connection)
        collect_inverters_once(config, connection)
        connection.close()
        return 0

    stop_event = threading.Event()

    def request_stop(_signum: int, _frame: Any) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    print(
        f"Starting telemetry collector: boards={len(config.boards)} "
        f"sources={len(config.sources)} "
        f"inverters={len(config.inverters)} "
        f"interval={config.interval_seconds:g}s database={config.database}",
        flush=True,
    )
    try:
        while not stop_event.is_set():
            cycle_started = time.monotonic()
            collect_once(config, connection)
            collect_inverters_once(config, connection)
            remaining = max(
                0.0, config.interval_seconds - (time.monotonic() - cycle_started)
            )
            stop_event.wait(remaining)
    finally:
        connection.close()
    print("Telemetry collector stopped", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
