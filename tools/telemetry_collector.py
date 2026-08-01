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
    "sampled_at_utc",
    "sampled_at_unix_ms",
    "source_ip",
    *TELEMETRY_FIELDS,
    "cells_v_json",
    "payload_json",
)

SCHEMA_SQL = """
PRAGMA user_version = 1;

CREATE TABLE IF NOT EXISTS boards (
    board_id TEXT PRIMARY KEY,
    hostname TEXT NOT NULL UNIQUE,
    mac TEXT NOT NULL UNIQUE COLLATE NOCASE,
    last_ip TEXT,
    last_seen_utc TEXT,
    last_error TEXT,
    updated_at_utc TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS telemetry_samples (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    board_id TEXT NOT NULL,
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
    FOREIGN KEY (board_id) REFERENCES boards(board_id)
);

CREATE INDEX IF NOT EXISTS idx_telemetry_board_time
    ON telemetry_samples(board_id, sampled_at_unix_ms DESC);

CREATE INDEX IF NOT EXISTS idx_telemetry_time
    ON telemetry_samples(sampled_at_unix_ms DESC);

CREATE VIEW IF NOT EXISTS latest_telemetry AS
SELECT samples.*
FROM telemetry_samples AS samples
JOIN (
    SELECT board_id, MAX(id) AS latest_id
    FROM telemetry_samples
    GROUP BY board_id
) AS latest
ON samples.id = latest.latest_id;
"""


@dataclasses.dataclass(frozen=True)
class BoardConfig:
    board_id: str
    hostname: str
    mac: str


@dataclasses.dataclass(frozen=True)
class CollectorConfig:
    interval_seconds: float
    request_timeout_seconds: float
    lease_file: Path
    database: Path
    boards: tuple[BoardConfig, ...]


@dataclasses.dataclass(frozen=True)
class Lease:
    expires_at: int
    mac: str
    ip: str
    hostname: str


@dataclasses.dataclass(frozen=True)
class CollectionResult:
    board: BoardConfig
    ip: str | None
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
        board = BoardConfig(
            board_id=str(item["id"]).strip(),
            hostname=str(item["hostname"]).strip(),
            mac=normalize_mac(str(item["mac"])),
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

    database_value = database_override or str(raw.get("database", "data/telemetry.sqlite3"))
    return CollectorConfig(
        interval_seconds=interval,
        request_timeout_seconds=timeout,
        lease_file=resolve_project_path(str(raw["lease_file"])),
        database=resolve_project_path(database_value),
        boards=tuple(boards),
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
    connection.executescript(SCHEMA_SQL)
    return connection


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


def load_cached_ips(connection: sqlite3.Connection) -> dict[str, str]:
    rows = connection.execute(
        "SELECT board_id, last_ip FROM boards WHERE last_ip IS NOT NULL"
    ).fetchall()
    return {str(row["board_id"]): str(row["last_ip"]) for row in rows}


def resolve_board_ip(
    board: BoardConfig, leases: Iterable[Lease], cached_ips: dict[str, str]
) -> str | None:
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
    board: BoardConfig, ip: str | None, timeout_seconds: float
) -> CollectionResult:
    if ip is None:
        return CollectionResult(
            board=board,
            ip=None,
            payload=None,
            sampled_at_utc=None,
            sampled_at_unix_ms=None,
            error="no current DHCP lease or cached IP",
        )

    request = urllib.request.Request(
        f"http://{ip}/api/telemetry",
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
    except (OSError, ValueError, UnicodeError, json.JSONDecodeError) as exc:
        return CollectionResult(
            board=board,
            ip=ip,
            payload=None,
            sampled_at_utc=None,
            sampled_at_unix_ms=None,
            error=f"{type(exc).__name__}: {exc}",
        )

    return CollectionResult(
        board=board,
        ip=ip,
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


def store_results(
    connection: sqlite3.Connection, results: Iterable[CollectionResult]
) -> tuple[int, int]:
    successes = 0
    failures = 0
    placeholders = ", ".join("?" for _ in SAMPLE_COLUMNS)
    insert_sql = (
        f"INSERT INTO telemetry_samples ({', '.join(SAMPLE_COLUMNS)}) "
        f"VALUES ({placeholders})"
    )

    with connection:
        for result in results:
            updated_at = utc_now()
            if not result.succeeded:
                failures += 1
                connection.execute(
                    """
                    UPDATE boards
                    SET last_ip = COALESCE(?, last_ip),
                        last_error = ?,
                        updated_at_utc = ?
                    WHERE board_id = ?
                    """,
                    (
                        result.ip,
                        (result.error or "unknown error")[:500],
                        updated_at,
                        result.board.board_id,
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
                UPDATE boards
                SET last_ip = ?,
                    last_seen_utc = ?,
                    last_error = NULL,
                    updated_at_utc = ?
                WHERE board_id = ?
                """,
                (
                    result.ip,
                    result.sampled_at_utc,
                    updated_at,
                    result.board.board_id,
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
    results: list[CollectionResult] = []
    with concurrent.futures.ThreadPoolExecutor(
        max_workers=min(len(config.boards), 8)
    ) as executor:
        futures = {
            executor.submit(
                fetch_telemetry,
                board,
                board_ips[board],
                config.request_timeout_seconds,
            ): board
            for board in config.boards
        }
        for future in concurrent.futures.as_completed(futures):
            results.append(future.result())

    results.sort(key=lambda result: result.board.board_id)
    successes, failures = store_results(connection, results)
    timestamp = utc_now()
    details = ", ".join(
        (
            f"{result.board.board_id}@{result.ip}=stored"
            if result.succeeded
            else f"{result.board.board_id}@{result.ip or '-'}=skipped"
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
                f"{timestamp} warning: {result.board.board_id}: {result.error}",
                flush=True,
            )
    return results


def print_status(connection: sqlite3.Connection, database_path: Path) -> None:
    print(f"Database: {database_path}")
    rows = connection.execute(
        """
        SELECT
            boards.board_id,
            boards.hostname,
            boards.mac,
            boards.last_ip,
            boards.last_seen_utc,
            boards.last_error,
            COUNT(telemetry_samples.id) AS sample_count,
            MAX(telemetry_samples.sampled_at_utc) AS latest_sample
        FROM boards
        LEFT JOIN telemetry_samples
            ON telemetry_samples.board_id = boards.board_id
        GROUP BY boards.board_id
        ORDER BY boards.board_id
        """
    ).fetchall()
    for row in rows:
        print(
            f"{row['board_id']}: samples={row['sample_count']} "
            f"latest={row['latest_sample'] or '-'} ip={row['last_ip'] or '-'} "
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
        connection.close()
        return 0

    stop_event = threading.Event()

    def request_stop(_signum: int, _frame: Any) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    print(
        f"Starting telemetry collector: boards={len(config.boards)} "
        f"interval={config.interval_seconds:g}s database={config.database}",
        flush=True,
    )
    try:
        while not stop_event.is_set():
            cycle_started = time.monotonic()
            collect_once(config, connection)
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
