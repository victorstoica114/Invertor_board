#!/usr/bin/env python3
"""LAN dashboard for the local Daly, Seplos and JK Bluetooth BMS devices."""

from __future__ import annotations

import argparse
import asyncio
from collections import deque
import copy
import datetime as dt
from dataclasses import dataclass
import hmac
import logging
import os
from pathlib import Path
import time
from typing import Any, Awaitable, Callable, Final, Mapping

from aiohttp import web

try:
    from tools import bms_ble
except ModuleNotFoundError:  # Direct execution: python tools/bms_dashboard.py
    import bms_ble  # type: ignore[no-redef]


LOGGER = logging.getLogger("bms_dashboard")
PROJECT_ROOT: Final[Path] = Path(__file__).resolve().parents[1]
STATIC_ROOT: Final[Path] = PROJECT_ROOT / "tools" / "bms_dashboard_static"
HISTORY_LIMIT: Final[int] = 180
MIN_REFRESH_SECONDS: Final[float] = 3.0

TelemetryReader = Callable[[str], Awaitable[dict[str, Any]]]
ProtocolReader = Callable[[], Awaitable[dict[str, Any]]]
JkProtocolWriter = Callable[[str, str, str], Awaitable[dict[str, Any]]]
SeplosProtocolWriter = Callable[[str, str], Awaitable[dict[str, Any]]]


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


def control_authorized(headers: Mapping[str, str], configured_token: str) -> bool:
    supplied = headers.get("X-Control-Token", "")
    return bool(configured_token) and hmac.compare_digest(supplied, configured_token)


@dataclass(frozen=True)
class DashboardConfig:
    host: str = "0.0.0.0"
    port: int = 8765
    poll_interval_seconds: float = 20.0
    protocol_interval_seconds: float = 300.0
    control_token: str = ""

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
        token = os.environ.get("BMS_DASHBOARD_CONTROL_TOKEN", "")
        if not 1 <= port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if poll_interval < 5:
            raise ValueError("poll interval must be at least 5 seconds")
        if protocol_interval < 30:
            raise ValueError("protocol interval must be at least 30 seconds")
        return cls(host, port, poll_interval, protocol_interval, token)


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
    ) -> None:
        self.config = config
        self.telemetry_reader = telemetry_reader
        self.jk_protocol_reader = jk_protocol_reader
        self.seplos_protocol_reader = seplos_protocol_reader
        self.jk_protocol_writer = jk_protocol_writer
        self.seplos_protocol_writer = seplos_protocol_writer
        self.operation_lock = asyncio.Lock()
        self.refresh_event = asyncio.Event()
        self.poll_task: asyncio.Task[None] | None = None
        self.polling = False
        self.generation = 0
        self.last_poll_started: str | None = None
        self.last_poll_finished: str | None = None
        self.last_protocol_monotonic = 0.0
        self.last_refresh_request_monotonic = 0.0
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

    def public_snapshot(self) -> dict[str, Any]:
        return {
            "server_time": utc_now(),
            "generation": self.generation,
            "polling": self.polling,
            "poll_interval_seconds": self.config.poll_interval_seconds,
            "last_poll_started": self.last_poll_started,
            "last_poll_finished": self.last_poll_finished,
            "control_enabled": bool(self.config.control_token),
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
        "default-src 'self'; style-src 'self'; script-src 'self'; img-src 'self' data:; connect-src 'self'"
    )
    return response


def dashboard_state(request: web.Request) -> DashboardState:
    return request.app[STATE_KEY]


async def index_handler(_request: web.Request) -> web.FileResponse:
    return web.FileResponse(STATIC_ROOT / "index.html")


async def status_handler(request: web.Request) -> web.Response:
    return web.json_response(dashboard_state(request).public_snapshot())


async def health_handler(request: web.Request) -> web.Response:
    state = dashboard_state(request)
    return web.json_response(
        {
            "ok": state.poll_task is not None and not state.poll_task.done(),
            "polling": state.polling,
            "generation": state.generation,
            "server_time": utc_now(),
        }
    )


async def refresh_handler(request: web.Request) -> web.Response:
    accepted = dashboard_state(request).request_refresh()
    return web.json_response({"accepted": accepted, "reason": None if accepted else "refresh rate limited"})


def require_control(request: web.Request) -> DashboardState:
    state = dashboard_state(request)
    if not state.config.control_token:
        raise web.HTTPServiceUnavailable(text="BMS control is disabled on this server")
    if not control_authorized(request.headers, state.config.control_token):
        raise web.HTTPUnauthorized(text="invalid control token")
    return state


async def protocol_refresh_handler(request: web.Request) -> web.Response:
    state = require_control(request)
    if not await state.refresh_protocols():
        raise web.HTTPConflict(text="another Bluetooth operation is in progress")
    return web.json_response({"ok": True, "protocols": state.public_snapshot()["protocols"]})


async def jk_protocol_handler(request: web.Request) -> web.Response:
    state = require_control(request)
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
    state = require_control(request)
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


async def stop_background(app: web.Application) -> None:
    task = app[STATE_KEY].poll_task
    if task is None:
        return
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass


def create_app(config: DashboardConfig, state: DashboardState | None = None) -> web.Application:
    if not (STATIC_ROOT / "index.html").is_file():
        raise RuntimeError(f"dashboard static files are missing from {STATIC_ROOT}")
    app = web.Application(middlewares=[response_headers], client_max_size=16 * 1024)
    app[STATE_KEY] = state or DashboardState(config)
    app.router.add_get("/", index_handler)
    app.router.add_get("/api/status", status_handler)
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
        "starting on http://%s:%d (poll %.1fs, controls %s)",
        config.host,
        config.port,
        config.poll_interval_seconds,
        "enabled" if config.control_token else "disabled",
    )
    web.run_app(create_app(config), host=config.host, port=config.port, print=None)


if __name__ == "__main__":
    main()
