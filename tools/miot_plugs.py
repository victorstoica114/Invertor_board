"""Local Xiaomi MIoT smart-plug telemetry and guarded power control."""

from __future__ import annotations

from dataclasses import dataclass
import ipaddress
import json
from pathlib import Path
import re
import time
from typing import Any, Final

from miio import Device


SUPPORTED_MODEL: Final[str] = "cuco.plug.v2eur"
DEVICE_ID_PATTERN: Final[re.Pattern[str]] = re.compile(r"^[a-z0-9][a-z0-9_-]{0,31}$")
TOKEN_PATTERN: Final[re.Pattern[str]] = re.compile(r"^[0-9a-fA-F]{32}$")
FAULT_LABELS: Final[dict[int, str]] = {0: "No faults", 1: "Over temperature", 2: "Overload"}
PROPERTY_SPECS: Final[tuple[tuple[str, int, int], ...]] = (
    ("on", 2, 1),
    ("default_power_on_state", 2, 2),
    ("fault", 2, 3),
    ("physical_controls_locked", 7, 1),
    ("energy_counter", 11, 1),
    ("electric_power_w", 11, 2),
    ("indicator_light", 13, 1),
    ("delay", 14, 1),
    ("delay_time_seconds", 14, 2),
    ("delay_remaining_seconds", 14, 3),
)


class MiotPlugError(RuntimeError):
    """Raised when a local MIoT operation does not complete or verify."""


@dataclass(frozen=True)
class PlugDefinition:
    id: str
    name: str
    ip: str
    token: str
    model: str = SUPPORTED_MODEL
    reference_voltage_v: float = 230.0

    def public_identity(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "ip": self.ip,
            "model": self.model,
            "reference_voltage_v": self.reference_voltage_v,
        }


def load_inventory(path: Path) -> dict[str, PlugDefinition]:
    """Load a private plug inventory without ever returning tokens to the web layer."""
    resolved = path.expanduser().resolve()
    try:
        mode = resolved.stat().st_mode & 0o777
        payload = json.loads(resolved.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
        raise MiotPlugError(f"cannot load plug inventory: {exc}") from exc
    if mode & 0o077:
        raise MiotPlugError(f"plug inventory must use mode 0600, found {mode:04o}")
    records = payload.get("devices") if isinstance(payload, dict) else None
    if not isinstance(records, list) or not records:
        raise MiotPlugError("plug inventory must contain a non-empty devices list")

    inventory: dict[str, PlugDefinition] = {}
    seen_ips: set[str] = set()
    for index, record in enumerate(records):
        if not isinstance(record, dict):
            raise MiotPlugError(f"device {index} must be an object")
        device_id = str(record.get("id", "")).strip()
        name = str(record.get("name", "")).strip()
        ip = str(record.get("ip", "")).strip()
        token = str(record.get("token", "")).strip()
        model = str(record.get("model", SUPPORTED_MODEL)).strip()
        try:
            reference_voltage_v = float(record.get("reference_voltage_v", 230))
        except (TypeError, ValueError) as exc:
            raise MiotPlugError(
                f"device {device_id or index} has an invalid reference voltage"
            ) from exc
        if not DEVICE_ID_PATTERN.fullmatch(device_id):
            raise MiotPlugError(f"device {index} has an invalid id")
        if not name:
            raise MiotPlugError(f"device {device_id} has no name")
        try:
            parsed_ip = ipaddress.ip_address(ip)
        except ValueError as exc:
            raise MiotPlugError(f"device {device_id} has an invalid IP address") from exc
        if parsed_ip.version != 4 or not parsed_ip.is_private:
            raise MiotPlugError(f"device {device_id} must use a private IPv4 address")
        if not TOKEN_PATTERN.fullmatch(token):
            raise MiotPlugError(f"device {device_id} has an invalid token")
        if model != SUPPORTED_MODEL:
            raise MiotPlugError(f"device {device_id} uses unsupported model {model!r}")
        if not 100 <= reference_voltage_v <= 260:
            raise MiotPlugError(
                f"device {device_id} reference voltage must be between 100 and 260 V"
            )
        if device_id in inventory or ip in seen_ips:
            raise MiotPlugError(f"duplicate plug id or IP for {device_id}")
        inventory[device_id] = PlugDefinition(
            device_id,
            name,
            ip,
            token.lower(),
            model,
            reference_voltage_v,
        )
        seen_ips.add(ip)
    return inventory


def _device(definition: PlugDefinition) -> Device:
    return Device(
        ip=definition.ip,
        token=definition.token,
        lazy_discover=True,
        timeout=4,
        model=definition.model,
    )


def _property_batches(size: int = 4):
    for index in range(0, len(PROPERTY_SPECS), size):
        yield PROPERTY_SPECS[index : index + size]


def apply_voltage_reference(
    telemetry: dict[str, Any],
    voltage_v: float,
    source: str,
    source_name: str | None = None,
    source_sample: str | None = None,
) -> None:
    """Attach voltage provenance and calculate current from measured power."""
    telemetry["voltage_v"] = round(float(voltage_v), 2)
    telemetry["voltage_source"] = source
    telemetry["voltage_source_name"] = source_name
    telemetry["voltage_source_sample"] = source_sample
    power = telemetry.get("electric_power_w")
    telemetry["estimated_current_a"] = (
        round(float(power) / float(voltage_v), 4)
        if isinstance(power, (int, float)) and not isinstance(power, bool)
        else None
    )
    telemetry["measurement_sources"] = {
        "electric_power_w": "device",
        "energy_counter": "device",
        "estimated_current_a": "calculated_from_power_and_voltage",
        "voltage_v": source,
    }


def read_plug(definition: PlugDefinition) -> dict[str, Any]:
    """Read supported properties using the model's published MIoT identifiers."""
    device = _device(definition)
    telemetry: dict[str, Any] = {}
    property_errors: dict[str, int] = {}
    try:
        for batch in _property_batches():
            request = [
                {"did": name, "siid": service_id, "piid": property_id}
                for name, service_id, property_id in batch
            ]
            response = device.send("get_properties", request, retry_count=1)
            if not isinstance(response, list):
                raise MiotPlugError("device returned an invalid property response")
            for item in response:
                name = str(item.get("did", ""))
                code = int(item.get("code", -1))
                if code == 0:
                    telemetry[name] = item.get("value")
                elif name:
                    property_errors[name] = code
    except Exception as exc:
        if isinstance(exc, MiotPlugError):
            raise
        raise MiotPlugError(f"{type(exc).__name__}: {exc}") from exc
    fault = telemetry.get("fault")
    telemetry["fault_label"] = FAULT_LABELS.get(fault, f"Unknown fault ({fault})")
    apply_voltage_reference(
        telemetry,
        definition.reference_voltage_v,
        "configured_fallback",
    )
    if property_errors:
        telemetry["property_errors"] = property_errors
    return {**definition.public_identity(), "telemetry": telemetry}


def write_power(definition: PlugDefinition, enabled: bool) -> dict[str, Any]:
    """Set power, then read the device back until the requested state is verified."""
    if not isinstance(enabled, bool):
        raise ValueError("on must be a boolean")
    before_device = read_plug(definition)
    before = bool(before_device["telemetry"].get("on"))
    if before != enabled:
        try:
            response = _device(definition).send(
                "set_properties",
                [{"did": "on", "siid": 2, "piid": 1, "value": enabled}],
                retry_count=1,
            )
        except Exception as exc:
            raise MiotPlugError(f"{type(exc).__name__}: {exc}") from exc
        if not isinstance(response, list) or not response or int(response[0].get("code", -1)) != 0:
            raise MiotPlugError("device rejected the power command")

    after_device = before_device
    for attempt in range(3):
        if attempt:
            time.sleep(0.25)
        after_device = read_plug(definition)
        if bool(after_device["telemetry"].get("on")) == enabled:
            break
    after = bool(after_device["telemetry"].get("on"))
    if after != enabled:
        raise MiotPlugError("power state did not verify after write")
    return {
        "written": before != enabled,
        "verified": True,
        "before": before,
        "after": after,
        "device": after_device,
    }
