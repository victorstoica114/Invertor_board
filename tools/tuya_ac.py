"""Local Tuya air-conditioner telemetry with configurable datapoint mapping."""

from __future__ import annotations

from dataclasses import dataclass
import ipaddress
import json
from pathlib import Path
import re
import time
from typing import Any, Final

import tinytuya


DEVICE_ID_PATTERN: Final[re.Pattern[str]] = re.compile(r"^[a-z0-9][a-z0-9_-]{0,31}$")
DEFAULT_DPS: Final[dict[str, str]] = {
    "power": "1",
    "mode": "4",
    "fan": "5",
    "temperature_unit": "19",
    "current_temperature": "23",
    "target_temperature": "24",
}
DEFAULT_MODE_VALUES: Final[tuple[str, ...]] = ("auto", "cold", "hot", "wet", "wind")
DEFAULT_FAN_VALUES: Final[tuple[str, ...]] = ("auto", "low", "mid", "high")


class TuyaAcError(RuntimeError):
    """Raised when a local Tuya operation does not complete or verify."""


@dataclass(frozen=True)
class AcDefinition:
    id: str
    name: str
    ip: str
    device_id: str
    local_key: str
    product_id: str
    version: float
    temperature_scale: float
    dps: dict[str, str]
    mode_values: tuple[str, ...] = DEFAULT_MODE_VALUES
    fan_values: tuple[str, ...] = DEFAULT_FAN_VALUES
    minimum_temperature_c: float = 16.0
    maximum_temperature_c: float = 30.0
    temperature_step_c: float = 1.0
    current_temperature_scale: float | None = None
    target_temperature_scale: float | None = None

    def public_identity(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "ip": self.ip,
            "product_id": self.product_id,
            "version": self.version,
            "controls": {
                "modes": list(self.mode_values),
                "fan_speeds": list(self.fan_values),
                "temperature": {
                    "minimum_c": self.minimum_temperature_c,
                    "maximum_c": self.maximum_temperature_c,
                    "step_c": self.temperature_step_c,
                },
            },
        }


def load_config(path: Path) -> AcDefinition:
    """Load one private Tuya definition without returning its local key publicly."""
    resolved = path.expanduser().resolve()
    try:
        mode = resolved.stat().st_mode & 0o777
        payload = json.loads(resolved.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
        raise TuyaAcError(f"cannot load AC configuration: {exc}") from exc
    if mode & 0o077:
        raise TuyaAcError(f"AC configuration must use mode 0600, found {mode:04o}")
    if not isinstance(payload, dict):
        raise TuyaAcError("AC configuration must be a JSON object")

    ac_id = str(payload.get("id", "air-conditioner")).strip()
    name = str(payload.get("name", "Air conditioner")).strip()
    ip = str(payload.get("ip", "")).strip()
    device_id = str(payload.get("device_id", "")).strip()
    local_key = str(payload.get("local_key", "")).strip()
    product_id = str(payload.get("product_id", "")).strip()
    try:
        version = float(payload.get("version", 3.3))
        temperature_scale = float(payload.get("temperature_scale", 10))
        current_temperature_scale = float(
            payload.get("current_temperature_scale", temperature_scale)
        )
        target_temperature_scale = float(
            payload.get("target_temperature_scale", temperature_scale)
        )
    except (TypeError, ValueError) as exc:
        raise TuyaAcError("version and temperature scales must be numbers") from exc
    raw_dps = payload.get("dps", {})
    if not isinstance(raw_dps, dict):
        raise TuyaAcError("dps must be an object")
    dps = {**DEFAULT_DPS, **{str(key): str(value) for key, value in raw_dps.items()}}
    raw_controls = payload.get("controls", {})
    if not isinstance(raw_controls, dict):
        raise TuyaAcError("controls must be an object")

    def string_options(name: str, defaults: tuple[str, ...]) -> tuple[str, ...]:
        values = raw_controls.get(name, defaults)
        if not isinstance(values, (list, tuple)) or not values:
            raise TuyaAcError(f"controls.{name} must be a non-empty list")
        options = tuple(str(value).strip() for value in values)
        if any(not value or len(value) > 32 for value in options) or len(set(options)) != len(options):
            raise TuyaAcError(f"controls.{name} contains invalid values")
        return options

    mode_values = string_options("modes", DEFAULT_MODE_VALUES)
    fan_values = string_options("fan_speeds", DEFAULT_FAN_VALUES)
    raw_temperature = raw_controls.get("temperature", {})
    if not isinstance(raw_temperature, dict):
        raise TuyaAcError("controls.temperature must be an object")
    try:
        minimum_temperature_c = float(raw_temperature.get("minimum_c", 16))
        maximum_temperature_c = float(raw_temperature.get("maximum_c", 30))
        temperature_step_c = float(raw_temperature.get("step_c", 1))
    except (TypeError, ValueError) as exc:
        raise TuyaAcError("temperature control limits must be numbers") from exc

    if not DEVICE_ID_PATTERN.fullmatch(ac_id):
        raise TuyaAcError("AC configuration has an invalid id")
    if not name or not device_id or not local_key:
        raise TuyaAcError("AC name, device_id and local_key are required")
    try:
        parsed_ip = ipaddress.ip_address(ip)
    except ValueError as exc:
        raise TuyaAcError("AC configuration has an invalid IP address") from exc
    if parsed_ip.version != 4 or not parsed_ip.is_private:
        raise TuyaAcError("AC must use a private IPv4 address")
    if len(local_key.encode("utf-8")) != 16:
        raise TuyaAcError("Tuya local_key must be exactly 16 bytes")
    if version not in (3.1, 3.2, 3.3, 3.4, 3.5):
        raise TuyaAcError("unsupported Tuya protocol version")
    if min(temperature_scale, current_temperature_scale, target_temperature_scale) <= 0:
        raise TuyaAcError("temperature scales must be greater than zero")
    if not 5 <= minimum_temperature_c < maximum_temperature_c <= 40:
        raise TuyaAcError("temperature limits must be between 5 and 40 C")
    if temperature_step_c <= 0 or temperature_step_c > 5:
        raise TuyaAcError("temperature step must be greater than zero and at most 5 C")
    if not dps.get("power", "").isdigit():
        raise TuyaAcError("the power datapoint must be numeric")
    return AcDefinition(
        id=ac_id,
        name=name,
        ip=ip,
        device_id=device_id,
        local_key=local_key,
        product_id=product_id,
        version=version,
        temperature_scale=temperature_scale,
        dps=dps,
        mode_values=mode_values,
        fan_values=fan_values,
        minimum_temperature_c=minimum_temperature_c,
        maximum_temperature_c=maximum_temperature_c,
        temperature_step_c=temperature_step_c,
        current_temperature_scale=current_temperature_scale,
        target_temperature_scale=target_temperature_scale,
    )


def _device(definition: AcDefinition) -> tinytuya.Device:
    device = tinytuya.Device(
        dev_id=definition.device_id,
        address=definition.ip,
        local_key=definition.local_key,
        version=definition.version,
    )
    device.set_socketPersistent(False)
    device.set_socketTimeout(5)
    device.set_socketRetryLimit(1)
    return device


def _mapped_value(dps: dict[str, Any], definition: AcDefinition, name: str) -> Any:
    return dps.get(definition.dps.get(name, ""))


def _temperature(value: Any, scale: float) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return round(float(value) / scale, 1)


def _temperature_scale(definition: AcDefinition, field: str) -> float:
    configured = getattr(definition, f"{field}_temperature_scale")
    return definition.temperature_scale if configured is None else configured


def read_ac(definition: AcDefinition) -> dict[str, Any]:
    """Read the AC status and expose both mapped fields and non-secret raw DPS."""
    try:
        response = _device(definition).status()
    except Exception as exc:
        raise TuyaAcError(f"{type(exc).__name__}: {exc}") from exc
    if not isinstance(response, dict):
        raise TuyaAcError("device returned an invalid status response")
    if response.get("Error") or response.get("Err"):
        raise TuyaAcError(str(response.get("Error") or response.get("Err")))
    raw_dps = response.get("dps")
    if not isinstance(raw_dps, dict):
        raise TuyaAcError("device response does not contain datapoints")
    dps = {str(key): value for key, value in raw_dps.items()}
    telemetry = {
        "on": _mapped_value(dps, definition, "power"),
        "mode": _mapped_value(dps, definition, "mode"),
        "fan": _mapped_value(dps, definition, "fan"),
        "temperature_unit": _mapped_value(dps, definition, "temperature_unit"),
        "current_temperature_c": _temperature(
            _mapped_value(dps, definition, "current_temperature"),
            _temperature_scale(definition, "current"),
        ),
        "target_temperature_c": _temperature(
            _mapped_value(dps, definition, "target_temperature"),
            _temperature_scale(definition, "target"),
        ),
        "raw_dps": dps,
    }
    return {**definition.public_identity(), "telemetry": telemetry}


def _normalized_setting(definition: AcDefinition, setting: str, value: Any) -> tuple[str, Any, Any]:
    if setting == "power":
        if not isinstance(value, bool):
            raise ValueError("power must be a boolean")
        return "on", value, value
    if setting == "mode":
        requested = str(value)
        if requested not in definition.mode_values:
            raise ValueError("unsupported air-conditioner mode")
        return "mode", requested, requested
    if setting == "fan":
        requested = str(value)
        if requested not in definition.fan_values:
            raise ValueError("unsupported air-conditioner fan speed")
        return "fan", requested, requested
    if setting == "target_temperature_c":
        if isinstance(value, bool):
            raise ValueError("target temperature must be a number")
        try:
            requested = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("target temperature must be a number") from exc
        if not definition.minimum_temperature_c <= requested <= definition.maximum_temperature_c:
            raise ValueError(
                "target temperature is outside the configured control range"
            )
        offset = (requested - definition.minimum_temperature_c) / definition.temperature_step_c
        if abs(offset - round(offset)) > 1e-6:
            raise ValueError("target temperature does not match the configured step")
        raw = requested * _temperature_scale(definition, "target")
        raw_value: int | float = int(raw) if raw.is_integer() else raw
        return "target_temperature_c", requested, raw_value
    raise ValueError("unsupported air-conditioner setting")


def _values_match(first: Any, second: Any) -> bool:
    if isinstance(first, (int, float)) and isinstance(second, (int, float)):
        return abs(float(first) - float(second)) < 1e-6
    return first == second


def write_setting(definition: AcDefinition, setting: str, value: Any) -> dict[str, Any]:
    """Set one supported AC field and verify it by reading the device back."""
    telemetry_key, requested, raw_value = _normalized_setting(definition, setting, value)
    before_device = read_ac(definition)
    before = before_device["telemetry"].get(telemetry_key)
    if not _values_match(before, requested):
        datapoint_name = "power" if setting == "power" else (
            "target_temperature" if setting == "target_temperature_c" else setting
        )
        try:
            response = _device(definition).set_value(
                int(definition.dps[datapoint_name]), raw_value, nowait=False
            )
        except Exception as exc:
            raise TuyaAcError(f"{type(exc).__name__}: {exc}") from exc
        if not isinstance(response, dict) or response.get("Error") or response.get("Err"):
            raise TuyaAcError(f"device rejected the {setting} command: {response}")

    after_device = before_device
    for attempt in range(3):
        if attempt:
            time.sleep(0.35)
        after_device = read_ac(definition)
        if _values_match(after_device["telemetry"].get(telemetry_key), requested):
            break
    after = after_device["telemetry"].get(telemetry_key)
    if not _values_match(after, requested):
        raise TuyaAcError(f"{setting} did not verify after write")
    return {
        "setting": setting,
        "written": not _values_match(before, requested),
        "verified": True,
        "before": before,
        "after": after,
        "device": after_device,
    }


def write_power(definition: AcDefinition, enabled: bool) -> dict[str, Any]:
    """Set AC power and verify the requested state by reading it back."""
    if not isinstance(enabled, bool):
        raise ValueError("on must be a boolean")
    return write_setting(definition, "power", enabled)
