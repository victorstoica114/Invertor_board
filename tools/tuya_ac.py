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
    "target_temperature": "2",
    "current_temperature": "3",
    "mode": "4",
    "fan": "5",
    "current_humidity": "18",
    "fault": "20",
    "pm25": "101",
    "sleep": "105",
    "capability_flags": "110",
    "vertical_swing": "113",
    "horizontal_swing": "114",
    "energy_saving": "119",
    "generator_mode": "120",
    "fault_secondary": "122",
    "advanced_flags": "123",
    "air_quality": "125",
    "vertical_position": "126",
    "horizontal_position": "127",
    "model_code": "128",
    "energy_quota": "129",
    "eco_temperature": "130",
    "filter_dirty": "131",
    "hot_cold_air": "132",
    "swing_action": "133",
    "statistics": "134",
    "running_time": "135",
    "service_value": "136",
}
DEFAULT_MODE_VALUES: Final[tuple[str, ...]] = ("auto", "cold", "hot", "wet", "wind")
DEFAULT_FAN_VALUES: Final[tuple[str, ...]] = (
    "strong", "high", "mid_high", "mid", "mid_low", "low", "mute", "auto"
)
DEFAULT_SLEEP_VALUES: Final[tuple[str, ...]] = ("off", "normal", "old", "child")
DEFAULT_VERTICAL_SWING_VALUES: Final[tuple[str, ...]] = ("0", "1", "2", "3")
DEFAULT_HORIZONTAL_SWING_VALUES: Final[tuple[str, ...]] = (
    "0", "1", "2", "3", "4", "5", "6", "7"
)
DEFAULT_VERTICAL_POSITION_VALUES: Final[tuple[str, ...]] = ("0", "1", "2", "3", "4", "5")
DEFAULT_HORIZONTAL_POSITION_VALUES: Final[tuple[str, ...]] = ("0", "1", "2", "3", "4", "5")
DEFAULT_ENERGY_SAVING_VALUES: Final[tuple[str, ...]] = ("0", "1", "2", "3")
DEFAULT_GENERATOR_MODE_VALUES: Final[tuple[str, ...]] = ("off", "L1", "L2", "L3")

OPTION_LABELS: Final[dict[str, dict[str, str]]] = {
    "sleep": {"off": "Off", "normal": "Standard", "old": "Elderly", "child": "Child"},
    "vertical_swing": {"0": "Off", "1": "Full", "2": "Upper", "3": "Lower"},
    "horizontal_swing": {
        "0": "Off", "1": "Full", "2": "Left", "3": "Center", "4": "Right",
        "5": "Slight left", "6": "Slight right", "7": "Wide angle",
    },
    "vertical_position": {
        "0": "Hold current", "1": "Top", "2": "Slightly up", "3": "Middle",
        "4": "Slightly down", "5": "Bottom",
    },
    "horizontal_position": {
        "0": "Hold current", "1": "Leftmost", "2": "Slight left", "3": "Center",
        "4": "Slight right", "5": "Rightmost",
    },
    "energy_saving": {
        "0": "Off", "1": "Automatic", "2": "Power limit", "3": "Temperature limit",
    },
    "generator_mode": {"off": "Off", "L1": "30%", "L2": "50%", "L3": "80%"},
}

ADVANCED_FLAGS: Final[dict[str, int]] = {
    "eco": 0x0001,
    "self_clean": 0x0004,
    "display": 0x0008,
    "buzzer": 0x0010,
    "health": 0x0020,
    "anti_mildew": 0x0100,
    "anti_frost": 0x1000,
    "soft_wind": 0x8000,
}
ADVANCED_FLAG_LABELS: Final[dict[str, str]] = {
    "eco": "ECO",
    "display": "Display",
    "buzzer": "Buzzer",
    "anti_mildew": "Anti-mildew",
    "health": "Health",
    "self_clean": "Self-cleaning",
    "anti_frost": "8 C heating",
    "soft_wind": "Soft airflow",
}
CAPABILITY_FLAGS: Final[dict[str, int]] = {
    "dry_temperature_control": 0,
    "fan_temperature_control": 1,
    "auto_temperature_control": 2,
    "fresh_air_volume": 3,
    "vertical_positioning": 4,
    "horizontal_positioning": 5,
    "light_sensor": 6,
    "anti_mildew": 7,
    "humidity_sensor": 8,
    "self_clean": 9,
    "energy_saving": 10,
    "power_statistics": 11,
    "generator_mode": 12,
    "hot_cold_air": 13,
    "air_quality": 14,
    "anti_frost": 17,
    "filter_dirty": 18,
    "pm25": 20,
    "fahrenheit": 21,
    "soft_wind": 22,
    "wide_horizontal_positioning": 23,
    "fresh_air": 24,
}
AIR_QUALITY_LABELS: Final[dict[str, str]] = {
    "great": "Excellent", "good": "Good", "middle": "Moderate", "bad": "Poor",
    "varybad": "Severe",
}
ENUM_SETTING_ATTRIBUTES: Final[dict[str, str]] = {
    "mode": "mode_values",
    "fan": "fan_values",
    "sleep": "sleep_values",
    "vertical_swing": "vertical_swing_values",
    "horizontal_swing": "horizontal_swing_values",
    "vertical_position": "vertical_position_values",
    "horizontal_position": "horizontal_position_values",
    "energy_saving": "energy_saving_values",
    "generator_mode": "generator_mode_values",
}
WRITABLE_SETTINGS: Final[frozenset[str]] = frozenset({
    "mode", "fan", "target_temperature_c", "sleep", "vertical_swing",
    "horizontal_swing", "vertical_position", "horizontal_position", "energy_saving",
    "generator_mode", "eco_temperature_c", "hot_cold_air", *ADVANCED_FLAGS,
})


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
    sleep_values: tuple[str, ...] = DEFAULT_SLEEP_VALUES
    vertical_swing_values: tuple[str, ...] = DEFAULT_VERTICAL_SWING_VALUES
    horizontal_swing_values: tuple[str, ...] = DEFAULT_HORIZONTAL_SWING_VALUES
    vertical_position_values: tuple[str, ...] = DEFAULT_VERTICAL_POSITION_VALUES
    horizontal_position_values: tuple[str, ...] = DEFAULT_HORIZONTAL_POSITION_VALUES
    energy_saving_values: tuple[str, ...] = DEFAULT_ENERGY_SAVING_VALUES
    generator_mode_values: tuple[str, ...] = DEFAULT_GENERATOR_MODE_VALUES

    @staticmethod
    def _options(setting: str, values: tuple[str, ...]) -> list[dict[str, str]]:
        labels = OPTION_LABELS.get(setting, {})
        return [{"value": value, "label": labels.get(value, value)} for value in values]

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
                "sleep": self._options("sleep", self.sleep_values),
                "vertical_swing": self._options(
                    "vertical_swing", self.vertical_swing_values
                ),
                "horizontal_swing": self._options(
                    "horizontal_swing", self.horizontal_swing_values
                ),
                "vertical_position": self._options(
                    "vertical_position", self.vertical_position_values
                ),
                "horizontal_position": self._options(
                    "horizontal_position", self.horizontal_position_values
                ),
                "energy_saving": self._options(
                    "energy_saving", self.energy_saving_values
                ),
                "generator_mode": self._options(
                    "generator_mode", self.generator_mode_values
                ),
                "eco_temperature": {"minimum_c": 26, "maximum_c": 31, "step_c": 1},
                "advanced_switches": [
                    {"setting": setting, "label": ADVANCED_FLAG_LABELS[setting]}
                    for setting in ADVANCED_FLAGS
                ],
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
    sleep_values = string_options("sleep", DEFAULT_SLEEP_VALUES)
    vertical_swing_values = string_options(
        "vertical_swing", DEFAULT_VERTICAL_SWING_VALUES
    )
    horizontal_swing_values = string_options(
        "horizontal_swing", DEFAULT_HORIZONTAL_SWING_VALUES
    )
    vertical_position_values = string_options(
        "vertical_position", DEFAULT_VERTICAL_POSITION_VALUES
    )
    horizontal_position_values = string_options(
        "horizontal_position", DEFAULT_HORIZONTAL_POSITION_VALUES
    )
    energy_saving_values = string_options(
        "energy_saving", DEFAULT_ENERGY_SAVING_VALUES
    )
    generator_mode_values = string_options(
        "generator_mode", DEFAULT_GENERATOR_MODE_VALUES
    )
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
        sleep_values=sleep_values,
        vertical_swing_values=vertical_swing_values,
        horizontal_swing_values=horizontal_swing_values,
        vertical_position_values=vertical_position_values,
        horizontal_position_values=horizontal_position_values,
        energy_saving_values=energy_saving_values,
        generator_mode_values=generator_mode_values,
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


def _integer(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float) and value.is_integer():
        return int(value)
    if isinstance(value, str):
        try:
            return int(value.strip(), 10)
        except ValueError:
            return None
    return None


def _hex_flags(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        try:
            return int(value.strip(), 16)
        except ValueError:
            return None
    return None


def _json_value(value: Any) -> Any:
    if not isinstance(value, str):
        return value
    try:
        return json.loads(value)
    except json.JSONDecodeError:
        return value


def _number_or_none(value: Any) -> int | float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return value


def _option_label(setting: str, value: Any) -> str | None:
    if value is None:
        return None
    raw = str(value)
    return OPTION_LABELS.get(setting, {}).get(raw, raw)


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
    capability_raw = _integer(_mapped_value(dps, definition, "capability_flags"))
    advanced_raw_value = _mapped_value(dps, definition, "advanced_flags")
    advanced_raw = _hex_flags(advanced_raw_value)
    capabilities = {
        name: bool(capability_raw & (1 << bit)) if capability_raw is not None else None
        for name, bit in CAPABILITY_FLAGS.items()
    }
    advanced = {
        name: bool(advanced_raw & mask) if advanced_raw is not None else None
        for name, mask in ADVANCED_FLAGS.items()
    }
    explicit_temperature_unit = _mapped_value(dps, definition, "temperature_unit")
    if explicit_temperature_unit is None:
        temperature_unit = "F" if capabilities["fahrenheit"] else "C"
    else:
        temperature_unit = str(explicit_temperature_unit).upper()
    fault_code = _integer(_mapped_value(dps, definition, "fault"))
    fault_secondary = _integer(_mapped_value(dps, definition, "fault_secondary"))
    active_faults = [value for value in (fault_code, fault_secondary) if value not in (None, 0)]
    fault_label = "No faults" if not active_faults else ", ".join(
        f"0x{value:X}" for value in active_faults
    )
    current_humidity = _number_or_none(_mapped_value(dps, definition, "current_humidity"))
    if current_humidity == 0:
        current_humidity = None
    datapoint_names = {
        "target_temperature_c": "target_temperature",
        "eco_temperature_c": "eco_temperature",
    }
    available_controls = {
        setting: definition.dps.get(datapoint_names.get(setting, setting), "") in dps
        for setting in WRITABLE_SETTINGS - ADVANCED_FLAGS.keys()
    }
    available_controls.update({setting: advanced_raw is not None for setting in ADVANCED_FLAGS})
    air_quality = _mapped_value(dps, definition, "air_quality")
    telemetry = {
        "on": _mapped_value(dps, definition, "power"),
        "mode": _mapped_value(dps, definition, "mode"),
        "fan": _mapped_value(dps, definition, "fan"),
        "temperature_unit": temperature_unit,
        "current_temperature_c": _temperature(
            _mapped_value(dps, definition, "current_temperature"),
            _temperature_scale(definition, "current"),
        ),
        "target_temperature_c": _temperature(
            _mapped_value(dps, definition, "target_temperature"),
            _temperature_scale(definition, "target"),
        ),
        "current_humidity_pct": current_humidity,
        "sleep": _mapped_value(dps, definition, "sleep"),
        "sleep_label": _option_label("sleep", _mapped_value(dps, definition, "sleep")),
        "vertical_swing": _mapped_value(dps, definition, "vertical_swing"),
        "vertical_swing_label": _option_label(
            "vertical_swing", _mapped_value(dps, definition, "vertical_swing")
        ),
        "horizontal_swing": _mapped_value(dps, definition, "horizontal_swing"),
        "horizontal_swing_label": _option_label(
            "horizontal_swing", _mapped_value(dps, definition, "horizontal_swing")
        ),
        "vertical_position": _mapped_value(dps, definition, "vertical_position"),
        "vertical_position_label": _option_label(
            "vertical_position", _mapped_value(dps, definition, "vertical_position")
        ),
        "horizontal_position": _mapped_value(dps, definition, "horizontal_position"),
        "horizontal_position_label": _option_label(
            "horizontal_position", _mapped_value(dps, definition, "horizontal_position")
        ),
        "energy_saving": _mapped_value(dps, definition, "energy_saving"),
        "energy_saving_label": _option_label(
            "energy_saving", _mapped_value(dps, definition, "energy_saving")
        ),
        "generator_mode": _mapped_value(dps, definition, "generator_mode"),
        "generator_mode_label": _option_label(
            "generator_mode", _mapped_value(dps, definition, "generator_mode")
        ),
        "eco_temperature_c": _number_or_none(
            _mapped_value(dps, definition, "eco_temperature")
        ),
        "hot_cold_air": _mapped_value(dps, definition, "hot_cold_air"),
        "air_quality": air_quality,
        "air_quality_label": AIR_QUALITY_LABELS.get(str(air_quality), str(air_quality))
        if air_quality is not None else None,
        "pm25_ug_m3": _number_or_none(_mapped_value(dps, definition, "pm25")),
        "filter_dirty": _mapped_value(dps, definition, "filter_dirty"),
        "fault_code": fault_code,
        "fault_secondary_code": fault_secondary,
        "fault_label": fault_label,
        "capability_flags_raw": capability_raw,
        "capabilities": capabilities,
        "advanced_flags_raw": advanced_raw_value,
        **advanced,
        "available_controls": available_controls,
        "model_code": _mapped_value(dps, definition, "model_code"),
        "energy_quota_raw": _mapped_value(dps, definition, "energy_quota"),
        "swing_action_raw": _mapped_value(dps, definition, "swing_action"),
        "statistics": _json_value(_mapped_value(dps, definition, "statistics")),
        "running_time_raw": _mapped_value(dps, definition, "running_time"),
        "service_value_136_raw": _mapped_value(dps, definition, "service_value"),
        "raw_dps": dps,
    }
    return {**definition.public_identity(), "telemetry": telemetry}


def _normalized_setting(definition: AcDefinition, setting: str, value: Any) -> tuple[str, Any, Any]:
    if setting == "power":
        if not isinstance(value, bool):
            raise ValueError("power must be a boolean")
        return "on", value, value
    if setting in ENUM_SETTING_ATTRIBUTES:
        requested = str(value)
        allowed = getattr(definition, ENUM_SETTING_ATTRIBUTES[setting])
        if requested not in allowed:
            raise ValueError(f"unsupported air-conditioner {setting.replace('_', ' ')}")
        return setting, requested, requested
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
    if setting == "eco_temperature_c":
        if isinstance(value, bool):
            raise ValueError("ECO temperature must be a number")
        try:
            requested = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("ECO temperature must be a number") from exc
        if not 26 <= requested <= 31 or not requested.is_integer():
            raise ValueError("ECO temperature must be a whole number from 26 to 31 C")
        return "eco_temperature_c", int(requested), int(requested)
    if setting == "hot_cold_air" or setting in ADVANCED_FLAGS:
        if not isinstance(value, bool):
            raise ValueError(f"{setting.replace('_', ' ')} must be a boolean")
        return setting, value, value
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
    datapoint_name = "power" if setting == "power" else (
        "target_temperature" if setting == "target_temperature_c" else (
            "eco_temperature" if setting == "eco_temperature_c" else setting
        )
    )
    if setting in ADVANCED_FLAGS:
        if setting == "self_clean" and requested and before_device["telemetry"].get("on"):
            raise ValueError("self-cleaning can only be started while the air conditioner is off")
        advanced_raw = _hex_flags(before_device["telemetry"].get("advanced_flags_raw"))
        if advanced_raw is None:
            raise TuyaAcError("advanced-function flags are unavailable")
        mask = ADVANCED_FLAGS[setting]
        updated_flags = advanced_raw | mask if requested else advanced_raw & ~mask
        raw_value = f"{updated_flags:04x}"
        datapoint_name = "advanced_flags"
    if not _values_match(before, requested):
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
