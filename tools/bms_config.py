#!/usr/bin/env python3
"""Guarded Bluetooth configuration for the local Daly, Seplos and JK BMSs.

Bluetooth is used here only for configuration.  Operational telemetry remains
the responsibility of the ESP32 bridges.  Every write is preceded by an
identity check and followed by a fresh configuration read-back.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, replace
from typing import Any, Final

try:
    from tools import bms_ble
except ModuleNotFoundError:  # Direct execution/import from tools/.
    import bms_ble  # type: ignore[no-redef]


@dataclass(frozen=True)
class SettingSpec:
    key: str
    label: str
    group: str
    register: int
    unit: str = ""
    factor: float = 1.0
    offset: float = 0.0
    signed: bool = False
    kind: str = "number"
    minimum: float | None = None
    maximum: float | None = None
    step: float | None = None
    options: tuple[tuple[int | str, str], ...] = ()
    critical: bool = True
    description: str = ""
    source: str = "register"
    frame_offset: int | None = None
    length: int = 4
    bit: int | None = None
    bits: int = 16


GROUP_TITLES: Final[dict[str, str]] = {
    "identity": "Identity and communication",
    "battery": "Battery and topology",
    "voltage": "Voltage thresholds",
    "current": "Current and short-circuit protection",
    "temperature": "Temperature protection",
    "balancing": "Balancing",
    "capacity": "Capacity and state of charge",
    "timing": "Timing and sleep",
    "switches": "MOSFETs and feature switches",
    "inverter": "Inverter communication",
    "precharge": "Precharge and output control",
    "other": "Other parameters",
}


def _spec_item(spec: SettingSpec, raw: int, value: Any, *, writable: bool = True) -> dict[str, Any]:
    item: dict[str, Any] = {
        "key": spec.key,
        "label": spec.label,
        "type": "select" if spec.options else spec.kind,
        "value": value,
        "raw": raw,
        "writable": writable,
        "critical": spec.critical,
        "description": spec.description,
    }
    if spec.unit:
        item["unit"] = spec.unit
    if spec.minimum is not None:
        item["minimum"] = spec.minimum
    if spec.maximum is not None:
        item["maximum"] = spec.maximum
    if spec.step is not None:
        item["step"] = spec.step
    if spec.options:
        item["options"] = [{"value": value, "label": label} for value, label in spec.options]
    return item


def _decode_number(spec: SettingSpec, raw: int) -> int | float | bool:
    if spec.signed and raw & (1 << (spec.bits - 1)):
        raw -= 1 << spec.bits
    if spec.kind == "bool":
        return bool(raw)
    value = (raw - spec.offset) / spec.factor
    return int(value) if spec.factor == 1 and spec.offset == 0 else round(value, 4)


def _encode_number(spec: SettingSpec, value: Any) -> int:
    if spec.kind == "bool":
        if isinstance(value, str):
            if value.casefold() in ("true", "on", "enabled", "1"):
                value = True
            elif value.casefold() in ("false", "off", "disabled", "0"):
                value = False
            else:
                raise ValueError(f"{spec.label} must be enabled or disabled")
        if not isinstance(value, (bool, int)):
            raise ValueError(f"{spec.label} must be enabled or disabled")
        return 1 if bool(value) else 0
    if spec.options:
        matching = [option for option, _label in spec.options if value == option or str(value) == str(option)]
        if not matching:
            raise ValueError(f"{spec.label} must be one of the supported options")
        if not isinstance(matching[0], int):
            raise ValueError(f"{spec.label} uses a non-numeric protocol selector")
        return matching[0]
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{spec.label} must be numeric") from exc
    if spec.minimum is not None and numeric < spec.minimum:
        raise ValueError(f"{spec.label} must be at least {spec.minimum}")
    if spec.maximum is not None and numeric > spec.maximum:
        raise ValueError(f"{spec.label} must be at most {spec.maximum}")
    raw = round(numeric * spec.factor + spec.offset)
    minimum_raw = -(1 << (spec.bits - 1)) if spec.signed else 0
    maximum_raw = (1 << (spec.bits - 1)) - 1 if spec.signed else (1 << spec.bits) - 1
    if not minimum_raw <= raw <= maximum_raw:
        raise ValueError(f"{spec.label} cannot be represented by this BMS")
    return raw & ((1 << spec.bits) - 1)


def _groups(specs: tuple[SettingSpec, ...], raw_values: dict[str, int]) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for spec in specs:
        raw = raw_values[spec.key]
        grouped.setdefault(spec.group, []).append(_spec_item(spec, raw, _decode_number(spec, raw)))
    return [
        {"key": group, "title": GROUP_TITLES.get(group, group.replace("_", " ").title()), "settings": items}
        for group, items in grouped.items()
    ]


DALY_SPECS: Final[tuple[SettingSpec, ...]] = (
    SettingSpec("rated_capacity", "Rated capacity", "battery", 0x0080, "Ah", 10, minimum=0, maximum=6553.5, step=0.1),
    SettingSpec("cell_voltage_reference", "Cell voltage reference", "battery", 0x0081, "mV", minimum=0, maximum=65535, step=1),
    SettingSpec("acquisition_board_count", "Acquisition board count", "battery", 0x0082, minimum=1, maximum=3, step=1),
    SettingSpec("board_1_cell_count", "Board 1 cell count", "battery", 0x0083, minimum=0, maximum=32, step=1),
    SettingSpec("board_2_cell_count", "Board 2 cell count", "battery", 0x0084, minimum=0, maximum=32, step=1),
    SettingSpec("board_3_cell_count", "Board 3 cell count", "battery", 0x0085, minimum=0, maximum=32, step=1),
    SettingSpec("board_1_temperature_sensor_count", "Board 1 temperature sensor count", "battery", 0x0086, minimum=0, maximum=8, step=1),
    SettingSpec("board_2_temperature_sensor_count", "Board 2 temperature sensor count", "battery", 0x0087, minimum=0, maximum=8, step=1),
    SettingSpec("board_3_temperature_sensor_count", "Board 3 temperature sensor count", "battery", 0x0088, minimum=0, maximum=8, step=1),
    SettingSpec("battery_type", "Battery type", "battery", 0x0089, kind="select", options=((0, "LiFePO4"), (1, "Lithium ion"), (2, "LTO"))),
    SettingSpec("sleep_wait_time", "Sleep wait time", "timing", 0x008A, "s", minimum=0, maximum=65535, step=1),
    SettingSpec("cell_overvoltage_warning", "Cell overvoltage warning", "voltage", 0x008B, "mV", minimum=0, maximum=65535, step=1),
    SettingSpec("cell_overvoltage_alarm", "Cell overvoltage alarm", "voltage", 0x008C, "mV", minimum=0, maximum=65535, step=1),
    SettingSpec("cell_undervoltage_warning", "Cell undervoltage warning", "voltage", 0x008D, "mV", minimum=0, maximum=65535, step=1),
    SettingSpec("cell_undervoltage_alarm", "Cell undervoltage alarm", "voltage", 0x008E, "mV", minimum=0, maximum=65535, step=1),
    SettingSpec("total_overvoltage_warning", "Pack overvoltage warning", "voltage", 0x008F, "V", 10, minimum=0, maximum=6553.5, step=0.1),
    SettingSpec("total_overvoltage_alarm", "Pack overvoltage alarm", "voltage", 0x0090, "V", 10, minimum=0, maximum=6553.5, step=0.1),
    SettingSpec("total_undervoltage_warning", "Pack undervoltage warning", "voltage", 0x0091, "V", 10, minimum=0, maximum=6553.5, step=0.1),
    SettingSpec("total_undervoltage_alarm", "Pack undervoltage alarm", "voltage", 0x0092, "V", 10, minimum=0, maximum=6553.5, step=0.1),
    SettingSpec("charging_overcurrent_warning", "Charge overcurrent warning", "current", 0x0093, "A", 10, 30000, minimum=-3000, maximum=3553.5, step=0.1),
    SettingSpec("charging_overcurrent_alarm", "Charge overcurrent alarm", "current", 0x0094, "A", 10, 30000, minimum=-3000, maximum=3553.5, step=0.1),
    SettingSpec("discharging_overcurrent_warning", "Discharge overcurrent warning", "current", 0x0095, "A", 10, 30000, minimum=-3000, maximum=3553.5, step=0.1),
    SettingSpec("discharging_overcurrent_alarm", "Discharge overcurrent alarm", "current", 0x0096, "A", 10, 30000, minimum=-3000, maximum=3553.5, step=0.1),
    SettingSpec("charging_overtemperature_warning", "Charge overtemperature warning", "temperature", 0x0097, "°C", 1, 40, minimum=-40, maximum=100, step=1),
    SettingSpec("charging_overtemperature_alarm", "Charge overtemperature alarm", "temperature", 0x0098, "°C", 1, 40, minimum=-40, maximum=100, step=1),
    SettingSpec("charging_undertemperature_warning", "Charge undertemperature warning", "temperature", 0x0099, "°C", 1, 40, minimum=-40, maximum=100, step=1),
    SettingSpec("charging_undertemperature_alarm", "Charge undertemperature alarm", "temperature", 0x009A, "°C", 1, 40, minimum=-40, maximum=100, step=1),
    SettingSpec("discharging_overtemperature_warning", "Discharge overtemperature warning", "temperature", 0x009B, "°C", 1, 40, minimum=-40, maximum=100, step=1),
    SettingSpec("discharging_overtemperature_alarm", "Discharge overtemperature alarm", "temperature", 0x009C, "°C", 1, 40, minimum=-40, maximum=100, step=1),
    SettingSpec("discharging_undertemperature_warning", "Discharge undertemperature warning", "temperature", 0x009D, "°C", 1, 40, minimum=-40, maximum=100, step=1),
    SettingSpec("discharging_undertemperature_alarm", "Discharge undertemperature alarm", "temperature", 0x009E, "°C", 1, 40, minimum=-40, maximum=100, step=1),
    SettingSpec("cell_voltage_difference_warning", "Cell voltage difference warning", "voltage", 0x009F, "mV", minimum=0, maximum=65535, step=1),
    SettingSpec("cell_voltage_difference_alarm", "Cell voltage difference alarm", "voltage", 0x00A0, "mV", minimum=0, maximum=65535, step=1),
    SettingSpec("temperature_difference_warning", "Temperature difference warning", "temperature", 0x00A1, "°C", minimum=0, maximum=100, step=1),
    SettingSpec("temperature_difference_alarm", "Temperature difference alarm", "temperature", 0x00A2, "°C", minimum=0, maximum=100, step=1),
    SettingSpec("balancing_activation_voltage", "Balancing activation voltage", "balancing", 0x00A3, "mV", minimum=0, maximum=65535, step=1),
    SettingSpec("balancing_activation_difference", "Balancing activation difference", "balancing", 0x00A4, "mV", minimum=0, maximum=65535, step=1),
    SettingSpec("charging", "Charge MOSFET", "switches", 0x00A5, kind="bool"),
    SettingSpec("discharging", "Discharge MOSFET", "switches", 0x00A6, kind="bool"),
    SettingSpec("state_of_charge", "State of charge calibration", "capacity", 0x00A7, "%", 10, minimum=0, maximum=100, step=0.1),
    SettingSpec("mosfet_overtemperature_alarm", "MOSFET overtemperature alarm", "temperature", 0x00A8, "°C", 1, 40, minimum=-40, maximum=100, step=1),
    SettingSpec("balancer", "Balancer", "switches", 0x00CF, kind="bool"),
)


class _DalySession:
    def __init__(self, client: Any) -> None:
        self.client = client
        self.event = asyncio.Event()
        self.buffer = bytearray()

    def _expected_length(self) -> int | None:
        if len(self.buffer) < 2:
            return None
        if self.buffer[1] == 0x03:
            return 3 + self.buffer[2] + 2 if len(self.buffer) >= 3 else None
        if self.buffer[1] == 0x06:
            return 8
        return None

    def notification(self, _sender: Any, data: bytearray) -> None:
        if len(data) >= 2 and data[0] == 0xD2 and data[1] in (0x03, 0x06):
            self.buffer.clear()
            self.event.clear()
        self.buffer.extend(data)
        expected = self._expected_length()
        if expected is None or len(self.buffer) < expected:
            return
        frame = self.buffer[:expected]
        if bms_ble.crc16_modbus(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
            return
        self.event.set()

    async def command(self, function: int, register: int, value_or_count: int) -> bytes:
        self.buffer.clear()
        self.event.clear()
        frame = bytearray((0xD2, function, register >> 8, register & 0xFF, value_or_count >> 8, value_or_count & 0xFF))
        frame.extend(bms_ble.crc16_modbus(frame).to_bytes(2, "little"))
        await self.client.write_gatt_char(bms_ble.normalize_uuid("fff2"), bytes(frame), response=False)
        await asyncio.wait_for(self.event.wait(), timeout=8)
        expected = self._expected_length()
        if expected is None:
            raise RuntimeError("Daly returned an incomplete response")
        message = bytes(self.buffer[:expected])
        if message[0:2] != bytes((0xD2, function)):
            raise RuntimeError("Daly returned an unexpected response")
        return message

    async def read(self, register: int, count: int) -> bytes:
        frame = await self.command(0x03, register, count)
        if frame[2] != count * 2:
            raise RuntimeError("Daly returned the wrong register count")
        return frame[3:-2]

    async def write(self, register: int, value: int) -> None:
        frame = await self.command(0x06, register, value)
        if int.from_bytes(frame[2:4], "big") != register or int.from_bytes(frame[4:6], "big") != value:
            raise RuntimeError("Daly write acknowledgement does not match the request")


async def _read_daly_session(session: _DalySession) -> dict[str, Any]:
    settings = await session.read(0x0080, 41)
    balancer = await session.read(0x00CF, 1)
    try:
        version = await session.read(0x00A9, 32)
    except TimeoutError:
        # Some Daly BLE bridges intermittently omit this optional 64-byte
        # identity block.  Configuration registers and the MAC identity are
        # still valid, so a missing version must not disable guarded writes.
        version = b""
    registers = {
        f"0x{address:04X}": int.from_bytes(settings[(address - 0x80) * 2 : (address - 0x80) * 2 + 2], "big")
        for address in range(0x80, 0xA9)
    }
    registers["0x00CF"] = int.from_bytes(balancer, "big")
    raw_values = {spec.key: registers[f"0x{spec.register:04X}"] for spec in DALY_SPECS}
    return {
        "device": "daly",
        "protocol": "Daly D2 Modbus BLE",
        "identity": {
            "model": "Daly smart BMS",
            "hardware": bms_ble._decode_ascii(version[32:64]) or None,
            "firmware": bms_ble._decode_ascii(version[:32]) or None,
            "address": bms_ble.DEVICE_INVENTORY["daly"]["address"],
            "name": bms_ble.DEVICE_INVENTORY["daly"]["advertised_name"],
            "confirmation": bms_ble.DEVICE_INVENTORY["daly"]["address"],
        },
        "groups": _groups(DALY_SPECS, raw_values),
        "raw": {"registers": registers},
    }


async def read_daly_configuration() -> dict[str, Any]:
    BleakClient, scanner, _ = bms_ble._ble_imports()
    entry = bms_ble.DEVICE_INVENTORY["daly"]
    device = await bms_ble._find_device(scanner, entry["address"])
    async with BleakClient(device, timeout=15) as client:
        session = _DalySession(client)
        await client.start_notify(bms_ble.normalize_uuid("fff1"), session.notification)
        return await _read_daly_session(session)


async def write_daly_setting(key: str, value: Any, confirmation: str) -> dict[str, Any]:
    spec = next((item for item in DALY_SPECS if item.key == key), None)
    if spec is None:
        raise ValueError("unknown Daly setting")
    entry = bms_ble.DEVICE_INVENTORY["daly"]
    if confirmation.upper() != entry["address"]:
        raise ValueError(f"confirmation must exactly match the Daly MAC address: {entry['address']}")
    target_raw = _encode_number(spec, value)
    BleakClient, scanner, _ = bms_ble._ble_imports()
    device = await bms_ble._find_device(scanner, entry["address"])
    async with BleakClient(device, timeout=15) as client:
        session = _DalySession(client)
        await client.start_notify(bms_ble.normalize_uuid("fff1"), session.notification)
        before_configuration = await _read_daly_session(session)
        before = next(item for group in before_configuration["groups"] for item in group["settings"] if item["key"] == key)
        if before["raw"] == target_raw:
            return {"written": False, "verified": True, "before": before["value"], "after": before["value"], "after_configuration": before_configuration}
        await session.write(spec.register, target_raw)
        await asyncio.sleep(0.5)
        after_configuration = await _read_daly_session(session)
    after = next(item for group in after_configuration["groups"] for item in group["settings"] if item["key"] == key)
    if after["raw"] != target_raw:
        raise RuntimeError("Daly did not report the requested value after the write")
    return {"written": True, "verified": True, "before": before["value"], "after": after["value"], "after_configuration": after_configuration}


def _seplos_spec(
    key: str,
    label: str,
    group: str,
    register: int,
    unit: str = "",
    factor: float = 1,
    offset: float = 0,
    signed: bool = False,
    minimum: float | None = None,
    maximum: float | None = None,
) -> SettingSpec:
    return SettingSpec(
        key, label, group, register, unit, factor, offset, signed,
        minimum=minimum, maximum=maximum,
        step=round(1 / factor, 4) if factor >= 1 else 1,
    )


_SEPLOS_ROWS: Final[tuple[tuple[int, str, str, str, float, float, bool, float | None, float | None], ...]] = (
    (0x1301, "Cell count", "battery", "", 1, 0, False, 1, 32),
    (0x1302, "Pack high-voltage recovery", "voltage", "V", 100, 0, False, 0, 1000),
    (0x1303, "Pack high-voltage alarm", "voltage", "V", 100, 0, False, 0, 1000),
    (0x1304, "Pack overvoltage recovery", "voltage", "V", 100, 0, False, 0, 1000),
    (0x1305, "Pack overvoltage protection", "voltage", "V", 100, 0, False, 0, 1000),
    (0x1306, "Pack low-voltage recovery", "voltage", "V", 100, 0, False, 0, 1000),
    (0x1307, "Pack low-voltage alarm", "voltage", "V", 100, 0, False, 0, 1000),
    (0x1308, "Pack undervoltage recovery", "voltage", "V", 100, 0, False, 0, 1000),
    (0x1309, "Pack undervoltage protection", "voltage", "V", 100, 0, False, 0, 1000),
    (0x130A, "Cell high-voltage recovery", "voltage", "V", 1000, 0, False, 0, 6),
    (0x130B, "Cell high-voltage alarm", "voltage", "V", 1000, 0, False, 0, 6),
    (0x130C, "Cell overvoltage recovery", "voltage", "V", 1000, 0, False, 0, 6),
    (0x130D, "Cell overvoltage protection", "voltage", "V", 1000, 0, False, 0, 6),
    (0x130E, "Cell low-voltage recovery", "voltage", "V", 1000, 0, False, 0, 6),
    (0x130F, "Cell low-voltage alarm", "voltage", "V", 1000, 0, False, 0, 6),
    (0x1310, "Cell undervoltage recovery", "voltage", "V", 1000, 0, False, 0, 6),
    (0x1311, "Cell undervoltage protection", "voltage", "V", 1000, 0, False, 0, 6),
    (0x1312, "Cell undervoltage fault", "voltage", "V", 1000, 0, False, 0, 6),
    (0x1313, "Cell voltage difference protection", "voltage", "V", 1000, 0, False, 0, 6),
    (0x1314, "Secondary charge current protection", "current", "A", 1, 0, True, -1000, 1000),
    (0x1315, "Charge high-current recovery", "current", "A", 1, 0, False, 0, 1000),
    (0x1316, "Charge high-current alarm", "current", "A", 1, 0, True, -1000, 1000),
    (0x1317, "Charge overcurrent protection", "current", "A", 1, 0, False, 0, 1000),
    (0x1318, "Charge overcurrent delay", "current", "s", 10, 0, True, 0, 6000),
    (0x1319, "Second charge overcurrent protection", "current", "A", 1, 0, True, -1000, 1000),
    (0x131A, "Second charge overcurrent delay", "current", "ms", 1, 0, True, 0, 32767),
    (0x131B, "Discharge low-current recovery", "current", "A", 1, 0, True, -1000, 1000),
    (0x131C, "Discharge low-current alarm", "current", "A", 1, 0, True, -1000, 1000),
    (0x131D, "Discharge overcurrent protection", "current", "A", 1, 0, True, -1000, 1000),
    (0x131E, "Discharge overcurrent delay", "current", "s", 10, 0, True, 0, 6000),
    (0x131F, "Second discharge current protection", "current", "A", 1, 0, False, 0, 1000),
    (0x1320, "Second discharge current delay", "current", "ms", 1, 0, True, 0, 32767),
    (0x1321, "Output short-circuit protection", "current", "A", 1, 0, False, 0, 1000),
    (0x1322, "Output short-circuit delay", "current", "µs", 1, 0, False, 0, 65535),
    (0x1323, "Overcurrent recovery delay", "current", "s", 10, 0, False, 0, 6000),
    (0x1324, "Overcurrent lock count", "current", "times", 1, 0, True, 0, 32767),
    (0x1325, "Charge high-switch limit time", "timing", "s", 10, 0, False, 0, 6000),
    (0x1326, "Pulse current", "current", "A", 1, 0, False, 0, 1000),
    (0x1327, "Pulse time", "timing", "s", 10, 0, False, 0, 6000),
    (0x132B, "Precharge short threshold", "precharge", "%", 10, 0, False, 0, 100),
    (0x132C, "Precharge stop threshold", "precharge", "%", 10, 0, False, 0, 100),
    (0x132D, "Precharge fault threshold", "precharge", "%", 10, 0, False, 0, 100),
    (0x132E, "Precharge timeout", "precharge", "s", 1, 0, False, 0, 65535),
)

_SEPLOS_TEMPERATURE_ROWS: Final[tuple[tuple[int, str], ...]] = (
    (0x132F, "Charge high-temperature recovery"), (0x1330, "Charge high-temperature alarm"),
    (0x1331, "Charge overtemperature recovery"), (0x1332, "Charge overtemperature protection"),
    (0x1333, "Charge low-temperature recovery"), (0x1334, "Charge low-temperature alarm"),
    (0x1335, "Charge undertemperature recovery"), (0x1336, "Charge undertemperature protection"),
    (0x1337, "Discharge high-temperature recovery"), (0x1338, "Discharge high-temperature alarm"),
    (0x1339, "Discharge overtemperature recovery"), (0x133A, "Discharge overtemperature protection"),
    (0x133B, "Discharge low-temperature recovery"), (0x133C, "Discharge low-temperature alarm"),
    (0x133D, "Discharge undertemperature recovery"), (0x133E, "Discharge undertemperature protection"),
    (0x133F, "Environment high-temperature recovery"), (0x1340, "Environment high-temperature alarm"),
    (0x1341, "Environment overtemperature recovery"), (0x1342, "Environment overtemperature protection"),
    (0x1343, "Environment low-temperature recovery"), (0x1344, "Environment low-temperature alarm"),
    (0x1345, "Environment undertemperature recovery"), (0x1346, "Environment undertemperature protection"),
    (0x1347, "Power high-temperature recovery"), (0x1348, "Power high-temperature alarm"),
    (0x1349, "Power overtemperature recovery"), (0x134A, "Power overtemperature protection"),
    (0x134B, "Cell heating stop temperature"), (0x134C, "Cell heating start temperature"),
    (0x134D, "Balancing high-temperature inhibit"), (0x134E, "Balancing low-temperature inhibit"),
)

_SEPLOS_TAIL_ROWS: Final[tuple[tuple[int, str, str, str, float, float, bool, float | None, float | None], ...]] = (
    (0x134F, "Static balancing time", "balancing", "h", 1, 0, False, 0, 65535),
    (0x1350, "Balancing start voltage", "balancing", "V", 1000, 0, False, 0, 6),
    (0x1351, "Balancing start difference", "balancing", "V", 1000, 0, False, 0, 6),
    (0x1352, "Balancing stop difference", "balancing", "V", 1000, 0, False, 0, 6),
    (0x1353, "Full SOC release", "capacity", "%", 10, 0, False, 0, 100),
    (0x1354, "Low SOC recovery", "capacity", "%", 10, 0, False, 0, 100),
    (0x1355, "Low SOC alarm", "capacity", "%", 10, 0, False, 0, 100),
    (0x1356, "Under-SOC recovery", "capacity", "%", 10, 0, False, 0, 100),
    (0x1357, "Under-SOC protection", "capacity", "%", 10, 0, False, 0, 100),
    (0x1358, "Rated capacity", "capacity", "Ah", 100, 0, False, 0, 655.35),
    (0x1359, "Total capacity", "capacity", "Ah", 100, 0, False, 0, 655.35),
    (0x135A, "Remaining capacity", "capacity", "Ah", 100, 0, False, 0, 655.35),
    (0x135B, "Standby-to-sleep time", "timing", "h", 1, 0, False, 0, 65535),
    (0x135C, "FOCS output delay", "timing", "s", 10, 0, False, 0, 6000),
    (0x135D, "FOCS output interval", "timing", "min", 1, 0, False, 0, 65535),
    (0x135E, "PCS output count", "timing", "times", 1, 0, False, 0, 65535),
    (0x135F, "Compensation position 1", "other", "cell", 1, 0, False, 0, 32),
    (0x1360, "Position 1 resistance", "other", "mΩ", 1, 0, False, 0, 65535),
    (0x1361, "Compensation position 2", "other", "cell", 1, 0, False, 0, 32),
    (0x1362, "Position 2 resistance", "other", "mΩ", 1, 0, False, 0, 65535),
    (0x1363, "Cell difference alarm", "voltage", "mV", 1, 0, False, 0, 65535),
    (0x1364, "Cell difference alarm recovery", "voltage", "mV", 1, 0, False, 0, 65535),
    (0x1365, "PCS requested charge voltage", "inverter", "V", 100, 0, False, 0, 1000),
    (0x1366, "PCS requested charge current", "inverter", "A", 1, 0, False, 0, 1000),
    (0x1367, "PCS requested discharge current", "inverter", "A", 1, 0, True, -1000, 1000),
)

SEPLOS_REGISTER_SPECS: Final[tuple[SettingSpec, ...]] = tuple(
    _seplos_spec(f"reg_{address:04x}", label, group, address, unit, factor, offset, signed, minimum, maximum)
    for address, label, group, unit, factor, offset, signed, minimum, maximum in (*_SEPLOS_ROWS, *_SEPLOS_TAIL_ROWS)
) + tuple(
    _seplos_spec(f"reg_{address:04x}", label, "temperature", address, "°C", 10, 2731.5, False, -100, 200)
    for address, label in _SEPLOS_TEMPERATURE_ROWS
)

_SFA_GROUPS: Final[tuple[tuple[str, tuple[str | None, ...]], ...]] = (
    ("Voltage", ("Cell high-voltage alarm", "Cell overvoltage protection", "Cell low-voltage alarm", "Cell undervoltage protection", "Pack high-voltage alarm", "Pack overvoltage protection", "Pack low-voltage alarm", "Pack undervoltage protection")),
    ("Cell temperature", ("Charge high-temperature alarm", "Charge overtemperature protection", "Charge low-temperature alarm", "Charge undertemperature protection", "Discharge high-temperature alarm", "Discharge overtemperature protection", "Discharge low-temperature alarm", "Discharge undertemperature protection")),
    ("Environment and power", ("Environment high-temperature alarm", "Environment overtemperature protection", "Environment low-temperature alarm", "Environment undertemperature protection", "Power high-temperature alarm", "Power overtemperature protection", "Low-temperature cell heating", "Cell-voltage fault")),
    ("General", ("FOCS output", None, None, None, None, None, None, None)),
    ("Current protection 1", ("Charge-current alarm", "Charge overcurrent protection", "Secondary charge overcurrent protection", "Discharge-current alarm", "Discharge overcurrent protection", "Secondary discharge overcurrent protection", "Output short-circuit protection", None)),
    ("Current protection 2", ("Output-short latch", None, "Secondary charge-current latch", "Secondary discharge-current latch", None, None, None, None)),
    ("Capacity", ("Low-SOC alarm", "Intermittent charge", "External switch control", "Standby/sleep mode", "History recording", "Under-SOC protection", "Active current limiting", "Passive current limiting")),
    ("Balancing", ("Balancing module", "Static balancing indication", "Static balancing overtime", "Balancing temperature limit", None, None, None, None)),
    ("Indicators", ("Buzzer", "LCD display", "Manual FOCS output", "Automatic FOCS output", "Undervoltage recovery", "Aerosol test", "Aerosol normally-disconnected mode", "Temperature/current adjustment")),
    ("Hard faults", ("NTC fault", "AFE fault", "Charge MOSFET fault", "Discharge MOSFET fault", "Cell fault", "Break-line fault", "Key fault", "Aerosol alarm")),
)

SEPLOS_COIL_SPECS: Final[tuple[SettingSpec, ...]] = tuple(
    SettingSpec(
        f"coil_{0x1400 + byte_index * 8 + bit:04x}",
        f"{group_name}: {label}",
        "switches",
        0x1400 + byte_index * 8 + bit,
        kind="bool",
        source="coil",
        bit=bit,
    )
    for byte_index, (group_name, labels) in enumerate(_SFA_GROUPS)
    for bit, label in enumerate(labels)
    if label is not None
)


def _build_modbus_write_coil(device: int, register: int, state: bool) -> bytes:
    frame = bytearray((device, 0x0F, register >> 8, register & 0xFF, 0, 1, 1, 1 if state else 0))
    frame.extend(bms_ble.crc16_modbus(frame).to_bytes(2, "little"))
    return bytes(frame)


async def _seplos_read_as(session: bms_ble._SeplosSession, device: int, function: int, register: int, count: int) -> bytes:
    session.buffer.clear()
    session.frame_event.clear()
    await session.client.write_gatt_char(
        bms_ble.normalize_uuid("fff2"), bms_ble.build_modbus_read(device, function, register, count), response=False
    )
    frame = await session._response()
    return frame[3:-2]


async def _read_seplos_session(session: bms_ble._SeplosSession) -> dict[str, Any]:
    identity = bms_ble.parse_seplos_identity(await session.read(0x1700, 0x33))
    protocol = bms_ble.parse_seplos_protocol(await session.read(0x1800, 0x24))
    spa = await _seplos_read_as(session, 0, 0x04, 0x1300, 0x35)
    spa += await _seplos_read_as(session, 0, 0x04, 0x1335, 0x33)
    coils = await _seplos_read_as(session, 0, 0x01, 0x1400, 5)
    registers = {0x1300 + index: int.from_bytes(spa[index * 2 : index * 2 + 2], "big") for index in range(0x68)}
    raw_values = {spec.key: registers[spec.register] for spec in SEPLOS_REGISTER_SPECS}
    raw_values.update(
        {
            spec.key: 1 if coils[(spec.register - 0x1400) // 8] & (1 << ((spec.register - 0x1400) % 8)) else 0
            for spec in SEPLOS_COIL_SPECS
        }
    )
    groups = _groups((*SEPLOS_REGISTER_SPECS, *SEPLOS_COIL_SPECS), raw_values)
    profile_options = tuple((name, item["protocol_name"]) for name, item in bms_ble.SEPLOS_PROTOCOLS.items())
    protocol_spec = SettingSpec("inverter_protocol", "Inverter protocol", "inverter", 0x1800, kind="select", options=profile_options)
    protocol_item = _spec_item(protocol_spec, protocol["selector_raw"], protocol["selector_profile"] or "unknown")
    inverter_group = next((group for group in groups if group["key"] == "inverter"), None)
    if inverter_group is None:
        groups.append({"key": "inverter", "title": GROUP_TITLES["inverter"], "settings": [protocol_item]})
    else:
        inverter_group["settings"].insert(0, protocol_item)
    return {
        "device": "seplos",
        "protocol": "Seplos V3 Modbus BLE",
        "identity": {
            "model": identity["device"],
            "hardware": identity["factory"],
            "firmware": identity["firmware"],
            "serial": identity["bms_serial_number"],
            "pack_serial": identity["pack_serial_number"],
            "address": bms_ble.DEVICE_INVENTORY["seplos"]["address"],
            "confirmation": identity["bms_serial_number"],
        },
        "groups": groups,
        "raw": {
            "registers": {f"0x{address:04X}": value for address, value in registers.items()},
            "function_switches": coils.hex(),
            "protocol": protocol,
        },
    }


async def read_seplos_configuration() -> dict[str, Any]:
    BleakClient, scanner, _ = bms_ble._ble_imports()
    entry = bms_ble.DEVICE_INVENTORY["seplos"]
    device = await bms_ble._find_device(scanner, entry["address"])
    async with BleakClient(device, timeout=15) as client:
        session = bms_ble._SeplosSession(client)
        await client.start_notify(bms_ble.normalize_uuid("fff1"), session.notification)
        return await _read_seplos_session(session)


async def _write_seplos_coil(session: bms_ble._SeplosSession, register: int, state: bool) -> None:
    session.buffer.clear()
    session.frame_event.clear()
    await session.client.write_gatt_char(
        bms_ble.normalize_uuid("fff2"), _build_modbus_write_coil(0, register, state), response=False
    )
    frame = await session._response()
    if frame[1] != 0x0F or int.from_bytes(frame[2:4], "big") != register:
        raise RuntimeError("Seplos coil acknowledgement does not match the request")


async def write_seplos_setting(key: str, value: Any, confirmation: str) -> dict[str, Any]:
    if key == "inverter_protocol":
        if str(value) not in bms_ble.SEPLOS_PROTOCOLS:
            raise ValueError("unknown Seplos inverter protocol")
        result = await bms_ble.set_seplos_protocol(str(value), confirmation)
        after_configuration = await read_seplos_configuration()
        return {
            "written": bool(result["changed"]), "verified": True,
            "before": result["before"]["selector_profile"], "after": result["after"]["selector_profile"],
            "after_configuration": after_configuration,
        }
    specs = (*SEPLOS_REGISTER_SPECS, *SEPLOS_COIL_SPECS)
    spec = next((item for item in specs if item.key == key), None)
    if spec is None:
        raise ValueError("unknown Seplos setting")
    entry = bms_ble.DEVICE_INVENTORY["seplos"]
    if confirmation.upper() != entry["advertised_name"].upper():
        raise ValueError(f"confirmation must exactly match the Seplos serial: {entry['advertised_name']}")
    target_raw = _encode_number(spec, value)
    BleakClient, scanner, _ = bms_ble._ble_imports()
    device = await bms_ble._find_device(scanner, entry["address"])
    async with BleakClient(device, timeout=15) as client:
        session = bms_ble._SeplosSession(client)
        await client.start_notify(bms_ble.normalize_uuid("fff1"), session.notification)
        before_configuration = await _read_seplos_session(session)
        if before_configuration["identity"]["confirmation"].upper() != entry["advertised_name"].upper():
            raise RuntimeError("connected Seplos identity does not match the configured battery")
        before = next(item for group in before_configuration["groups"] for item in group["settings"] if item["key"] == key)
        if before["raw"] == target_raw:
            return {"written": False, "verified": True, "before": before["value"], "after": before["value"], "after_configuration": before_configuration}
        if spec.source == "coil":
            await _write_seplos_coil(session, spec.register, bool(target_raw))
        else:
            # This V3 Bluetooth bridge byte-swaps each UINT16 written through
            # function 0x10.  Supplying the swapped word preserves the value
            # observed through function 0x04 (verified on the live PCT block).
            swapped = ((target_raw & 0xFF) << 8) | (target_raw >> 8)
            await session.write(spec.register, swapped)
        await asyncio.sleep(0.5)
        after_configuration = await _read_seplos_session(session)
    after = next(item for group in after_configuration["groups"] for item in group["settings"] if item["key"] == key)
    if after["raw"] != target_raw:
        raise RuntimeError("Seplos did not report the requested value after the write")
    return {"written": True, "verified": True, "before": before["value"], "after": after["value"], "after_configuration": after_configuration}


_JK_NUMBER_BASE_SPECS: Final[tuple[SettingSpec, ...]] = (
    SettingSpec("smart_sleep_voltage", "Smart sleep voltage", "timing", 0x01, "V", 1000, minimum=0.003, maximum=3.65, step=0.001, frame_offset=6, length=1),
    SettingSpec("cell_undervoltage_protection", "Cell undervoltage protection", "voltage", 0x02, "V", 1000, minimum=1.2, maximum=4.35, step=0.001, frame_offset=10),
    SettingSpec("cell_undervoltage_recovery", "Cell undervoltage recovery", "voltage", 0x03, "V", 1000, minimum=1.2, maximum=4.35, step=0.001, frame_offset=14),
    SettingSpec("cell_overvoltage_protection", "Cell overvoltage protection", "voltage", 0x04, "V", 1000, minimum=1.2, maximum=4.35, step=0.001, frame_offset=18),
    SettingSpec("cell_overvoltage_recovery", "Cell overvoltage recovery", "voltage", 0x05, "V", 1000, minimum=1.2, maximum=4.35, step=0.001, frame_offset=22),
    SettingSpec("balance_trigger_voltage", "Balance trigger difference", "balancing", 0x06, "V", 1000, minimum=0.003, maximum=1, step=0.001, frame_offset=26),
    SettingSpec("cell_soc100_voltage", "Cell voltage at 100% SOC", "capacity", 0x07, "V", 1000, minimum=0.003, maximum=4.35, step=0.001, frame_offset=30),
    SettingSpec("cell_soc0_voltage", "Cell voltage at 0% SOC", "capacity", 0x08, "V", 1000, minimum=0.003, maximum=4.35, step=0.001, frame_offset=34),
    SettingSpec("requested_charge_voltage", "Requested charge voltage per cell", "inverter", 0x09, "V", 1000, minimum=0.003, maximum=4.35, step=0.001, frame_offset=38),
    SettingSpec("requested_float_voltage", "Requested float voltage per cell", "inverter", 0x0A, "V", 1000, minimum=0.003, maximum=4.35, step=0.001, frame_offset=42),
    SettingSpec("power_off_voltage", "Power-off voltage per cell", "voltage", 0x0B, "V", 1000, minimum=1.2, maximum=4.35, step=0.001, frame_offset=46),
    SettingSpec("max_charge_current", "Maximum charge current", "current", 0x0C, "A", 1000, minimum=1, maximum=600.1, step=0.1, frame_offset=50),
    SettingSpec("charge_overcurrent_delay", "Charge overcurrent delay", "current", 0x0D, "s", minimum=2, maximum=600, step=1, frame_offset=54),
    SettingSpec("charge_overcurrent_recovery", "Charge overcurrent recovery", "current", 0x0E, "s", minimum=2, maximum=600, step=1, frame_offset=58),
    SettingSpec("max_discharge_current", "Maximum discharge current", "current", 0x0F, "A", 1000, minimum=1, maximum=1200.1, step=0.1, frame_offset=62),
    SettingSpec("discharge_overcurrent_delay", "Discharge overcurrent delay", "current", 0x10, "s", minimum=2, maximum=600, step=1, frame_offset=66),
    SettingSpec("discharge_overcurrent_recovery", "Discharge overcurrent recovery", "current", 0x11, "s", minimum=2, maximum=600, step=1, frame_offset=70),
    SettingSpec("short_circuit_recovery", "Short-circuit recovery", "current", 0x12, "s", minimum=2, maximum=600, step=1, frame_offset=74),
    SettingSpec("max_balance_current", "Maximum balance current", "balancing", 0x13, "A", 1000, minimum=0.1, maximum=15, step=0.1, frame_offset=78),
    SettingSpec("charge_overtemperature", "Charge overtemperature protection", "temperature", 0x14, "°C", 10, minimum=30, maximum=80, step=0.1, frame_offset=82),
    SettingSpec("charge_overtemperature_recovery", "Charge overtemperature recovery", "temperature", 0x15, "°C", 10, minimum=20, maximum=80, step=0.1, frame_offset=86),
    SettingSpec("discharge_overtemperature", "Discharge overtemperature protection", "temperature", 0x16, "°C", 10, minimum=30, maximum=80, step=0.1, frame_offset=90),
    SettingSpec("discharge_overtemperature_recovery", "Discharge overtemperature recovery", "temperature", 0x17, "°C", 10, minimum=20, maximum=80, step=0.1, frame_offset=94),
    SettingSpec("charge_undertemperature", "Charge undertemperature protection", "temperature", 0x18, "°C", 10, signed=True, minimum=-45, maximum=20, step=0.1, frame_offset=98),
    SettingSpec("charge_undertemperature_recovery", "Charge undertemperature recovery", "temperature", 0x19, "°C", 10, signed=True, minimum=-45, maximum=20, step=0.1, frame_offset=102),
    SettingSpec("mosfet_overtemperature", "MOSFET overtemperature protection", "temperature", 0x1A, "°C", 10, signed=True, minimum=50, maximum=110, step=0.1, frame_offset=106),
    SettingSpec("mosfet_overtemperature_recovery", "MOSFET overtemperature recovery", "temperature", 0x1B, "°C", 10, signed=True, minimum=40, maximum=110, step=0.1, frame_offset=110),
    SettingSpec("cell_count", "Cell count", "battery", 0x1C, minimum=2, maximum=32, step=1, frame_offset=114),
    SettingSpec("charging", "Charge MOSFET", "switches", 0x1D, kind="bool", frame_offset=118),
    SettingSpec("discharging", "Discharge MOSFET", "switches", 0x1E, kind="bool", frame_offset=122),
    SettingSpec("balancer", "Balancer", "switches", 0x1F, kind="bool", frame_offset=126),
    SettingSpec("total_capacity", "Total battery capacity", "battery", 0x20, "Ah", 1000, minimum=2, maximum=20000, step=1, frame_offset=130),
    SettingSpec("short_circuit_delay", "Short-circuit delay", "current", 0x21, "µs", minimum=0, maximum=1000000, step=1, frame_offset=134),
    SettingSpec("balancing_start_voltage", "Balancing start voltage", "balancing", 0x22, "V", 1000, minimum=1.2, maximum=4.35, step=0.001, frame_offset=138),
    SettingSpec("discharge_precharge_time", "Discharge precharge time", "precharge", 0x25, "s", minimum=0, maximum=255, step=1, frame_offset=274),
    SettingSpec("heating_start_temperature", "Heating start temperature", "temperature", 0x37, "°C", signed=True, minimum=-40, maximum=100, step=1, frame_offset=284, length=1),
    SettingSpec("heating_stop_temperature", "Heating stop temperature", "temperature", 0x38, "°C", signed=True, minimum=-40, maximum=100, step=1, frame_offset=285, length=1),
    SettingSpec("smart_sleep_delay", "Smart sleep delay", "timing", 0x39, "h", minimum=0, maximum=255, step=1, frame_offset=286, length=1),
    SettingSpec("discharge_undertemperature", "Discharge undertemperature protection", "temperature", 0x3A, "°C", signed=True, minimum=-40, maximum=100, step=1, frame_offset=296, length=1),
    SettingSpec("discharge_undertemperature_recovery", "Discharge undertemperature recovery", "temperature", 0x3B, "°C", signed=True, minimum=-40, maximum=100, step=1, frame_offset=297, length=1),
)

# JK's command-length byte and the width of a value in the settings frame are
# separate concepts.  The v19 frame stores the main block as UINT32 values;
# only the compact tail fields are one byte wide.
_JK_BYTE_SETTING_OFFSETS: Final[frozenset[int]] = frozenset((284, 285, 286, 296, 297))
JK_NUMBER_SPECS: Final[tuple[SettingSpec, ...]] = tuple(
    replace(
        spec,
        source="jk_setting",
        bits=8 if spec.frame_offset in _JK_BYTE_SETTING_OFFSETS else 32,
    )
    for spec in _JK_NUMBER_BASE_SPECS
)

_JK_EXTENDED_SWITCHES: Final[tuple[tuple[str, str, int, int, int], ...]] = (
    ("heating", "Heating", 0x27, 282, 0),
    ("disable_temperature_sensors", "Disable temperature sensors", 0x28, 282, 1),
    ("display_always_on", "Display always on", 0x2B, 282, 4),
    ("smart_sleep", "Smart sleep", 0x2D, 282, 6),
    ("disable_pcl_module", "Disable PCL module", 0x2E, 282, 7),
    ("timed_stored_data", "Timed data storage", 0x2F, 283, 0),
    ("charging_float_mode", "Charging float mode", 0x30, 283, 1),
    ("emergency_button_trigger", "Emergency button trigger", 0x31, 283, 2),
    ("dry_contact_alarm_intermittent", "Intermittent dry-contact alarm", 0x32, 283, 3),
    ("discharge_ocp_2", "Discharge overcurrent protection II", 0x33, 283, 4),
    ("discharge_ocp_3", "Discharge overcurrent protection III", 0x34, 283, 5),
    ("gps_locked_charging", "GPS-locked charging", 0x35, 283, 6),
    ("gps_locked_discharging", "GPS-locked discharging", 0x36, 283, 7),
)

JK_SWITCH_SPECS: Final[tuple[SettingSpec, ...]] = tuple(
    SettingSpec(key, label, "switches", register, kind="bool", source="jk_bit", frame_offset=offset, bit=bit)
    for key, label, register, offset, bit in _JK_EXTENDED_SWITCHES
)

_TRIGGER_OPTIONS: Final[tuple[tuple[int, str], ...]] = tuple(enumerate((
    "Off", "Low SOC", "Pack overvoltage", "Pack undervoltage", "Cell overvoltage", "Cell undervoltage",
    "Charge overcurrent", "Discharge overcurrent", "Pack overtemperature", "MOSFET overtemperature",
    "System alarm", "Pack low temperature", "Remote control", "Above SOC", "MOSFET abnormal",
)))


async def _read_jk_frames(bms: Any) -> tuple[bytes, bytes]:
    from aiobmsble.bms.jikong_bms import BMS as JkBMS

    bms._valid_reply = 0x03
    try:
        await bms._await_msg(JkBMS._cmd(0x97), char=bms._char_write_handle)
        info = bytes(bms._msg)
        bms._valid_reply = 0x01
        await bms._await_msg(JkBMS._cmd(0x96), char=bms._char_write_handle)
        settings = bytes(bms._msg)
    finally:
        bms._valid_reply = 0x02
    return info, settings


def _jk_configuration(info: bytes, settings: bytes) -> dict[str, Any]:
    parsed = bms_ble.parse_jk_device_info(info)
    raw_values: dict[str, int] = {}
    for spec in JK_NUMBER_SPECS:
        assert spec.frame_offset is not None
        size = spec.bits // 8
        raw_values[spec.key] = int.from_bytes(settings[spec.frame_offset : spec.frame_offset + size], "little")
    for spec in JK_SWITCH_SPECS:
        assert spec.frame_offset is not None and spec.bit is not None
        raw_values[spec.key] = 1 if settings[spec.frame_offset] & (1 << spec.bit) else 0
    groups = _groups((*JK_NUMBER_SPECS, *JK_SWITCH_SPECS), raw_values)

    communication: list[dict[str, Any]] = []
    for interface, definition in bms_ble.JK_PROTOCOL_INTERFACES.items():
        selected = parsed["interfaces"][interface]["selected"]["index"]
        options = tuple((item["index"], item["name"]) for item in parsed["interfaces"][interface]["enabled_protocols"])
        spec = SettingSpec(f"protocol_{interface}", f"{interface.upper()} protocol", "inverter", definition["register"], kind="select", options=options, length=2, source="jk_info", frame_offset=definition["offset"])
        communication.append(_spec_item(spec, selected, selected))
    for key, label, register, offset in (
        ("lcd_buzzer_trigger", "LCD buzzer trigger", 0xA9, 234),
        ("dry1_trigger", "Dry contact 1 trigger", 0xAA, 235),
        ("dry2_trigger", "Dry contact 2 trigger", 0xAB, 236),
    ):
        spec = SettingSpec(key, label, "inverter", register, kind="select", options=_TRIGGER_OPTIONS, length=2, source="jk_info", frame_offset=offset)
        communication.append(_spec_item(spec, info[offset], info[offset]))
    communication.extend(
        (
            _spec_item(SettingSpec("multiplexed_port_mode", "Multiplexed port mode", "inverter", 0x2A, kind="select", options=((0, "CAN"), (1, "RS485")), source="jk_mux", length=4), 1 if settings[282] & 0x08 else 0, 1 if settings[282] & 0x08 else 0),
            _spec_item(SettingSpec("requested_charge_voltage_time", "Requested charge voltage time", "inverter", 0xB3, "h", 10, kind="number", minimum=0, maximum=18.2, step=0.1, source="jk_info", frame_offset=266, length=1), info[266], round(info[266] / 10, 1)),
            _spec_item(SettingSpec("requested_float_voltage_time", "Requested float voltage time", "inverter", 0xB4, "h", 10, kind="number", minimum=0, maximum=18.2, step=0.1, source="jk_info", frame_offset=267, length=1), info[267], round(info[267] / 10, 1)),
            _spec_item(SettingSpec("emergency_duration", "Emergency duration", "switches", 0xB5, "min", minimum=0, maximum=255, step=1, source="jk_info", frame_offset=269, length=1), info[269], info[269]),
            _spec_item(SettingSpec("re_bulk_soc", "Re-bulk SOC", "capacity", 0xB7, "%", minimum=0, maximum=50, step=1, source="jk_info", frame_offset=278, length=1), info[278], info[278]),
        )
    )
    inverter_group = next(group for group in groups if group["key"] == "inverter")
    inverter_group["settings"].extend(communication)
    return {
        "device": "jk",
        "protocol": "JK02 32S BLE",
        "identity": {
            "model": parsed["model"], "hardware": parsed["hardware_version"], "firmware": parsed["software_version"],
            "name": parsed["name"], "serial": parsed["serial_number"],
            "address": bms_ble.DEVICE_INVENTORY["jk"]["address"], "confirmation": parsed["serial_number"],
        },
        "groups": groups,
        "raw": {"settings_frame": settings.hex(), "device_info_frame": info.hex()},
    }


async def read_jk_configuration() -> dict[str, Any]:
    from aiobmsble.bms.jikong_bms import BMS as JkBMS

    _, scanner, _ = bms_ble._ble_imports()
    entry = bms_ble.DEVICE_INVENTORY["jk"]
    device = await bms_ble._find_device(scanner, entry["address"])
    async with JkBMS(device) as bms:
        info, settings = await _read_jk_frames(bms)
    return _jk_configuration(info, settings)


def _find_jk_spec(key: str, info: bytes, settings: bytes) -> SettingSpec:
    spec = next((item for item in (*JK_NUMBER_SPECS, *JK_SWITCH_SPECS) if item.key == key), None)
    if spec is not None:
        return spec
    if key.startswith("protocol_"):
        interface = key.removeprefix("protocol_")
        if interface not in bms_ble.JK_PROTOCOL_INTERFACES:
            raise ValueError("unknown JK interface")
        definition = bms_ble.JK_PROTOCOL_INTERFACES[interface]
        parsed = bms_ble.parse_jk_device_info(info)["interfaces"][interface]
        options = tuple((item["index"], item["name"]) for item in parsed["enabled_protocols"])
        return SettingSpec(key, f"{interface.upper()} protocol", "inverter", definition["register"], kind="select", options=options, source="jk_info", frame_offset=definition["offset"], length=2)
    mapping = {
        "lcd_buzzer_trigger": ("LCD buzzer trigger", 0xA9, 234, _TRIGGER_OPTIONS, 2),
        "dry1_trigger": ("Dry contact 1 trigger", 0xAA, 235, _TRIGGER_OPTIONS, 2),
        "dry2_trigger": ("Dry contact 2 trigger", 0xAB, 236, _TRIGGER_OPTIONS, 2),
        "multiplexed_port_mode": ("Multiplexed port mode", 0x2A, None, ((0, "CAN"), (1, "RS485")), 4),
    }
    if key in mapping:
        label, register, offset, options, length = mapping[key]
        return SettingSpec(key, label, "inverter", register, kind="select", options=options, source="jk_mux" if offset is None else "jk_info", frame_offset=offset, length=length)
    for item in (
        SettingSpec("requested_charge_voltage_time", "Requested charge voltage time", "inverter", 0xB3, "h", 10, minimum=0, maximum=18.2, step=0.1, source="jk_info", frame_offset=266, length=1),
        SettingSpec("requested_float_voltage_time", "Requested float voltage time", "inverter", 0xB4, "h", 10, minimum=0, maximum=18.2, step=0.1, source="jk_info", frame_offset=267, length=1),
        SettingSpec("emergency_duration", "Emergency duration", "switches", 0xB5, "min", minimum=0, maximum=255, step=1, source="jk_info", frame_offset=269, length=1),
        SettingSpec("re_bulk_soc", "Re-bulk SOC", "capacity", 0xB7, "%", minimum=0, maximum=50, step=1, source="jk_info", frame_offset=278, length=1),
    ):
        if item.key == key:
            return item
    raise ValueError("unknown JK setting")


async def write_jk_setting(key: str, value: Any, confirmation: str) -> dict[str, Any]:
    from aiobmsble.bms.jikong_bms import BMS as JkBMS

    _, scanner, _ = bms_ble._ble_imports()
    entry = bms_ble.DEVICE_INVENTORY["jk"]
    device = await bms_ble._find_device(scanner, entry["address"])
    async with JkBMS(device) as bms:
        info, settings = await _read_jk_frames(bms)
        before_configuration = _jk_configuration(info, settings)
        serial = before_configuration["identity"]["serial"]
        if confirmation.upper() != serial.upper():
            raise ValueError(f"confirmation must exactly match the JK serial number: {serial}")
        spec = _find_jk_spec(key, info, settings)
        before = next(item for group in before_configuration["groups"] for item in group["settings"] if item["key"] == key)
        if spec.options:
            allowed = {str(option[0]) for option in spec.options}
            if str(value) not in allowed:
                raise ValueError(f"{spec.label} must be one of the supported options")
            target_raw = int(value)
        else:
            target_raw = _encode_number(spec, value)
        if before["raw"] == target_raw:
            return {"written": False, "verified": True, "before": before["value"], "after": before["value"], "after_configuration": before_configuration}
        await bms._client.write_gatt_char(
            bms._char_write_handle, bms_ble.build_jk_frame(spec.register, target_raw, spec.length), response=False
        )
        await asyncio.sleep(0.8)
        after_info, after_settings = await _read_jk_frames(bms)
        after_configuration = _jk_configuration(after_info, after_settings)
    after = next(item for group in after_configuration["groups"] for item in group["settings"] if item["key"] == key)
    if after["raw"] != target_raw:
        raise RuntimeError("JK did not report the requested value after the write")
    return {"written": True, "verified": True, "before": before["value"], "after": after["value"], "after_configuration": after_configuration}


async def read_bms_configuration(alias: str) -> dict[str, Any]:
    readers = {"daly": read_daly_configuration, "seplos": read_seplos_configuration, "jk": read_jk_configuration}
    try:
        reader = readers[alias]
    except KeyError as exc:
        raise ValueError("unknown BMS") from exc
    return await reader()


async def write_bms_setting(alias: str, key: str, value: Any, confirmation: str) -> dict[str, Any]:
    writers = {"daly": write_daly_setting, "seplos": write_seplos_setting, "jk": write_jk_setting}
    try:
        writer = writers[alias]
    except KeyError as exc:
        raise ValueError("unknown BMS") from exc
    return await writer(key, value, confirmation)
