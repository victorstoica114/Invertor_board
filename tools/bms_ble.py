#!/usr/bin/env python3
"""Read and safely configure the BMS devices found near this Raspberry Pi.

Telemetry uses aiobmsble.  The protocol inspection/configuration code is kept
here because aiobmsble intentionally exposes read-only battery telemetry.
"""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import asdict, is_dataclass
import json
import sys
from typing import Any, Final


DEVICE_INVENTORY: Final[dict[str, dict[str, str]]] = {
    "daly": {
        "address": "D0:18:05:01:4B:F9",
        "advertised_name": "DL-Dali Cristi",
        "driver": "daly_bms",
    },
    "seplos": {
        "address": "C0:D6:3C:55:21:C6",
        "advertised_name": "SP144B-C2506260009",
        "driver": "seplos_bms",
    },
    "jk": {
        "address": "C8:47:80:45:18:0E",
        "advertised_name": "508279C28000335",
        "driver": "jikong_bms",
    },
}

UART_PROTOCOLS: Final[tuple[str, ...]] = (
    "4G-GPS Remote module Common protocol V4.2",
    "JK BMS RS485 Modbus V1.0",
    "NIU U SERIES",
    "China tower shared battery cabinet V1.1",
    "PACE_RS485_Modbus_V1.3",
    "PYLON_low_voltage_Protocol_RS485_V3.5",
    "Growatt_BMS_RS485_Protocol_1xSxxP_ESS_Rev2.01",
    "Voltronic_Inverter_and_BMS_485_communication_protocol_20200325",
    "China tower shared battery cabinet V2.0",
    "WOW_RS485_Modbus_V1.3",
    "JK BMS LCD Protocol V2.0",
    "UART1 User customization",
    "UART2 User customization",
    "(9600) JK BMS RS485 Modbus V1.0",
    "(9600) PYLON_low_voltage_Protocol_RS485_V3.5",
    "JK BMS PBxx SERIES LCD Protocol V1.0",
    "JK BMS LIN BUS V1.0",
    "RS485 Protocol 17",
    "RS485 Protocol 18",
    "RS485 Protocol 19",
    "RS485 Protocol 20",
)

CAN_PROTOCOLS: Final[tuple[str, ...]] = (
    "JK BMS CAN Protocol (250K) V2.0",
    "Deye Low-voltage hybrid inverter CAN communication protocol V1.0",
    "PYLON-Low-voltage-V1.2",
    "Growatt BMS CAN-Bus-protocol-low-voltage_Rev_05",
    "Victron_CANbus_BMS_protocol_20170717",
    "MEGAREVO_Hybird_BMSCAN_Protocol_V1.0",
    "JK BMS CAN Protocol (500K) V2.0",
    "INVT BMS CAN Bus protocol V1.02",
    "GoodWe LV BMS Protocol (EX/EM/S-BP/BP)",
    "FSS-ConnectingBat-TI-en-10 Version 1.0",
    "MUST PV1800F-CAN communication Protocol1.04.04",
    "LuxpowerTek Battery CAN protocol V01",
    "CAN BUS User customization 1",
    "CAN BUS User customization 2",
)

JK_PROTOCOL_INTERFACES: Final[dict[str, dict[str, Any]]] = {
    "uart1": {"register": 0xA5, "offset": 184, "mask_offset": 186, "options": UART_PROTOCOLS},
    "can": {"register": 0xA6, "offset": 185, "mask_offset": 202, "options": CAN_PROTOCOLS},
    "uart2": {"register": 0xA8, "offset": 218, "mask_offset": 219, "options": UART_PROTOCOLS},
    "uart3": {"register": 0xB6, "offset": 270, "mask_offset": 271, "options": UART_PROTOCOLS},
}

# Protocol indexes exposed by the Seplos V3 inverter matching table.  On the
# wire the index occupies the high byte of register 0x1800/0x1823.  Keep the
# writable allow-list deliberately small: these are the three profiles verified
# against this SG16S200A-SP144B-C and the published Seplos compatibility table.
SEPLOS_PROTOCOLS: Final[dict[str, dict[str, Any]]] = {
    "srne_485": {
        "index": 9,
        "selector": 0x0900,
        "protocol_name": "PACE BMS Modbus Protocol for RS485",
        "baud_rate": 9600,
    },
    "growatt_485": {
        "index": 10,
        "selector": 0x0A00,
        "protocol_name": "Growatt BMS RS485 Protocol",
        "baud_rate": 9600,
    },
    "pylon_485": {
        "index": 11,
        "selector": 0x0B00,
        "protocol_name": "Pylon low voltage protocol",
        "baud_rate": 9600,
    },
}

BLE_BASE_UUID: Final[str] = "0000{}-0000-1000-8000-00805f9b34fb"
JK_INFO_FRAME_SIZE: Final[int] = 300


def normalize_uuid(short_uuid: str) -> str:
    """Return the standard Bluetooth base UUID for a 16-bit UUID."""
    return BLE_BASE_UUID.format(short_uuid.lower())


def crc16_modbus(data: bytes | bytearray) -> int:
    """Return the Modbus CRC16 used by the Daly and Seplos BLE transports."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_modbus_read(device: int, function: int, register: int, count: int) -> bytes:
    """Build a Seplos V3 read frame."""
    if not 0 <= device <= 0xFF or function not in (0x01, 0x04):
        raise ValueError("invalid Seplos device/function")
    if not 0 <= register <= 0xFFFF or not 1 <= count <= 0xFFFF:
        raise ValueError("invalid Seplos register/count")
    wire_count = count * (0x10 if function == 0x01 else 1)
    frame = bytearray((device, function, register >> 8, register & 0xFF, wire_count >> 8, wire_count & 0xFF))
    frame.extend(crc16_modbus(frame).to_bytes(2, "little"))
    return bytes(frame)


def build_modbus_write(device: int, register: int, values: tuple[int, ...]) -> bytes:
    """Build a Seplos V3 function-0x10 register write frame."""
    if not 0 <= device <= 0xFF or not 0 <= register <= 0xFFFF:
        raise ValueError("invalid Seplos device/register")
    if not values or len(values) > 0x7B or any(not 0 <= value <= 0xFFFF for value in values):
        raise ValueError("invalid Seplos register values")
    frame = bytearray(
        (
            device,
            0x10,
            register >> 8,
            register & 0xFF,
            len(values) >> 8,
            len(values) & 0xFF,
            len(values) * 2,
        )
    )
    for value in values:
        frame.extend(value.to_bytes(2, "big"))
    frame.extend(crc16_modbus(frame).to_bytes(2, "little"))
    return bytes(frame)


def build_jk_frame(register: int, value: int = 0, length: int = 0) -> bytes:
    """Build the 20-byte JK BLE command frame."""
    if not 0 <= register <= 0xFF or not 0 <= value <= 0xFFFFFFFF or not 0 <= length <= 4:
        raise ValueError("invalid JK register/value/length")
    frame = bytearray(20)
    frame[0:4] = b"\xAA\x55\x90\xEB"
    frame[4] = register
    frame[5] = length
    frame[6:10] = value.to_bytes(4, "little")
    frame[-1] = sum(frame[:-1]) & 0xFF
    return bytes(frame)


def _decode_ascii(data: bytes) -> str:
    return data.split(b"\0", 1)[0].decode("ascii", errors="replace").strip()


def _option(index: int, options: tuple[str, ...]) -> dict[str, Any]:
    return {"index": index, "name": options[index] if index < len(options) else "unknown"}


def parse_jk_device_info(frame: bytes) -> dict[str, Any]:
    """Decode identity and protocol selectors from a JK02 32S info frame."""
    if len(frame) < JK_INFO_FRAME_SIZE or frame[:5] != b"\x55\xAA\xEB\x90\x03":
        raise ValueError("not a complete JK device-info frame")
    if sum(frame[: JK_INFO_FRAME_SIZE - 1]) & 0xFF != frame[JK_INFO_FRAME_SIZE - 1]:
        raise ValueError("JK device-info checksum mismatch")

    result: dict[str, Any] = {
        "model": _decode_ascii(frame[6:22]),
        "hardware_version": _decode_ascii(frame[22:30]),
        "software_version": _decode_ascii(frame[30:38]),
        "name": _decode_ascii(frame[46:62]),
        "serial_number": _decode_ascii(frame[86:94]),
        "interfaces": {},
    }
    for interface, definition in JK_PROTOCOL_INTERFACES.items():
        index = frame[definition["offset"]]
        enabled_mask = int.from_bytes(frame[definition["mask_offset"] : definition["mask_offset"] + 4], "little")
        selected = _option(index, definition["options"])
        selected["enabled"] = bool(enabled_mask & (1 << index))
        result["interfaces"][interface] = {
            "selected": selected,
            "enabled_mask": enabled_mask,
            "enabled_protocols": [
                _option(i, definition["options"])
                for i in range(len(definition["options"]))
                if enabled_mask & (1 << i)
            ],
        }
    return result


def parse_seplos_identity(payload: bytes) -> dict[str, Any]:
    """Decode the Seplos V3 VIA block (0x1700..0x1732)."""
    if len(payload) < 102:
        raise ValueError("Seplos VIA payload is too short")
    return {
        "factory": _decode_ascii(payload[0:20]),
        "device": _decode_ascii(payload[20:40]),
        "firmware": _decode_ascii(payload[40:42]),
        "bms_serial_number": _decode_ascii(payload[42:72]),
        "pack_serial_number": _decode_ascii(payload[72:102]),
    }


def parse_seplos_protocol(payload: bytes) -> dict[str, Any]:
    """Decode the Seplos V3 PCT block (0x1800..0x1823).

    The selector register has mixed/firmware-specific semantics.  Keep both its
    on-wire value and individual bytes visible instead of guessing a write map.
    """
    if len(payload) < 72:
        raise ValueError("Seplos PCT payload is too short")
    selector = int.from_bytes(payload[0:2], "big")
    profile = next((name for name, item in SEPLOS_PROTOCOLS.items() if item["selector"] == selector), None)
    return {
        "selector_raw": selector,
        "selector_index": payload[0],
        "selector_variant": payload[1],
        "selector_profile": profile,
        "selector_bytes": list(payload[0:2]),
        "baud_rate": int.from_bytes(payload[2:4], "big"),
        "inverter_name": _decode_ascii(payload[4:36]),
        "protocol_name": _decode_ascii(payload[36:68]),
        "protocol_version": _decode_ascii(payload[68:70]),
        "pre_switch_raw": int.from_bytes(payload[70:72], "big"),
        "pre_switch_index": payload[70],
    }


def resolve_protocol(value: str, options: tuple[str, ...]) -> int:
    """Resolve a numeric or exact case-insensitive JK protocol selector."""
    try:
        index = int(value, 0)
    except ValueError:
        matches = [i for i, option in enumerate(options) if option.casefold() == value.casefold()]
        if len(matches) != 1:
            raise ValueError("protocol must be an index or an exact name from the protocol list") from None
        index = matches[0]
    if not 0 <= index < len(options):
        raise ValueError(f"protocol index must be between 0 and {len(options) - 1}")
    return index


def _jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, dict):
        return {key: _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if hasattr(value, "_asdict"):
        return _jsonable(value._asdict())
    return str(value)


def _ble_imports() -> tuple[Any, Any, dict[str, Any]]:
    try:
        from bleak import BleakClient, BleakScanner
        from aiobmsble.bms.daly_bms import BMS as DalyBMS
        from aiobmsble.bms.jikong_bms import BMS as JkBMS
        from aiobmsble.bms.seplos_bms import BMS as SeplosBMS
    except ImportError as exc:
        raise RuntimeError(
            "BLE dependencies are missing; install tools/bms_ble_requirements.txt in a virtual environment"
        ) from exc
    return BleakClient, BleakScanner, {"daly": DalyBMS, "seplos": SeplosBMS, "jk": JkBMS}


async def _find_device(scanner: Any, address: str) -> Any:
    device = await scanner.find_device_by_address(address, timeout=12)
    if device is None:
        raise RuntimeError(f"Bluetooth device {address} was not found")
    return device


async def scan_devices() -> dict[str, Any]:
    """Scan and report the configured BMS devices that are currently present."""
    _, scanner, _ = _ble_imports()
    discovered = await scanner.discover(timeout=10, return_adv=True)
    result: dict[str, Any] = {}
    by_address = {address.upper(): item for address, item in discovered.items()}
    for alias, entry in DEVICE_INVENTORY.items():
        found = by_address.get(entry["address"])
        result[alias] = {**entry, "present": found is not None}
        if found is not None:
            device, advertisement = found
            result[alias].update(
                {
                    "name": device.name,
                    "rssi": advertisement.rssi,
                    "service_uuids": list(advertisement.service_uuids),
                }
            )
    return result


async def read_telemetry(alias: str) -> dict[str, Any]:
    """Read one BMS through the matching aiobmsble driver."""
    _, scanner, drivers = _ble_imports()
    entry = DEVICE_INVENTORY[alias]
    device = await _find_device(scanner, entry["address"])
    async with drivers[alias](device) as bms:
        info = await bms.device_info()
        telemetry = await bms.async_update()
    return {
        "device": alias,
        "address": entry["address"],
        "info": _jsonable(info),
        "telemetry": _jsonable(telemetry),
    }


class _JkSession:
    """Small direct-BLE JK session used for protocol reads and writes."""

    def __init__(self, client: Any) -> None:
        self.client = client
        self.buffer = bytearray()
        self.info_event = asyncio.Event()

    def notification(self, _sender: Any, data: bytearray) -> None:
        chunk = bytes(data)
        while chunk.startswith(b"AT\r\n"):
            chunk = chunk[4:]
        if not chunk:
            return
        header_at = chunk.find(b"\x55\xAA\xEB\x90")
        if header_at >= 0:
            self.buffer.clear()
            chunk = chunk[header_at:]
        if not self.buffer and not chunk.startswith(b"\x55\xAA\xEB\x90"):
            return
        self.buffer.extend(chunk)
        if len(self.buffer) >= JK_INFO_FRAME_SIZE and self.buffer[4] == 0x03:
            self.info_event.set()

    async def device_info_frame(self) -> bytes:
        self.buffer.clear()
        self.info_event.clear()
        await self.client.write_gatt_char(normalize_uuid("ffe1"), build_jk_frame(0x97), response=False)
        await asyncio.wait_for(self.info_event.wait(), timeout=8)
        return bytes(self.buffer[:JK_INFO_FRAME_SIZE])


async def read_jk_protocols() -> dict[str, Any]:
    BleakClient, scanner, _ = _ble_imports()
    address = DEVICE_INVENTORY["jk"]["address"]
    device = await _find_device(scanner, address)
    async with BleakClient(device, timeout=15) as client:
        session = _JkSession(client)
        await client.start_notify(normalize_uuid("ffe1"), session.notification)
        frame = await session.device_info_frame()
    return {"device": "jk", "address": address, **parse_jk_device_info(frame)}


class _SeplosSession:
    """Direct-BLE Seplos V3 read session."""

    def __init__(self, client: Any) -> None:
        self.client = client
        self.buffer = bytearray()
        self.frame_event = asyncio.Event()

    def _expected_length(self) -> int | None:
        if len(self.buffer) < 2:
            return None
        function = self.buffer[1]
        if function & 0x80:
            return 5
        if function in (0x01, 0x04):
            return 3 + self.buffer[2] + 2 if len(self.buffer) >= 3 else None
        if function in (0x0F, 0x10):
            return 8
        return None

    def notification(self, _sender: Any, data: bytearray) -> None:
        if data and len(data) >= 2 and data[1] & 0x7F in (0x01, 0x04, 0x0F, 0x10):
            self.buffer.clear()
            self.frame_event.clear()
        self.buffer.extend(data)
        expected = self._expected_length()
        if expected is not None and len(self.buffer) >= expected:
            self.frame_event.set()

    async def _response(self) -> bytes:
        await asyncio.wait_for(self.frame_event.wait(), timeout=8)
        expected = self._expected_length()
        if expected is None:
            raise RuntimeError("Seplos returned an unsupported response")
        frame = bytes(self.buffer[:expected])
        if frame[1] & 0x80:
            raise RuntimeError(f"Seplos returned Modbus exception 0x{frame[2]:02X}")
        if crc16_modbus(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
            raise RuntimeError("Seplos response CRC mismatch")
        return frame

    async def read(self, register: int, count: int, function: int = 0x04) -> bytes:
        self.buffer.clear()
        self.frame_event.clear()
        await self.client.write_gatt_char(
            normalize_uuid("fff2"), build_modbus_read(0, function, register, count), response=False
        )
        frame = await self._response()
        return frame[3:-2]

    async def write(self, register: int, *values: int) -> None:
        self.buffer.clear()
        self.frame_event.clear()
        await self.client.write_gatt_char(
            normalize_uuid("fff2"), build_modbus_write(0, register, tuple(values)), response=False
        )
        frame = await self._response()
        if frame[1] != 0x10 or int.from_bytes(frame[2:4], "big") != register:
            raise RuntimeError("Seplos write acknowledgement does not match the request")
        if int.from_bytes(frame[4:6], "big") != len(values):
            raise RuntimeError("Seplos write acknowledgement has the wrong register count")


async def read_seplos_protocol() -> dict[str, Any]:
    BleakClient, scanner, _ = _ble_imports()
    address = DEVICE_INVENTORY["seplos"]["address"]
    device = await _find_device(scanner, address)
    async with BleakClient(device, timeout=15) as client:
        session = _SeplosSession(client)
        await client.start_notify(normalize_uuid("fff1"), session.notification)
        identity = parse_seplos_identity(await session.read(0x1700, 0x33))
        protocol = parse_seplos_protocol(await session.read(0x1800, 0x24))
    return {"device": "seplos", "address": address, "identity": identity, "inverter_protocol": protocol}


async def _read_seplos_session_state(session: _SeplosSession) -> tuple[dict[str, Any], dict[str, Any]]:
    identity = parse_seplos_identity(await session.read(0x1700, 0x33))
    protocol = parse_seplos_protocol(await session.read(0x1800, 0x24))
    return identity, protocol


async def _write_seplos_selector(session: _SeplosSession, selector: int) -> None:
    # Stage the profile first, then commit it.  Both registers are documented
    # R/W by the XZH V3 Modbus protocol; using the pre-switch register mirrors
    # the two-phase selection exposed by the manufacturer software.
    # This firmware byte-swaps a UINT16 written through function 0x10.  The
    # profile index is therefore sent as 0x0009 so the stored/read selector is
    # 0x0900.  This behavior was verified against the live pre-switch register
    # before committing the active selector.
    index = selector >> 8
    if selector != index << 8:
        raise ValueError("unsupported Seplos selector encoding")
    await session.write(0x1823, index)
    staged = parse_seplos_protocol(await session.read(0x1800, 0x24))
    if staged["pre_switch_raw"] != selector:
        raise RuntimeError("Seplos did not confirm the staged protocol selector")
    await session.write(0x1800, index)


async def set_seplos_protocol(profile_name: str, confirmation: str) -> dict[str, Any]:
    """Switch the verified Seplos inverter profile with read-back and rollback."""
    BleakClient, scanner, _ = _ble_imports()
    entry = DEVICE_INVENTORY["seplos"]
    address = entry["address"]
    if confirmation.upper() != entry["advertised_name"].upper():
        raise ValueError(f"confirmation must exactly match the Seplos serial: {entry['advertised_name']}")
    target = SEPLOS_PROTOCOLS[profile_name]
    device = await _find_device(scanner, address)

    async with BleakClient(device, timeout=15) as client:
        session = _SeplosSession(client)
        await client.start_notify(normalize_uuid("fff1"), session.notification)
        identity, before = await _read_seplos_session_state(session)
        if identity["bms_serial_number"].upper() != entry["advertised_name"].upper():
            raise RuntimeError("connected Seplos identity does not match the configured battery")
        if before["selector_raw"] == target["selector"]:
            return {
                "changed": False,
                "reason": "already selected",
                "identity": identity,
                "before": before,
                "after": before,
            }

        previous_selector = before["selector_raw"]
        try:
            await _write_seplos_selector(session, target["selector"])
            await asyncio.sleep(1)
            _, after = await _read_seplos_session_state(session)
            if after["selector_raw"] != target["selector"]:
                raise RuntimeError("Seplos did not report the requested protocol after the write")
            if after["baud_rate"] != target["baud_rate"]:
                raise RuntimeError("Seplos reported an unexpected baud rate for the requested protocol")
        except Exception:
            try:
                await _write_seplos_selector(session, previous_selector)
            except Exception as rollback_error:
                raise RuntimeError(
                    f"Seplos protocol change failed and rollback also failed: {rollback_error}"
                ) from rollback_error
            raise

    return {
        "changed": True,
        "profile": profile_name,
        "identity": identity,
        "before": before,
        "after": after,
    }


async def set_jk_protocol(interface: str, protocol: str, confirmation: str) -> dict[str, Any]:
    """Set a JK protocol only after identity, support-mask and MAC checks."""
    BleakClient, scanner, _ = _ble_imports()
    entry = DEVICE_INVENTORY["jk"]
    address = entry["address"]
    if confirmation.upper() != address:
        raise ValueError(f"confirmation must exactly match the JK MAC address: {address}")

    definition = JK_PROTOCOL_INTERFACES[interface]
    index = resolve_protocol(protocol, definition["options"])
    device = await _find_device(scanner, address)
    async with BleakClient(device, timeout=15) as client:
        session = _JkSession(client)
        await client.start_notify(normalize_uuid("ffe1"), session.notification)
        before = parse_jk_device_info(await session.device_info_frame())
        before_interface = before["interfaces"][interface]
        if not before_interface["enabled_mask"] & (1 << index):
            raise ValueError(f"protocol index {index} is not enabled by this JK firmware on {interface}")
        if before_interface["selected"]["index"] == index:
            return {"changed": False, "reason": "already selected", "before": before, "after": before}

        await client.write_gatt_char(
            normalize_uuid("ffe1"),
            build_jk_frame(definition["register"], index, 0x02),
            response=False,
        )
        await asyncio.sleep(1)
        after = parse_jk_device_info(await session.device_info_frame())

    if after["interfaces"][interface]["selected"]["index"] != index:
        raise RuntimeError("JK did not report the requested protocol after the write")
    return {"changed": True, "interface": interface, "before": before, "after": after}


async def _run(args: argparse.Namespace) -> Any:
    if args.command == "scan":
        return await scan_devices()
    if args.command == "read":
        aliases = list(DEVICE_INVENTORY) if args.device == "all" else [args.device]
        result: dict[str, Any] = {}
        for alias in aliases:
            try:
                result[alias] = await read_telemetry(alias)
            except Exception as exc:  # Continue so an absent BMS does not hide the others.
                result[alias] = {"error": f"{type(exc).__name__}: {exc}"}
        return result
    if args.command == "protocols":
        aliases = ("jk", "seplos") if args.device == "all" else (args.device,)
        result = {}
        for alias in aliases:
            try:
                if alias == "jk":
                    result[alias] = await read_jk_protocols()
                elif alias == "seplos":
                    result[alias] = await read_seplos_protocol()
                else:
                    result[alias] = {
                        "note": "No verified Daly inverter-protocol selector is exposed by this BLE firmware."
                    }
            except Exception as exc:
                result[alias] = {"error": f"{type(exc).__name__}: {exc}"}
        return result
    if args.command == "set-jk-protocol":
        return await set_jk_protocol(args.interface, args.protocol, args.confirm)
    if args.command == "set-seplos-protocol":
        return await set_seplos_protocol(args.profile, args.confirm)
    raise ValueError(f"unknown command: {args.command}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Interact with the local Daly, Seplos and JK BMS devices over BLE")
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("scan", help="show which configured BMS devices are currently in range")

    read_parser = subparsers.add_parser("read", help="read identity and telemetry without changing settings")
    read_parser.add_argument("--device", choices=("all", *DEVICE_INVENTORY), default="all")

    protocol_parser = subparsers.add_parser("protocols", help="read current and available communication protocols")
    protocol_parser.add_argument("--device", choices=("all", *DEVICE_INVENTORY), default="all")

    set_parser = subparsers.add_parser("set-jk-protocol", help="change one verified JK UART/CAN protocol selector")
    set_parser.add_argument("--interface", choices=tuple(JK_PROTOCOL_INTERFACES), required=True)
    set_parser.add_argument("--protocol", required=True, help="numeric index or exact protocol name")
    set_parser.add_argument("--confirm", required=True, help="must exactly match the target JK MAC address")

    seplos_set_parser = subparsers.add_parser(
        "set-seplos-protocol", help="switch one verified Seplos inverter communication profile"
    )
    seplos_set_parser.add_argument("--profile", choices=tuple(SEPLOS_PROTOCOLS), required=True)
    seplos_set_parser.add_argument(
        "--confirm", required=True, help="must exactly match the target Seplos serial number"
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        print(json.dumps(asyncio.run(_run(args)), indent=2, sort_keys=True))
    except (RuntimeError, TimeoutError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
