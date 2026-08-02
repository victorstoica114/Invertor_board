#!/usr/bin/env python3
"""Local-network telemetry and guarded configuration clients for Wi-Fi inverters."""

from __future__ import annotations

import contextlib
import fcntl
import math
import os
from pathlib import Path
import re
import socket
import time
from collections.abc import Iterator
from typing import Any


DISCOVERY_PORT = 58899
MAX_FRAME_BYTES = 4096
DEV_CODE_PATTERN = re.compile(rb"rsp>server=\s*(\d+)\s*;")
NETWORK_LOCK_PATH = Path(
    os.environ.get(
        "INVERTER_NETWORK_LOCK",
        f"/run/user/{os.getuid()}/inverter-network.lock",
    )
)


class InverterProtocolError(Exception):
    """Raised when a dongle or inverter returns an invalid/incomplete frame."""


@contextlib.contextmanager
def inverter_network_lock(timeout_seconds: float = 15.0) -> Iterator[None]:
    """Serialize reverse-tunnel use across the collector and control server."""
    NETWORK_LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic() + timeout_seconds
    with NETWORK_LOCK_PATH.open("a+", encoding="ascii") as lock_file:
        while True:
            try:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    raise TimeoutError("another inverter network operation is in progress")
                time.sleep(0.05)
        try:
            yield
        finally:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def crc16_modbus(data: bytes) -> int:
    """Return the Modbus CRC-16 value (poly 0xA001, initial value 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def crc16_xmodem(data: bytes) -> int:
    """Return the CRC-16/XMODEM value used by Voltronic ASCII commands."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _adjust_voltronic_crc_byte(value: int) -> int:
    return (value + 1) & 0xFF if value in (0x0A, 0x0D, 0x28) else value


def _recv_exact(connection: socket.socket, count: int) -> bytes:
    chunks: list[bytes] = []
    remaining = count
    while remaining:
        chunk = connection.recv(remaining)
        if not chunk:
            raise InverterProtocolError(
                f"connection closed with {remaining} of {count} bytes still expected"
            )
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def receive_wrapped_frame(connection: socket.socket) -> bytes:
    """Receive one Eybond wrapper frame using its big-endian length field."""
    header = _recv_exact(connection, 6)
    payload_length = int.from_bytes(header[4:6], "big")
    if payload_length < 2 or payload_length > MAX_FRAME_BYTES - len(header):
        raise InverterProtocolError(f"invalid wrapper payload length: {payload_length}")
    return header + _recv_exact(connection, payload_length)


@contextlib.contextmanager
def reverse_tunnel(
    inverter_ip: str,
    local_ip: str,
    local_port: int,
    timeout_seconds: float,
) -> Iterator[tuple[socket.socket, int]]:
    """Redirect one Eybond dongle and accept its reverse TCP connection."""
    deadline = time.monotonic() + timeout_seconds
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((local_ip, local_port))
        server.listen(2)
        server.settimeout(timeout_seconds)

        request = f"set>server={local_ip}:{local_port};".encode("ascii")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            udp.settimeout(timeout_seconds)
            udp.connect((inverter_ip, DISCOVERY_PORT))
            udp.send(request)
            reply = udp.recv(MAX_FRAME_BYTES)

        match = DEV_CODE_PATTERN.fullmatch(reply.strip())
        if match is None:
            raise InverterProtocolError(f"invalid UDP discovery response: {reply!r}")
        dev_code = int(match.group(1))
        if not 0 <= dev_code <= 0xFFFF:
            raise InverterProtocolError(f"invalid dongle device code: {dev_code}")

        connection: socket.socket | None = None
        while connection is None:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"{inverter_ip} did not connect to {local_ip}:{local_port}"
                )
            server.settimeout(remaining)
            candidate, address = server.accept()
            if address[0] != inverter_ip:
                candidate.close()
                continue
            connection = candidate

        with connection:
            connection.settimeout(timeout_seconds)
            yield connection, dev_code


ANENJI_REGISTER_DEFS: tuple[tuple[int, str | None, int, bool], ...] = (
    (201, "working_mode_code", 1, False),
    (202, "grid_voltage_v", 10, True),
    (203, "grid_frequency_hz", 100, True),
    (204, "grid_power_w", 1, True),
    (205, "inverter_voltage_v", 10, True),
    (206, "inverter_current_a", 10, True),
    (207, "inverter_frequency_hz", 100, True),
    (208, "inverter_power_w", 1, True),
    (209, "inverter_charging_power_w", 1, True),
    (210, "output_voltage_v", 10, True),
    (211, "output_current_a", 10, True),
    (212, "output_frequency_hz", 100, True),
    (213, "output_power_w", 1, True),
    (214, "output_apparent_power_va", 1, True),
    (215, "battery_voltage_v", 10, True),
    (216, None, 1, True),
    (217, "battery_power_w", 1, True),
    (218, None, 1, True),
    (219, "pv_voltage_v", 10, True),
    (220, "pv_current_a", 10, True),
    (221, None, 1, True),
    (222, None, 1, True),
    (223, "pv_power_w", 1, True),
    (224, "pv_charging_power_w", 1, True),
    (225, "load_pct", 1, True),
    (226, "dcdc_temperature_c", 1, True),
    (227, "inverter_temperature_c", 1, True),
    (228, "pv_temperature_c", 1, True),
    (229, "battery_soc_pct", 1, False),
    (230, None, 1, True),
    (231, "power_flow_status", 1, False),
    (232, "battery_current_a", 10, True),
    (233, "inverter_charging_average_current_a", 10, True),
    (234, "pv_charging_average_current_a", 10, True),
)

ANENJI_WORKING_MODES = {
    0: "POWER_ON",
    1: "STANDBY",
    2: "MAINS",
    3: "OFF_GRID",
    4: "BYPASS",
    5: "CHARGING",
    6: "FAULT",
}


def build_anenji_read_request(
    transaction_id: int,
    dev_code: int,
    register: int = 201,
    count: int = 34,
    slave_id: int = 1,
) -> bytes:
    pdu = bytes(
        (
            slave_id,
            0x03,
            (register >> 8) & 0xFF,
            register & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
        )
    )
    crc = crc16_modbus(pdu)
    rtu = pdu + crc.to_bytes(2, "little")
    body = b"\xff\x04" + rtu
    return (
        transaction_id.to_bytes(2, "big")
        + dev_code.to_bytes(2, "big")
        + len(body).to_bytes(2, "big")
        + body
    )


def build_anenji_write_request(
    transaction_id: int,
    dev_code: int,
    register: int,
    values: list[int] | tuple[int, ...],
    slave_id: int = 1,
) -> bytes:
    """Build an Eybond-wrapped Modbus 0x10 holding-register write."""
    if not values:
        raise ValueError("at least one register value is required")
    if len(values) > 0x7B:
        raise ValueError("too many Modbus registers in one write")
    if not 0 <= register <= 0xFFFF:
        raise ValueError("register must fit in 16 bits")
    if any(not 0 <= int(value) <= 0xFFFF for value in values):
        raise ValueError("register values must fit in 16 bits")
    register_bytes = b"".join(int(value).to_bytes(2, "big") for value in values)
    count = len(values)
    pdu = bytes(
        (
            slave_id,
            0x10,
            (register >> 8) & 0xFF,
            register & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
            len(register_bytes),
        )
    ) + register_bytes
    crc = crc16_modbus(pdu)
    body = b"\xff\x04" + pdu + crc.to_bytes(2, "little")
    return (
        transaction_id.to_bytes(2, "big")
        + dev_code.to_bytes(2, "big")
        + len(body).to_bytes(2, "big")
        + body
    )


def parse_anenji_register_response(
    frame: bytes,
    expected_transaction_id: int,
    count: int,
) -> tuple[list[int], str]:
    """Decode a wrapped Modbus 0x03 response for an arbitrary register count."""
    if len(frame) < 13:
        raise InverterProtocolError(f"Anenji response is too short: {len(frame)}")
    transaction_id = int.from_bytes(frame[0:2], "big")
    if transaction_id != expected_transaction_id:
        raise InverterProtocolError(
            f"transaction mismatch: expected {expected_transaction_id}, got {transaction_id}"
        )
    if frame[6:9] != b"\xff\x04\x01":
        raise InverterProtocolError(f"unexpected Anenji wrapper: {frame[6:9].hex()}")
    function = frame[9]
    if function == 0x83:
        error_code = frame[10] if len(frame) > 10 else -1
        raise InverterProtocolError(f"Anenji read rejected with Modbus error {error_code}")
    if function != 0x03:
        raise InverterProtocolError(f"unexpected Anenji function: 0x{function:02x}")

    byte_count = count * 2
    if len(frame) == 13 + byte_count and frame[10] == byte_count:
        layout = "byte_count"
        register_bytes = frame[11:-2]
    elif len(frame) == 12 + byte_count:
        layout = "legacy"
        register_bytes = frame[10:-2]
    else:
        raise InverterProtocolError(
            f"unexpected Anenji frame length {len(frame)} for {count} registers"
        )
    expected_crc = crc16_modbus(frame[8:-2])
    received_crc = int.from_bytes(frame[-2:], "little")
    if received_crc != expected_crc:
        raise InverterProtocolError(
            f"Anenji CRC mismatch: expected {expected_crc:04x}, got {received_crc:04x}"
        )
    return (
        [
            int.from_bytes(register_bytes[offset : offset + 2], "big")
            for offset in range(0, len(register_bytes), 2)
        ],
        layout,
    )


def parse_anenji_write_response(
    frame: bytes,
    expected_transaction_id: int,
    register: int,
    count: int,
) -> None:
    """Validate a wrapped Modbus 0x10 acknowledgement."""
    if len(frame) != 16:
        raise InverterProtocolError(f"unexpected Anenji write response length: {len(frame)}")
    transaction_id = int.from_bytes(frame[0:2], "big")
    if transaction_id != expected_transaction_id:
        raise InverterProtocolError(
            f"transaction mismatch: expected {expected_transaction_id}, got {transaction_id}"
        )
    if frame[6:9] != b"\xff\x04\x01":
        raise InverterProtocolError(f"unexpected Anenji wrapper: {frame[6:9].hex()}")
    if frame[9] == 0x90:
        raise InverterProtocolError(f"Anenji write rejected with Modbus error {frame[10]}")
    if frame[9] != 0x10:
        raise InverterProtocolError(f"unexpected Anenji write function: 0x{frame[9]:02x}")
    expected_crc = crc16_modbus(frame[8:-2])
    received_crc = int.from_bytes(frame[-2:], "little")
    if received_crc != expected_crc:
        raise InverterProtocolError(
            f"Anenji CRC mismatch: expected {expected_crc:04x}, got {received_crc:04x}"
        )
    acknowledged_register = int.from_bytes(frame[10:12], "big")
    acknowledged_count = int.from_bytes(frame[12:14], "big")
    if (acknowledged_register, acknowledged_count) != (register, count):
        raise InverterProtocolError(
            "Anenji write acknowledgement does not match the requested register range"
        )


def parse_anenji_response(frame: bytes, expected_transaction_id: int = 1) -> dict[str, Any]:
    """Decode the Anenji/SRNE register block 201..234."""
    raw_values, layout = parse_anenji_register_response(
        frame, expected_transaction_id, len(ANENJI_REGISTER_DEFS)
    )

    values: dict[str, int | float] = {}
    raw_registers: dict[str, int] = {}
    for raw, (address, name, divisor, signed) in zip(raw_values, ANENJI_REGISTER_DEFS):
        raw_registers[str(address)] = raw
        decoded = raw - 0x10000 if signed and raw >= 0x8000 else raw
        if name is not None:
            values[name] = decoded if divisor == 1 else decoded / divisor

    mode_code = int(values["working_mode_code"])
    battery_current = float(values["battery_current_a"])
    values.update(
        {
            "protocol": "ANENJI_MODBUS_201_234",
            "working_mode": ANENJI_WORKING_MODES.get(mode_code, f"UNKNOWN_{mode_code}"),
            # Register 232 is directional: positive means charging and negative
            # means discharging. Keep the signed value and expose the two
            # non-negative directions used by the database and dashboard.
            "battery_charge_current_a": battery_current if battery_current > 0 else 0.0,
            "battery_discharge_current_a": -battery_current if battery_current < 0 else 0.0,
            "raw": {
                "layout": layout,
                "transaction_id": expected_transaction_id,
                "dev_code": int.from_bytes(frame[2:4], "big"),
                "registers": raw_registers,
                "frame_hex": frame.hex(),
            },
        }
    )
    return values


def read_anenji(
    inverter_ip: str,
    local_ip: str,
    local_port: int,
    timeout_seconds: float,
) -> dict[str, Any]:
    with reverse_tunnel(inverter_ip, local_ip, local_port, timeout_seconds) as (
        connection,
        dev_code,
    ):
        connection.sendall(build_anenji_read_request(1, dev_code))
        return parse_anenji_response(receive_wrapped_frame(connection))


def _options(*items: tuple[int | float | str, str]) -> list[dict[str, Any]]:
    return [{"value": value, "label": label} for value, label in items]


ANENJI_CONFIGURATION_SPECS: tuple[dict[str, Any], ...] = (
    {"key": "output_mode", "label": "Output mode", "group": "Operating modes", "address": 300,
     "options": _options((0, "Single unit")), "critical": True},
    {"key": "output_priority", "label": "Output source priority", "group": "Operating modes", "address": 301,
     "options": _options((0, "Utility → PV → battery (UTI)"), (1, "PV → utility → battery (SOL)"),
                         (2, "PV → battery → utility (SBU)"), (3, "PV → utility → battery (SUB)")), "critical": True},
    {"key": "input_mode", "label": "AC input range", "group": "Operating modes", "address": 302,
     "options": _options((0, "Appliance (APL)"), (1, "UPS"), (2, "Generator (GNT)")), "critical": True},
    {"key": "buzzer_mode", "label": "Buzzer mode", "group": "Operating modes", "address": 303,
     "options": _options((0, "Muted"), (1, "Source changes, warnings and faults"),
                         (2, "Warnings and faults"), (3, "Faults only"))},
    {"key": "lcd_backlight", "label": "LCD backlight", "group": "Operating modes", "address": 305,
     "options": _options((0, "Timed off"), (1, "Always on"))},
    {"key": "lcd_auto_return", "label": "LCD auto-return", "group": "Operating modes", "address": 306,
     "options": _options((0, "Disabled"), (1, "After one minute"))},
    {"key": "power_saving", "label": "Power saving", "group": "Operating modes", "address": 307,
     "options": _options((0, "Disabled"), (1, "Enabled")), "critical": True},
    {"key": "overload_restart", "label": "Restart after overload", "group": "Protection and alarms", "address": 308,
     "options": _options((0, "Disabled"), (1, "Enabled"))},
    {"key": "overtemperature_restart", "label": "Restart after overtemperature", "group": "Protection and alarms", "address": 309,
     "options": _options((0, "Disabled"), (1, "Enabled"))},
    {"key": "overload_bypass", "label": "Transfer to bypass on overload", "group": "Protection and alarms", "address": 310,
     "options": _options((0, "Disabled"), (1, "Enabled")), "critical": True},
    {"key": "equalization_enabled", "label": "Battery equalization", "group": "Equalization", "address": 313,
     "options": _options((0, "Disabled"), (1, "Enabled")), "critical": True},
    {"key": "warning_mask", "label": "Warning display mask", "group": "Protection and alarms", "address": 314,
     "count": 2, "input_type": "text", "minimum": 0, "maximum": 0xFFFFFFFF, "format": "hex",
     "description": "32-bit mask: a set bit allows the corresponding warning to be displayed.", "critical": True},
    {"key": "output_voltage", "label": "Output voltage", "group": "AC output", "address": 320, "scale": 10,
     "unit": "V", "options": _options((220, "220 V"), (230, "230 V"), (240, "240 V")), "critical": True},
    {"key": "output_frequency", "label": "Output frequency", "group": "AC output", "address": 321, "scale": 100,
     "unit": "Hz", "options": _options((50, "50 Hz"), (60, "60 Hz")), "critical": True},
    {"key": "battery_type", "label": "Battery type", "group": "Battery and charging", "address": 322,
     "options": _options((0, "AGM"), (1, "Flooded"), (2, "User-defined"), (4, "Lithium Li2"),
                         (6, "Lithium Li4"), (8, "Lithium BMS (LiB)")),
     "description": "Changing battery type can reset dependent charge parameters.", "critical": True},
    {"key": "battery_overvoltage", "label": "Battery overvoltage protection", "group": "Battery and charging", "address": 323,
     "scale": 10, "unit": "V", "minimum": 40, "maximum": 66, "step": 0.1, "critical": True},
    {"key": "maximum_charge_voltage", "label": "Maximum charge voltage", "group": "Battery and charging", "address": 324,
     "scale": 10, "unit": "V", "minimum": 40, "maximum": 65, "step": 0.1, "critical": True},
    {"key": "float_charge_voltage", "label": "Float charge voltage", "group": "Battery and charging", "address": 325,
     "scale": 10, "unit": "V", "minimum": 40, "maximum": 65, "step": 0.1, "critical": True},
    {"key": "utility_discharge_recovery_voltage", "label": "Return from utility to battery", "group": "Battery and charging", "address": 326,
     "scale": 10, "unit": "V", "minimum": 0, "maximum": 65, "step": 0.1,
     "description": "Zero means return only when the battery is full.", "critical": True},
    {"key": "utility_low_battery_voltage", "label": "Transfer to utility voltage", "group": "Battery and charging", "address": 327,
     "scale": 10, "unit": "V", "minimum": 40, "maximum": 60, "step": 0.1, "critical": True},
    {"key": "offgrid_low_battery_voltage", "label": "Off-grid low battery cutoff", "group": "Battery and charging", "address": 329,
     "scale": 10, "unit": "V", "minimum": 36, "maximum": 58, "step": 0.1, "critical": True},
    {"key": "cv_to_float_minutes", "label": "CV-to-float wait", "group": "Battery and charging", "address": 330,
     "unit": "min", "minimum": 0, "maximum": 900, "step": 1},
    {"key": "charger_priority", "label": "Charging source priority", "group": "Battery and charging", "address": 331,
     "options": _options((1, "PV first (CSO)"), (2, "PV and utility (SNU)"), (3, "PV only (OSO)")), "critical": True},
    {"key": "maximum_charge_current", "label": "Maximum total charge current", "group": "Battery and charging", "address": 332,
     "scale": 10, "unit": "A", "minimum": 10, "maximum": 200, "step": 0.1, "critical": True},
    {"key": "maximum_utility_charge_current", "label": "Maximum utility charge current", "group": "Battery and charging", "address": 333,
     "scale": 10, "unit": "A", "minimum": 5, "maximum": 200, "step": 0.1, "critical": True},
    {"key": "equalization_voltage", "label": "Equalization voltage", "group": "Equalization", "address": 334,
     "scale": 10, "unit": "V", "minimum": 40, "maximum": 65, "step": 0.1, "critical": True},
    {"key": "equalization_time", "label": "Equalization time", "group": "Equalization", "address": 335,
     "unit": "min", "minimum": 0, "maximum": 900, "step": 1, "critical": True},
    {"key": "equalization_timeout", "label": "Equalization timeout", "group": "Equalization", "address": 336,
     "unit": "min", "minimum": 0, "maximum": 900, "step": 1, "critical": True},
    {"key": "equalization_interval", "label": "Equalization interval", "group": "Equalization", "address": 337,
     "unit": "days", "minimum": 1, "maximum": 90, "step": 1, "critical": True},
    {"key": "automatic_utility_output", "label": "Automatic utility output", "group": "Advanced", "address": 338,
     "options": _options((0, "Requires power button"), (1, "Automatic")), "critical": True},
    {"key": "automatic_lithium_activation", "label": "Automatic lithium activation", "group": "Advanced", "address": 339, "count": 2,
     "options": _options((0, "Disabled"), (1, "Enabled")), "critical": True},
    {"key": "utility_discharge_soc", "label": "Utility-mode discharge SOC limit", "group": "SOC limits", "address": 341,
     "unit": "%", "minimum": 5, "maximum": 96, "step": 1, "critical": True},
    {"key": "utility_discharge_recovery_soc", "label": "Utility-mode recovery SOC", "group": "SOC limits", "address": 342,
     "unit": "%", "minimum": 10, "maximum": 100, "step": 1, "critical": True},
    {"key": "offgrid_op1_discharge_soc", "label": "Off-grid OP1 discharge SOC limit", "group": "SOC limits", "address": 343,
     "unit": "%", "minimum": 0, "maximum": 95, "step": 1, "critical": True},
    {"key": "offgrid_discharge_soc", "label": "Off-grid discharge SOC limit", "group": "SOC limits", "address": 344,
     "unit": "%", "minimum": 0, "maximum": 95, "step": 1, "critical": True},
    {"key": "rgb_mode", "label": "RGB mode", "group": "Advanced", "address": 346,
     "options": _options((0, "MD1 normal"), (1, "MD2 fault only"), (2, "MD3 off"),
                         (3, "MD4 always lit"), (4, "MD5 always breathing"))},
    {"key": "lithium_stop_charge_soc", "label": "Lithium stop-charge SOC", "group": "SOC limits", "address": 347,
     "unit": "%", "minimum": 20, "maximum": 100, "step": 1, "critical": True},
    {"key": "maximum_discharge_current", "label": "Maximum discharge current protection", "group": "Protection and alarms", "address": 351,
     "unit": "A", "minimum": 0, "maximum": 500, "step": 1, "critical": True},
    {"key": "offgrid_op1_low_battery_voltage", "label": "Off-grid OP1 low battery voltage", "group": "Battery and charging", "address": 352,
     "scale": 10, "unit": "V", "minimum": 36, "maximum": 58, "step": 0.1, "critical": True},
    {"key": "op2_overload_warning", "label": "OP2 overload warning", "group": "Advanced", "address": 353,
     "unit": "%", "minimum": 10, "maximum": 100, "step": 1},
    {"key": "op2_output_enabled", "label": "OP2 output", "group": "Advanced", "address": 354,
     "options": _options((0, "Disabled"), (1, "Enabled")), "critical": True},
    {"key": "utility_recognition_time", "label": "Utility recognition time", "group": "Advanced", "address": 355,
     "unit": "s", "minimum": 5, "maximum": 300, "step": 1},
    {"key": "lithium_activation_time", "label": "Lithium activation time", "group": "Advanced", "address": 356,
     "unit": "s", "minimum": 6, "maximum": 300, "step": 1},
    {"key": "power_on_method", "label": "Power-on method", "group": "Remote operation", "address": 406,
     "options": _options((0, "Local or remote"), (1, "Local only"), (2, "Remote only")), "critical": True},
    {"key": "remote_switch", "label": "Remote power switch", "group": "Remote operation", "address": 420,
     "options": _options((0, "Power off"), (1, "Power on")), "critical": True,
     "description": "Powering off can interrupt loads and may temporarily make the inverter unreachable."},
    {"key": "bms_follow_mode", "label": "BMS follow mode", "group": "Battery and charging", "address": 896,
     "options": _options((0, "Do not follow BMS"), (1, "Follow charge/float voltage"),
                         (2, "Follow voltage and maximum charge current"),
                         (3, "Stop by SOC; local current"), (4, "Stop by SOC; follow BMS current")), "critical": True},
)

ANENJI_ACTION_SPECS: tuple[dict[str, Any], ...] = (
    {"key": "force_equalization", "label": "Force one equalization cycle", "group": "Remote operation",
     "address": 425, "write_value": 1, "description": "Starts one manual equalization cycle.", "critical": True},
    {"key": "clear_fault_lock", "label": "Clear fault lock", "group": "Remote operation",
     "address": 426, "write_value": 1, "description": "Only has an effect while the inverter is fault-locked.", "critical": True},
)


def _anenji_read_registers(
    connection: socket.socket,
    dev_code: int,
    transaction_id: int,
    register: int,
    count: int,
) -> tuple[list[int], str]:
    connection.sendall(build_anenji_read_request(transaction_id, dev_code, register, count))
    return parse_anenji_register_response(
        receive_wrapped_frame(connection), transaction_id, count
    )


def _decode_ascii_registers(values: list[int]) -> str:
    data = b"".join(value.to_bytes(2, "big") for value in values)
    return data.rstrip(b"\x00\xff ").decode("ascii", errors="replace")


def _anenji_configuration_from_connection(
    connection: socket.socket,
    dev_code: int,
    first_transaction_id: int = 1,
) -> tuple[dict[str, Any], int]:
    blocks = ((100, 10), (171, 1), (184, 14), (300, 16), (320, 37),
              (406, 1), (420, 1), (626, 8), (643, 5), (896, 1))
    registers: dict[int, int] = {}
    layouts: set[str] = set()
    transaction_id = first_transaction_id
    for start, count in blocks:
        raw_values, layout = _anenji_read_registers(
            connection, dev_code, transaction_id, start, count
        )
        registers.update({start + offset: raw for offset, raw in enumerate(raw_values)})
        layouts.add(layout)
        transaction_id += 1

    values: dict[str, Any] = {}
    groups: dict[str, list[dict[str, Any]]] = {}
    inconsistencies: dict[str, str] = {}
    for spec in ANENJI_CONFIGURATION_SPECS:
        address = int(spec["address"])
        count = int(spec.get("count", 1))
        raw = registers[address]
        if count == 2:
            raw = (raw << 16) | registers[address + 1]
        scale = int(spec.get("scale", 1))
        value: int | float = raw if scale == 1 else raw / scale
        values[str(spec["key"])] = value
        writable = True
        if spec.get("options") and not any(
            _same_value(value, option["value"]) for option in spec["options"]
        ):
            writable = False
        if spec.get("minimum") is not None and value < spec["minimum"]:
            writable = False
        if spec.get("maximum") is not None and value > spec["maximum"]:
            writable = False
        if not writable:
            inconsistencies[str(spec["key"])] = (
                f"register {address} reports {value}, outside this protocol map"
            )
        item = {key: value for key, value in spec.items() if key not in {"group", "count", "scale", "format"}}
        item.update({"type": "select" if spec.get("options") else spec.get("input_type", "number"),
                     "value": value, "raw_value": raw, "writable": writable})
        if not writable:
            item["description"] = inconsistencies[str(spec["key"])] + "; writing is disabled."
        if spec.get("format") == "hex":
            item["value"] = f"0x{raw:08X}"
            item["display_value"] = item["value"]
        groups.setdefault(str(spec["group"]), []).append(item)
    for spec in ANENJI_ACTION_SPECS:
        item = {key: value for key, value in spec.items() if key != "group"}
        item.update({"type": "action", "writable": True})
        groups.setdefault(str(spec["group"]), []).append(item)

    serial = _decode_ascii_registers([registers[address] for address in range(186, 198)])
    firmware = _decode_ascii_registers([registers[address] for address in range(626, 634)])
    configuration = {
        "protocol": "anenji_modbus",
        "identity": {
            "serial": serial,
            "device_type": registers[171],
            "protocol_version": registers[184],
            "firmware": firmware,
        },
        "ratings": {
            "rated_power_w": registers[643],
            "battery_series_count": registers[644],
            "eeprom_maximum_charge_voltage_v": registers[645] / 10,
            "eeprom_float_charge_voltage_v": registers[646] / 10,
            "eeprom_maximum_charge_current_a": registers[647] / 10,
        },
        "status": {
            "fault_mask": (registers[100] << 16) | registers[101],
            "warning_mask_unfiltered": (registers[104] << 16) | registers[105],
            "warning_mask_filtered": (registers[108] << 16) | registers[109],
        },
        "groups": [{"id": re.sub(r"[^a-z0-9]+", "-", name.lower()).strip("-"),
                    "title": name, "settings": items} for name, items in groups.items()],
        "values": values,
        "raw": {"dev_code": dev_code, "layouts": sorted(layouts),
                "registers": {str(key): value for key, value in sorted(registers.items())},
                "inconsistencies": inconsistencies},
    }
    return configuration, transaction_id


def read_anenji_configuration(
    inverter_ip: str,
    local_ip: str,
    local_port: int,
    timeout_seconds: float,
) -> dict[str, Any]:
    with reverse_tunnel(inverter_ip, local_ip, local_port, timeout_seconds) as (connection, dev_code):
        configuration, _ = _anenji_configuration_from_connection(connection, dev_code)
        return configuration


def build_easun_request(command: str, transaction_id: int, dev_code: int) -> bytes:
    command_bytes = command.encode("ascii")
    crc = crc16_xmodem(command_bytes)
    crc_bytes = bytes(
        (
            _adjust_voltronic_crc_byte((crc >> 8) & 0xFF),
            _adjust_voltronic_crc_byte(crc & 0xFF),
        )
    )
    ascii_payload = command_bytes + crc_bytes + b"\r"
    body = b"\xff\x04" + ascii_payload
    return (
        transaction_id.to_bytes(2, "big")
        + dev_code.to_bytes(2, "big")
        + len(body).to_bytes(2, "big")
        + body
    )


def decode_easun_response(frame: bytes, expected_transaction_id: int) -> str:
    if len(frame) < 12:
        raise InverterProtocolError(f"EASUN response is too short: {len(frame)} bytes")
    transaction_id = int.from_bytes(frame[0:2], "big")
    if transaction_id != expected_transaction_id:
        raise InverterProtocolError(
            f"transaction mismatch: expected {expected_transaction_id}, got {transaction_id}"
        )
    if frame[6:8] != b"\xff\x04":
        raise InverterProtocolError(f"unexpected EASUN wrapper: {frame[6:8].hex()}")
    payload = frame[8:]
    if len(payload) < 4 or payload[-1] != 0x0D:
        raise InverterProtocolError("EASUN response has no CRC/terminator")
    body = payload[:-3]
    received_crc = payload[-3:-1]
    crc = crc16_xmodem(body)
    expected_crc = bytes(
        (
            _adjust_voltronic_crc_byte((crc >> 8) & 0xFF),
            _adjust_voltronic_crc_byte(crc & 0xFF),
        )
    )
    if received_crc != expected_crc:
        raise InverterProtocolError(
            f"EASUN CRC mismatch: expected {expected_crc.hex()}, got {received_crc.hex()}"
        )
    try:
        text = body.decode("ascii")
    except UnicodeDecodeError as exc:
        raise InverterProtocolError("EASUN response is not ASCII") from exc
    return text[1:] if text.startswith("(") else text


def parse_easun_qpigs(text: str) -> dict[str, Any]:
    fields = text.split()
    if len(fields) < 20:
        raise InverterProtocolError(f"QPIGS returned only {len(fields)} fields: {text!r}")
    try:
        grid_voltage = float(fields[0])
        grid_frequency = float(fields[1])
        grid_absent = grid_voltage == 0.0 and grid_frequency == 0.0
        battery_voltage = float(fields[8])
        battery_charge_current = float(fields[9])
        battery_discharge_current = float(fields[15])
        battery_current = battery_charge_current - battery_discharge_current
        result: dict[str, Any] = {
            "protocol": "EASUN_VOLTRONIC_QPIGS",
            "grid_voltage_v": grid_voltage,
            "grid_frequency_hz": grid_frequency,
            # PI30 does not report AC-input current or active power. Zero is
            # nevertheless exact when both input voltage and frequency are
            # zero; with live grid input, leave the value unavailable rather
            # than presenting an energy-balance estimate as a measurement.
            "grid_power_w": 0.0 if grid_absent else None,
            "grid_power_source": "no_ac_input" if grid_absent else "unavailable_pi30",
            "output_voltage_v": float(fields[2]),
            "output_frequency_hz": float(fields[3]),
            "output_apparent_power_va": int(fields[4]),
            "output_power_w": int(fields[5]),
            "load_pct": int(fields[6]),
            "bus_voltage_v": float(fields[7]),
            "battery_voltage_v": battery_voltage,
            "battery_charge_current_a": battery_charge_current,
            "battery_discharge_current_a": battery_discharge_current,
            "battery_current_a": battery_current,
            "battery_power_w": round(battery_voltage * battery_current, 3),
            "battery_soc_pct": int(fields[10]),
            "inverter_temperature_c": float(fields[11]),
            "pv_current_a": float(fields[12]),
            "pv_voltage_v": float(fields[13]),
            "battery_voltage_scc_v": float(fields[14]),
            "device_status_bits": fields[16],
            "battery_voltage_offset": fields[17],
            "eeprom_version": fields[18],
            "pv_charging_power_w": int(fields[19]),
        }
    except (ValueError, IndexError) as exc:
        raise InverterProtocolError(f"invalid numeric value in QPIGS: {text!r}") from exc
    result["pv_power_w"] = result["pv_charging_power_w"]
    if len(fields) > 20:
        result["device_status_bits_2"] = fields[20]
    return result


def _query_easun(
    connection: socket.socket,
    command: str,
    transaction_id: int,
    dev_code: int,
) -> tuple[str, str]:
    request = build_easun_request(command, transaction_id, dev_code)
    connection.sendall(request)
    frame = receive_wrapped_frame(connection)
    return decode_easun_response(frame, transaction_id), frame.hex()


def read_easun(
    inverter_ip: str,
    local_ip: str,
    local_port: int,
    timeout_seconds: float,
) -> dict[str, Any]:
    with reverse_tunnel(inverter_ip, local_ip, local_port, timeout_seconds) as (
        connection,
        dev_code,
    ):
        qpigs, qpigs_frame = _query_easun(connection, "QPIGS", 1, dev_code)
        result = parse_easun_qpigs(qpigs)
        raw: dict[str, Any] = {
            "dev_code": dev_code,
            "responses": {"QPIGS": qpigs},
            "frames_hex": {"QPIGS": qpigs_frame},
        }
        optional_errors: dict[str, str] = {}
        # Live collection intentionally excludes QPIRI: that command returns
        # ratings and configuration rather than operational telemetry.  The
        # guarded control path queries it separately when configuration is
        # explicitly requested by the operator.
        for transaction_id, command in enumerate(
            ("QMOD", "QPIWS", "QPIGS2"), start=2
        ):
            try:
                response, frame_hex = _query_easun(
                    connection, command, transaction_id, dev_code
                )
            except (OSError, TimeoutError, InverterProtocolError) as exc:
                optional_errors[command] = f"{type(exc).__name__}: {exc}"
                break
            raw["responses"][command] = response
            raw["frames_hex"][command] = frame_hex
            if response.startswith("NAK"):
                optional_errors[command] = response
                continue
            if command == "QMOD":
                mode_code = response.strip()
                result["working_mode_code"] = mode_code
                result["working_mode"] = {
                    "P": "POWER_ON",
                    "S": "STANDBY",
                    "L": "LINE",
                    "B": "BATTERY",
                    "F": "FAULT",
                    "H": "POWER_SAVING",
                }.get(mode_code, f"UNKNOWN_{mode_code}")
            elif command == "QPIWS":
                result["warning_bits"] = response.strip()
            elif command == "QPIGS2":
                fields = response.split()
                if len(fields) >= 3:
                    try:
                        result["pv2_current_a"] = float(fields[0])
                        result["pv2_voltage_v"] = float(fields[1])
                        result["pv2_power_w"] = int(float(fields[2]))
                    except ValueError:
                        optional_errors[command] = f"invalid QPIGS2 response: {response}"
        if optional_errors:
            raw["optional_errors"] = optional_errors
        result["raw"] = raw
        return result


EASUN_FLAG_SPECS: tuple[dict[str, Any], ...] = (
    {"key": "buzzer", "label": "Buzzer", "flag": "a"},
    {"key": "overload_bypass", "label": "Transfer to bypass on overload", "flag": "b", "critical": True},
    {"key": "power_saving", "label": "Power saving", "flag": "j", "critical": True},
    {"key": "lcd_auto_return", "label": "LCD auto-return", "flag": "k"},
    {"key": "overload_restart", "label": "Restart after overload", "flag": "u"},
    {"key": "overtemperature_restart", "label": "Restart after overtemperature", "flag": "v"},
    {"key": "lcd_backlight", "label": "LCD backlight", "flag": "x"},
    {"key": "source_interrupt_alarm", "label": "Alarm on source interruption", "flag": "y"},
    {"key": "record_fault_codes", "label": "Record fault codes", "flag": "z"},
)

EASUN_CONFIGURATION_SPECS: tuple[dict[str, Any], ...] = (
    {"key": "output_frequency", "label": "Output frequency", "group": "AC output", "field": 3,
     "unit": "Hz", "options": _options((50, "50 Hz"), (60, "60 Hz")), "command": "frequency", "critical": True},
    {"key": "battery_recharge_voltage", "label": "Transfer from battery to utility", "group": "Battery and charging", "field": 8,
     "unit": "V", "minimum_factor": 0.88, "maximum_factor": 1.08, "step": 0.1, "command": "PBCV", "voltage": True, "critical": True},
    {"key": "battery_cutoff_voltage", "label": "Battery cutoff voltage", "group": "Battery and charging", "field": 9,
     "unit": "V", "minimum_factor": 0.80, "maximum_factor": 1.0, "step": 0.1, "command": "PSDV", "voltage": True, "critical": True},
    {"key": "bulk_charge_voltage", "label": "Bulk / constant-voltage charge", "group": "Battery and charging", "field": 10,
     "unit": "V", "minimum_factor": 1.0, "maximum_factor": 1.22, "step": 0.1, "command": "PCVV", "voltage": True, "critical": True},
    {"key": "float_charge_voltage", "label": "Float charge voltage", "group": "Battery and charging", "field": 11,
     "unit": "V", "minimum_factor": 1.0, "maximum_factor": 1.22, "step": 0.1, "command": "PBFT", "voltage": True, "critical": True},
    {"key": "battery_type", "label": "Battery type", "group": "Battery and charging", "field": 12,
     "options": _options((0, "AGM"), (1, "Flooded"), (2, "User-defined"),
                         (8, "Lithium battery (LIB)")), "command": "PBT",
     "description": ("Changing battery type can reset dependent charge parameters. "
                     "LIB is the lithium profile without BMS communication; configure "
                     "the inverter's BMS protocol separately."), "critical": True},
    {"key": "maximum_utility_charge_current", "label": "Maximum utility charge current", "group": "Battery and charging", "field": 13,
     "unit": "A", "command": "MUCHGC", "supported_values": "QMUCHGCR", "critical": True},
    {"key": "maximum_charge_current", "label": "Maximum total charge current", "group": "Battery and charging", "field": 14,
     "unit": "A", "command": "MCHGC", "supported_values": "QMCHGCR", "critical": True},
    {"key": "ac_input_range", "label": "AC input range", "group": "Operating modes", "field": 15,
     "options": _options((0, "Appliance"), (1, "UPS")), "command": "PGR", "critical": True},
    {"key": "output_source_priority", "label": "Output source priority", "group": "Operating modes", "field": 16,
     "options": _options((0, "Utility first"), (1, "Solar first"), (2, "Solar → battery → utility (SBU)")),
     "command": "POP", "critical": True},
    {"key": "charger_source_priority", "label": "Charger source priority", "group": "Operating modes", "field": 17,
     "options": _options((0, "Utility first"), (1, "Solar first"), (2, "Solar and utility"), (3, "Solar only")),
     "command": "PCP", "critical": True},
    {"key": "output_mode", "label": "Output / parallel mode", "group": "Advanced", "field": 21,
     "writable": False, "description": "Read-only here because PI30 requires a unit address for parallel-mode writes."},
    {"key": "battery_redischarge_voltage", "label": "Transfer from utility back to battery", "group": "Battery and charging", "field": 22,
     "unit": "V", "minimum_factor": 0, "maximum_factor": 1.22, "step": 0.1, "command": "PBDV", "voltage": True,
     "critical": True, "description": "Zero means return when charging reaches float mode."},
    {"key": "pv_ok_condition", "label": "Parallel PV-present condition", "group": "Advanced", "field": 23,
     "options": _options((0, "Any parallel unit has PV"), (1, "All parallel units have PV")), "command": "PPVOKC", "critical": True},
    {"key": "pv_power_balance", "label": "PV power balance", "group": "Advanced", "field": 24,
     "options": _options((0, "Limit PV by maximum charge current"), (1, "Allow charge power plus load power")),
     "command": "PSPB", "critical": True},
)


def _parse_pi30_integer(value: str) -> int:
    cleaned = value.strip()
    if re.fullmatch(r"\d{2}P", cleaned):
        cleaned = cleaned[:-1] + "0"
    if not cleaned.isdigit():
        raise InverterProtocolError(f"invalid PI30 integer value: {value!r}")
    return int(cleaned)


def _parse_easun_flags(value: str) -> dict[str, bool]:
    enabled, separator, disabled = value.strip().partition("D")
    if enabled.startswith("E"):
        enabled = enabled[1:]
    if not separator:
        raise InverterProtocolError(f"invalid QFLAG response: {value!r}")
    return {spec["flag"]: spec["flag"] in enabled for spec in EASUN_FLAG_SPECS} | {
        "_unknown_disabled": bool(set(disabled) - {spec["flag"] for spec in EASUN_FLAG_SPECS})
    }


def parse_easun_configuration(
    responses: dict[str, str],
    dev_code: int = 0,
) -> dict[str, Any]:
    """Build a UI-safe configuration model from supported PI30 queries."""
    fields = responses.get("QPIRI", "").split()
    if len(fields) < 25:
        raise InverterProtocolError(f"QPIRI returned only {len(fields)} fields")
    try:
        battery_rating = float(fields[7])
    except ValueError as exc:
        raise InverterProtocolError(f"invalid QPIRI battery rating: {fields[7]!r}") from exc
    if battery_rating <= 0:
        raise InverterProtocolError("QPIRI returned an invalid battery rating")
    voltage_multiplier = battery_rating / 12.0
    flags = _parse_easun_flags(responses.get("QFLAG", ""))
    supported_currents = {
        command: [_parse_pi30_integer(value) for value in responses.get(command, "").split()]
        for command in ("QMCHGCR", "QMUCHGCR")
    }

    values: dict[str, Any] = {}
    groups: dict[str, list[dict[str, Any]]] = {}
    raw_inconsistencies: dict[str, str] = {}
    for spec in EASUN_CONFIGURATION_SPECS:
        raw_text = fields[int(spec["field"])]
        writable = bool(spec.get("writable", True))
        if spec.get("options"):
            try:
                parsed_option = float(raw_text)
                value = int(parsed_option) if parsed_option.is_integer() else parsed_option
            except ValueError:
                value = _parse_pi30_integer(raw_text)
        else:
            try:
                value = float(raw_text)
            except ValueError:
                value = _parse_pi30_integer(raw_text)
        if spec.get("voltage"):
            if value != 0 and value < battery_rating * 0.75:
                value = round(float(value) * voltage_multiplier, 3)
            if value > battery_rating * 1.35:
                raw_inconsistencies[str(spec["key"])] = (
                    f"QPIRI reports {raw_text} V, which is not plausible for a {battery_rating:g} V inverter"
                )
                writable = False
        values[str(spec["key"])] = value
        item = {key: content for key, content in spec.items()
                if key not in {"group", "field", "command", "supported_values", "voltage",
                               "minimum_factor", "maximum_factor"}}
        if spec.get("supported_values"):
            choices = supported_currents[str(spec["supported_values"])]
            item["options"] = _options(*[(choice, f"{choice} A") for choice in choices])
        if "minimum_factor" in spec:
            item["minimum"] = round(battery_rating * float(spec["minimum_factor"]), 1)
            item["maximum"] = round(battery_rating * float(spec["maximum_factor"]), 1)
        item.update({"type": "select" if item.get("options") else "number", "value": value,
                     "raw_value": raw_text, "writable": writable})
        if not writable and str(spec["key"]) in raw_inconsistencies:
            item["description"] = raw_inconsistencies[str(spec["key"])] + "; writing is disabled."
        groups.setdefault(str(spec["group"]), []).append(item)

    for spec in EASUN_FLAG_SPECS:
        value = flags[str(spec["flag"])]
        values[str(spec["key"])] = value
        item = {key: content for key, content in spec.items() if key != "flag"}
        item.update({"type": "select", "value": value, "raw_value": int(value), "writable": True,
                     "options": _options((False, "Disabled"), (True, "Enabled"))})
        groups.setdefault("Alarms and behavior", []).append(item)

    return {
        "protocol": "easun_qpigs",
        "identity": {
            "serial": responses.get("QID", "").strip(),
            "protocol": responses.get("QPI", "PI30").strip(),
            "firmware": responses.get("QVFW", "").removeprefix("VERFW:").strip(),
        },
        "ratings": {
            "grid_voltage_v": float(fields[0]), "grid_current_a": float(fields[1]),
            "output_voltage_v": float(fields[2]), "output_frequency_hz": float(fields[3]),
            "output_current_a": float(fields[4]), "output_apparent_power_va": int(fields[5]),
            "output_power_w": int(fields[6]), "battery_voltage_v": battery_rating,
        },
        "groups": [{"id": re.sub(r"[^a-z0-9]+", "-", name.lower()).strip("-"),
                    "title": name, "settings": items} for name, items in groups.items()],
        "values": values,
        "raw": {"dev_code": dev_code, "responses": responses,
                "inconsistencies": raw_inconsistencies},
    }


def _easun_configuration_from_connection(
    connection: socket.socket,
    dev_code: int,
    first_transaction_id: int = 1,
) -> tuple[dict[str, Any], int]:
    responses: dict[str, str] = {}
    transaction_id = first_transaction_id
    for command in ("QPI", "QID", "QVFW", "QPIRI", "QFLAG", "QMCHGCR", "QMUCHGCR"):
        response, _ = _query_easun(connection, command, transaction_id, dev_code)
        if response.startswith("NAK"):
            raise InverterProtocolError(f"EASUN rejected required query {command}")
        responses[command] = response
        transaction_id += 1
    return parse_easun_configuration(responses, dev_code), transaction_id


def read_easun_configuration(
    inverter_ip: str,
    local_ip: str,
    local_port: int,
    timeout_seconds: float,
) -> dict[str, Any]:
    with reverse_tunnel(inverter_ip, local_ip, local_port, timeout_seconds) as (connection, dev_code):
        configuration, _ = _easun_configuration_from_connection(connection, dev_code)
        return configuration


READERS = {
    "anenji_modbus": read_anenji,
    "easun_qpigs": read_easun,
}

CONFIGURATION_READERS = {
    "anenji_modbus": read_anenji_configuration,
    "easun_qpigs": read_easun_configuration,
}


def _same_value(left: Any, right: Any) -> bool:
    if isinstance(left, bool) or isinstance(right, bool):
        return left is right
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return math.isclose(float(left), float(right), rel_tol=0, abs_tol=1e-6)
    return left == right


def _coerce_setting_value(spec: dict[str, Any], requested: Any) -> Any:
    options = spec.get("options")
    if options:
        for option in options:
            candidate = option["value"]
            if isinstance(candidate, bool):
                if requested is candidate or str(requested).lower() == str(candidate).lower():
                    return candidate
            elif str(requested) == str(candidate):
                return candidate
        raise ValueError(f"unsupported value for {spec['label']}")

    try:
        if spec.get("format") == "hex" or spec.get("input_type") == "text":
            value: int | float = int(str(requested).strip(), 0)
        else:
            value = float(requested)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{spec['label']} must be numeric") from exc
    if not math.isfinite(float(value)):
        raise ValueError(f"{spec['label']} must be finite")
    minimum = spec.get("minimum")
    maximum = spec.get("maximum")
    if minimum is not None and value < minimum:
        raise ValueError(f"{spec['label']} must be at least {minimum}")
    if maximum is not None and value > maximum:
        raise ValueError(f"{spec['label']} must be at most {maximum}")
    step = spec.get("step")
    if step:
        origin = float(minimum or 0)
        units = (float(value) - origin) / float(step)
        if not math.isclose(units, round(units), rel_tol=0, abs_tol=1e-6):
            raise ValueError(f"{spec['label']} must use increments of {step}")
    return int(value) if float(value).is_integer() else round(float(value), 6)


def _configuration_item(configuration: dict[str, Any], key: str) -> dict[str, Any]:
    for group in configuration["groups"]:
        for item in group["settings"]:
            if item["key"] == key:
                return item
    raise ValueError(f"unsupported inverter setting: {key}")


def _validate_anenji_dependencies(values: dict[str, Any]) -> None:
    overvoltage = float(values["battery_overvoltage"])
    maximum = float(values["maximum_charge_voltage"])
    float_voltage = float(values["float_charge_voltage"])
    recovery = float(values["utility_discharge_recovery_voltage"])
    utility_low = float(values["utility_low_battery_voltage"])
    offgrid_low = float(values["offgrid_low_battery_voltage"])
    op1_low = float(values["offgrid_op1_low_battery_voltage"])
    if maximum > overvoltage - 1:
        raise ValueError("maximum charge voltage must be at least 1 V below overvoltage protection")
    if float_voltage > maximum:
        raise ValueError("float voltage cannot exceed maximum charge voltage")
    if recovery and not utility_low < recovery <= maximum - 0.5:
        raise ValueError("utility recovery voltage must be above the utility low limit and below charge voltage")
    if op1_low > 0 and not offgrid_low <= op1_low < utility_low:
        raise ValueError("off-grid voltage thresholds must satisfy cutoff ≤ OP1 < transfer-to-utility")
    if float(values["maximum_utility_charge_current"]) > float(values["maximum_charge_current"]):
        raise ValueError("maximum utility charge current cannot exceed maximum total charge current")
    if not float_voltage <= float(values["equalization_voltage"]) <= overvoltage - 1:
        raise ValueError("equalization voltage must be between float and overvoltage-minus-1 V")
    soc_keys = ("offgrid_discharge_soc", "offgrid_op1_discharge_soc",
                "utility_discharge_soc", "utility_discharge_recovery_soc",
                "lithium_stop_charge_soc")
    if all(0 <= int(values[key]) <= 100 for key in soc_keys):
        if not int(values["offgrid_discharge_soc"]) <= int(values["offgrid_op1_discharge_soc"]) < int(values["utility_discharge_soc"]):
            raise ValueError("SOC limits must satisfy off-grid ≤ OP1 < utility-mode limit")
        if not int(values["utility_discharge_soc"]) < int(values["utility_discharge_recovery_soc"]):
            raise ValueError("utility-mode recovery SOC must exceed its discharge limit")
        if int(values["lithium_stop_charge_soc"]) < int(values["utility_discharge_recovery_soc"]):
            raise ValueError("lithium stop-charge SOC cannot be below utility recovery SOC")


def _validate_easun_dependencies(values: dict[str, Any]) -> None:
    cutoff = float(values["battery_cutoff_voltage"])
    recharge = float(values["battery_recharge_voltage"])
    bulk = float(values["bulk_charge_voltage"])
    float_voltage = float(values["float_charge_voltage"])
    if not cutoff < recharge < bulk:
        raise ValueError("battery voltages must satisfy cutoff < transfer-to-utility < bulk charge")
    if float_voltage > bulk:
        raise ValueError("float voltage cannot exceed bulk charge voltage")
    if int(values["maximum_utility_charge_current"]) > int(values["maximum_charge_current"]):
        raise ValueError("maximum utility charge current cannot exceed maximum total charge current")


def build_easun_setting_command(
    key: str,
    value: Any,
    configuration: dict[str, Any],
) -> str:
    """Validate a PI30 setting and produce its documented setter command."""
    item = _configuration_item(configuration, key)
    if not item.get("writable") or item.get("type") == "action":
        raise ValueError(f"{item['label']} is not writable")
    spec = next((entry for entry in EASUN_CONFIGURATION_SPECS if entry["key"] == key), None)
    flag_spec = next((entry for entry in EASUN_FLAG_SPECS if entry["key"] == key), None)
    normalized = _coerce_setting_value(item, value)
    proposed = dict(configuration["values"])
    proposed[key] = normalized
    _validate_easun_dependencies(proposed)
    if flag_spec is not None:
        return ("PE" if normalized else "PD") + str(flag_spec["flag"])
    if spec is None:
        raise ValueError(f"unsupported EASUN setting: {key}")
    prefix = str(spec["command"])
    if prefix == "frequency":
        return f"F{int(normalized):02d}"
    if prefix in {"PBCV", "PSDV", "PCVV", "PBFT", "PBDV"}:
        return f"{prefix}{float(normalized):04.1f}"
    if prefix in {"MCHGC", "MUCHGC"}:
        return f"{prefix}{int(normalized):03d}"
    if prefix in {"PBT", "PGR", "POP", "PCP"}:
        return f"{prefix}{int(normalized):02d}"
    if prefix in {"PPVOKC", "PSPB"}:
        return f"{prefix}{int(normalized)}"
    raise ValueError(f"no safe command formatter for {key}")


def _write_anenji_setting(
    inverter_ip: str,
    local_ip: str,
    local_port: int,
    timeout_seconds: float,
    key: str,
    requested: Any,
    confirmation: str,
    force: bool,
) -> dict[str, Any]:
    with reverse_tunnel(inverter_ip, local_ip, local_port, timeout_seconds) as (connection, dev_code):
        before_configuration, transaction_id = _anenji_configuration_from_connection(connection, dev_code)
        serial = before_configuration["identity"]["serial"]
        if not serial or confirmation != serial:
            raise ValueError("confirmation must exactly match the inverter serial number")
        item = _configuration_item(before_configuration, key)
        if not item.get("writable"):
            raise ValueError(f"{item['label']} is not writable")
        action_spec = next((spec for spec in ANENJI_ACTION_SPECS if spec["key"] == key), None)
        if action_spec is not None:
            normalized = int(action_spec["write_value"])
            address = int(action_spec["address"])
            encoded = [normalized]
            before_value = None
        else:
            normalized = _coerce_setting_value(item, requested)
            proposed = dict(before_configuration["values"])
            proposed[key] = normalized
            _validate_anenji_dependencies(proposed)
            spec = next(entry for entry in ANENJI_CONFIGURATION_SPECS if entry["key"] == key)
            address = int(spec["address"])
            scale = int(spec.get("scale", 1))
            raw = int(round(float(normalized) * scale))
            count = int(spec.get("count", 1))
            encoded = [(raw >> 16) & 0xFFFF, raw & 0xFFFF] if count == 2 else [raw]
            before_value = before_configuration["values"][key]
            if _same_value(before_value, normalized) and not force:
                return {"changed": False, "written": False, "verified": True,
                        "setting": key, "before": before_value, "after": before_value,
                        "before_configuration": before_configuration,
                        "after_configuration": before_configuration}

        connection.sendall(build_anenji_write_request(transaction_id, dev_code, address, encoded))
        parse_anenji_write_response(receive_wrapped_frame(connection), transaction_id, address, len(encoded))
        after_configuration, _ = _anenji_configuration_from_connection(connection, dev_code, transaction_id + 1)
        after_value = None if action_spec is not None else after_configuration["values"][key]
        if action_spec is None and not _same_value(after_value, normalized):
            raise InverterProtocolError(
                f"read-back mismatch for {item['label']}: requested {normalized}, got {after_value}"
            )
        return {"changed": action_spec is not None or not _same_value(before_value, after_value),
                "written": True, "verified": action_spec is None,
                "setting": key, "before": before_value, "after": after_value,
                "before_configuration": before_configuration,
                "after_configuration": after_configuration}


def _write_easun_setting(
    inverter_ip: str,
    local_ip: str,
    local_port: int,
    timeout_seconds: float,
    key: str,
    requested: Any,
    confirmation: str,
    force: bool,
) -> dict[str, Any]:
    with reverse_tunnel(inverter_ip, local_ip, local_port, timeout_seconds) as (connection, dev_code):
        before_configuration, transaction_id = _easun_configuration_from_connection(connection, dev_code)
        serial = before_configuration["identity"]["serial"]
        if not serial or confirmation != serial:
            raise ValueError("confirmation must exactly match the inverter serial number")
        item = _configuration_item(before_configuration, key)
        normalized = _coerce_setting_value(item, requested)
        command = build_easun_setting_command(key, normalized, before_configuration)
        before_value = before_configuration["values"][key]
        if _same_value(before_value, normalized) and not force:
            return {"changed": False, "written": False, "verified": True,
                    "setting": key, "before": before_value, "after": before_value,
                    "before_configuration": before_configuration,
                    "after_configuration": before_configuration}
        response, _ = _query_easun(connection, command, transaction_id, dev_code)
        if response.strip() != "ACK":
            raise InverterProtocolError(f"EASUN rejected {item['label']}: {response}")
        after_configuration, _ = _easun_configuration_from_connection(connection, dev_code, transaction_id + 1)
        after_value = after_configuration["values"][key]
        if not _same_value(after_value, normalized):
            raise InverterProtocolError(
                f"read-back mismatch for {item['label']}: requested {normalized}, got {after_value}"
            )
        return {"changed": not _same_value(before_value, after_value), "written": True,
                "verified": True, "setting": key, "before": before_value, "after": after_value,
                "before_configuration": before_configuration,
                "after_configuration": after_configuration}


CONFIGURATION_WRITERS = {
    "anenji_modbus": _write_anenji_setting,
    "easun_qpigs": _write_easun_setting,
}


def read_inverter(
    protocol: str,
    inverter_ip: str,
    local_ip: str,
    local_port: int,
    timeout_seconds: float,
) -> dict[str, Any]:
    try:
        reader = READERS[protocol]
    except KeyError as exc:
        raise ValueError(f"unsupported inverter protocol: {protocol}") from exc
    with inverter_network_lock():
        return reader(inverter_ip, local_ip, local_port, timeout_seconds)


def read_inverter_configuration(
    protocol: str,
    inverter_ip: str,
    local_ip: str,
    local_port: int,
    timeout_seconds: float,
) -> dict[str, Any]:
    try:
        reader = CONFIGURATION_READERS[protocol]
    except KeyError as exc:
        raise ValueError(f"unsupported inverter protocol: {protocol}") from exc
    with inverter_network_lock():
        return reader(inverter_ip, local_ip, local_port, timeout_seconds)


def write_inverter_setting(
    protocol: str,
    inverter_ip: str,
    local_ip: str,
    local_port: int,
    timeout_seconds: float,
    key: str,
    value: Any,
    confirmation: str,
    force: bool = False,
) -> dict[str, Any]:
    try:
        writer = CONFIGURATION_WRITERS[protocol]
    except KeyError as exc:
        raise ValueError(f"unsupported inverter protocol: {protocol}") from exc
    with inverter_network_lock():
        return writer(
            inverter_ip, local_ip, local_port, timeout_seconds,
            key, value, confirmation, force,
        )
