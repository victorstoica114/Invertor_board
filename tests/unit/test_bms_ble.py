import importlib.util
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).parents[2] / "tools" / "bms_ble.py"
SPEC = importlib.util.spec_from_file_location("bms_ble", MODULE_PATH)
bms_ble = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(bms_ble)


def test_modbus_read_frame_has_expected_crc():
    assert bms_ble.build_modbus_read(0, 4, 0x1800, 0x24).hex() == "000418000024f760"


def test_modbus_write_frame_has_expected_payload_and_crc():
    assert bms_ble.build_modbus_write(0, 0x1823, (0x0900,)).hex() == "00101823000102090032c2"


def test_jk_write_frame_encodes_register_value_and_checksum():
    frame = bms_ble.build_jk_frame(0xA5, 5, 2)
    assert len(frame) == 20
    assert frame[:10] == bytes.fromhex("aa5590eba50205000000")
    assert frame[-1] == sum(frame[:-1]) & 0xFF


def test_parse_live_seplos_protocol_block():
    payload = bytes.fromhex(
        "0b002580696e76657274657220334b000000000000000000000000000000000000000000"
        "50796c6f6e206c6f7720766f6c746167652070726f746f636f6c00000000000033350000"
    )
    parsed = bms_ble.parse_seplos_protocol(payload)
    assert parsed["selector_bytes"] == [0x0B, 0x00]
    assert parsed["selector_index"] == 11
    assert parsed["selector_profile"] == "pylon_485"
    assert parsed["pre_switch_index"] == 0
    assert parsed["baud_rate"] == 9600
    assert parsed["protocol_name"] == "Pylon low voltage protocol"
    assert parsed["protocol_version"] == "35"


def test_parse_jk_protocol_fields_and_support_masks():
    frame = bytearray(300)
    frame[:5] = bytes.fromhex("55aaeb9003")
    frame[6:22] = b"JK_B1A8S20P\0\0\0\0\0"
    frame[22:25] = b"19H"
    frame[30:35] = b"19.13"
    frame[184] = 1
    frame[185] = 2
    frame[186:190] = (0x1E7FF).to_bytes(4, "little")
    frame[202:206] = (0xFFF).to_bytes(4, "little")
    frame[218] = 5
    frame[219:223] = (0xE7FF).to_bytes(4, "little")
    frame[270] = 10
    frame[-1] = sum(frame[:-1]) & 0xFF

    parsed = bms_ble.parse_jk_device_info(bytes(frame))
    assert parsed["model"] == "JK_B1A8S20P"
    assert parsed["interfaces"]["uart1"]["selected"]["name"] == "JK BMS RS485 Modbus V1.0"
    assert parsed["interfaces"]["can"]["selected"]["name"] == "PYLON-Low-voltage-V1.2"
    assert parsed["interfaces"]["uart2"]["selected"]["index"] == 5
    assert parsed["interfaces"]["uart3"]["selected"]["enabled"] is False


def test_protocol_resolution_requires_numeric_or_exact_name():
    assert bms_ble.resolve_protocol("0x5", bms_ble.UART_PROTOCOLS) == 5
    assert bms_ble.resolve_protocol("pylon_low_voltage_protocol_rs485_v3.5", bms_ble.UART_PROTOCOLS) == 5
    with pytest.raises(ValueError):
        bms_ble.resolve_protocol("pylon", bms_ble.UART_PROTOCOLS)
