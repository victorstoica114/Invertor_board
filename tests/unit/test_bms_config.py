import asyncio

from tools import bms_config


def test_numeric_codec_respects_jk_storage_widths():
    signed_byte = bms_config.SettingSpec(
        "temperature", "Temperature", "temperature", 0, signed=True, bits=8, length=1
    )
    signed_word = bms_config.SettingSpec(
        "temperature", "Temperature", "temperature", 0, signed=True, bits=32
    )

    assert bms_config._decode_number(signed_byte, 0xF6) == -10
    assert bms_config._encode_number(signed_byte, -10) == 0xF6
    assert bms_config._decode_number(signed_word, 0xFFFFFF9C) == -100
    assert bms_config._encode_number(signed_word, -100) == 0xFFFFFF9C


def test_configuration_maps_cover_verified_live_register_blocks():
    daly_registers = {spec.register for spec in bms_config.DALY_SPECS}
    seplos_registers = {spec.register for spec in bms_config.SEPLOS_REGISTER_SPECS}
    jk_keys = {spec.key for spec in bms_config.JK_NUMBER_SPECS}

    assert daly_registers == {*range(0x0080, 0x00A9), 0x00CF}
    assert seplos_registers == set(range(0x1301, 0x1368)) - {0x1328, 0x1329, 0x132A}
    assert {"smart_sleep_voltage", "charge_undertemperature", "cell_count"} <= jk_keys
    smart_sleep = next(spec for spec in bms_config.JK_NUMBER_SPECS if spec.key == "smart_sleep_voltage")
    assert (smart_sleep.length, smart_sleep.bits) == (1, 32)


def test_select_codec_rejects_values_outside_the_device_map():
    battery_type = next(spec for spec in bms_config.DALY_SPECS if spec.key == "battery_type")
    assert bms_config._encode_number(battery_type, "2") == 2
    try:
        bms_config._encode_number(battery_type, 9)
    except ValueError as exc:
        assert "supported options" in str(exc)
    else:
        raise AssertionError("an unsupported selector must be rejected")


def test_daly_session_reassembles_fragmented_read_response():
    class Client:
        async def write_gatt_char(self, _uuid, _frame, response=False):
            assert response is False

    async def exercise():
        session = bms_config._DalySession(Client())
        frame = bytearray((0xD2, 0x03, 4, 0x12, 0x34, 0x56, 0x78))
        frame.extend(bms_config.bms_ble.crc16_modbus(frame).to_bytes(2, "little"))
        task = asyncio.create_task(session.command(0x03, 0x0080, 2))
        await asyncio.sleep(0)
        session.notification(None, frame[:5])
        assert not session.event.is_set()
        session.notification(None, frame[5:])
        assert await task == bytes(frame)

    asyncio.run(exercise())


def test_seplos_single_coil_write_uses_modbus_function_0f():
    frame = bms_config._build_modbus_write_coil(0, 0x1400, True)
    assert frame[:8] == bytes.fromhex("000f140000010101")
    assert bms_config.bms_ble.crc16_modbus(frame[:-2]) == int.from_bytes(frame[-2:], "little")
