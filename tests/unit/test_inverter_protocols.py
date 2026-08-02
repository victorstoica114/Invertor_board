import contextlib
import unittest
from unittest import mock

from tools import inverter_protocols as protocols


ANENJI_LIVE_FRAME = bytes.fromhex(
    "00010001004bff04010344000300000000000009070014138e0029000009070009"
    "138e001c00cf021c0000000012a50f9e00000000000000000000000300200024"
    "001d0064000002510000000000005152"
)

EASUN_LIVE_FRAME = bytes.fromhex(
    "000100010070ff04283030302e302030302e30203233302e302035302e30203030"
    "3639203030363920303031203430312032372e3030203030302031303020303032"
    "362030303030203330302e302032372e3030203030303031203030303130313130"
    "20303020303020303030303020303130696e0d"
)


class InverterProtocolTests(unittest.TestCase):
    @staticmethod
    def frame_with_anenji_register(address, raw_value):
        frame = bytearray(ANENJI_LIVE_FRAME)
        register_offset = address - 201
        register_start = 11 + register_offset * 2
        frame[register_start : register_start + 2] = int(raw_value).to_bytes(2, "big")
        crc = protocols.crc16_modbus(frame[8:-2])
        frame[-2:] = crc.to_bytes(2, "little")
        return bytes(frame)

    def test_modbus_crc_known_vector(self):
        self.assertEqual(protocols.crc16_modbus(b"123456789"), 0x4B37)

    def test_builds_live_easun_qpigs_request(self):
        self.assertEqual(
            protocols.build_easun_request("QPIGS", 1, 1).hex(),
            "00010001000aff045150494753b7a90d",
        )

    def test_builds_documented_anenji_write_request_and_validates_ack(self):
        request = protocols.build_anenji_write_request(1, 1, 320, [2200])
        self.assertEqual(
            request.hex(),
            "00010001000dff04011001400001020898be3a",
        )
        acknowledgement = bytes.fromhex(
            "00010001000aff0401100140000101e1"
        )
        protocols.parse_anenji_write_response(acknowledgement, 1, 320, 1)

    def test_parses_live_anenji_register_frame(self):
        result = protocols.parse_anenji_response(ANENJI_LIVE_FRAME)

        self.assertEqual(result["protocol"], "ANENJI_MODBUS_201_234")
        self.assertEqual(result["working_mode"], "OFF_GRID")
        self.assertAlmostEqual(result["output_voltage_v"], 231.1)
        self.assertEqual(result["output_power_w"], 28)
        self.assertAlmostEqual(result["battery_voltage_v"], 54.0)
        self.assertEqual(result["battery_soc_pct"], 100)
        self.assertAlmostEqual(result["pv_voltage_v"], 399.8)
        self.assertEqual(len(result["raw"]["registers"]), 34)
        self.assertEqual(result["raw"]["registers"]["216"], 0)

    def test_splits_anenji_signed_battery_current_into_charge_and_discharge(self):
        charging = protocols.parse_anenji_response(
            self.frame_with_anenji_register(232, 7)
        )
        discharging = protocols.parse_anenji_response(
            self.frame_with_anenji_register(232, 0xFFED)
        )

        self.assertAlmostEqual(charging["battery_current_a"], 0.7)
        self.assertAlmostEqual(charging["battery_charge_current_a"], 0.7)
        self.assertEqual(charging["battery_discharge_current_a"], 0.0)
        self.assertAlmostEqual(discharging["battery_current_a"], -1.9)
        self.assertEqual(discharging["battery_charge_current_a"], 0.0)
        self.assertAlmostEqual(discharging["battery_discharge_current_a"], 1.9)

    def test_rejects_anenji_crc_corruption(self):
        damaged = ANENJI_LIVE_FRAME[:-1] + bytes((ANENJI_LIVE_FRAME[-1] ^ 1,))

        with self.assertRaisesRegex(protocols.InverterProtocolError, "CRC mismatch"):
            protocols.parse_anenji_response(damaged)

    def test_decodes_and_parses_live_easun_qpigs_frame(self):
        text = protocols.decode_easun_response(EASUN_LIVE_FRAME, 1)
        result = protocols.parse_easun_qpigs(text)

        self.assertEqual(text.split()[0], "000.0")
        self.assertEqual(result["protocol"], "EASUN_VOLTRONIC_QPIGS")
        self.assertAlmostEqual(result["output_voltage_v"], 230.0)
        self.assertEqual(result["output_power_w"], 69)
        self.assertEqual(result["grid_power_w"], 0.0)
        self.assertEqual(result["grid_power_source"], "no_ac_input")
        self.assertAlmostEqual(result["battery_voltage_v"], 27.0)
        self.assertAlmostEqual(result["battery_current_a"], -1.0)
        self.assertAlmostEqual(result["battery_power_w"], -27.0)
        self.assertEqual(result["battery_soc_pct"], 100)
        self.assertAlmostEqual(result["pv_voltage_v"], 300.0)
        self.assertEqual(result["device_status_bits_2"], "010")

        fields = text.split()
        fields[0:2] = ["230.0", "50.0"]
        grid_present = protocols.parse_easun_qpigs(" ".join(fields))
        self.assertIsNone(grid_present["grid_power_w"])
        self.assertEqual(grid_present["grid_power_source"], "unavailable_pi30")

    def test_rejects_easun_crc_corruption(self):
        damaged = EASUN_LIVE_FRAME[:-3] + b"zz\r"

        with self.assertRaisesRegex(protocols.InverterProtocolError, "CRC mismatch"):
            protocols.decode_easun_response(damaged, 1)

    def test_live_easun_reader_never_queries_configuration(self):
        qpigs = protocols.decode_easun_response(EASUN_LIVE_FRAME, 1)
        responses = {
            "QPIGS": qpigs,
            "QMOD": "B",
            "QPIWS": "00000000",
            "QPIGS2": "1.0 120.0 120",
        }
        commands = []

        def query(_connection, command, _transaction_id, _dev_code):
            commands.append(command)
            return responses[command], f"frame-{command}"

        with (
            mock.patch.object(
                protocols,
                "reverse_tunnel",
                return_value=contextlib.nullcontext((object(), 1)),
            ),
            mock.patch.object(protocols, "_query_easun", side_effect=query),
        ):
            result = protocols.read_easun("192.0.2.1", "192.0.2.2", 8899, 1)

        self.assertEqual(commands, ["QPIGS", "QMOD", "QPIWS", "QPIGS2"])
        self.assertNotIn("QPIRI", result["raw"]["responses"])
        self.assertNotIn("rating_fields", result)
        self.assertEqual(result["working_mode"], "BATTERY")
        self.assertEqual(result["pv2_power_w"], 120)

    def test_parses_easun_configuration_and_builds_safe_commands(self):
        configuration = protocols.parse_easun_configuration(
            {
                "QPI": "PI30",
                "QID": "5535535553555",
                "QVFW": "VERFW:00072.40",
                "QPIRI": (
                    "230.0 20.0 230.0 50.0 20.0 3600 3600 24.0 "
                    "11.0 10.5 14.1 13.5 2 60 06P 1 0 1 6 01 0 0 52.0 0 1"
                ),
                "QFLAG": "EakxyDbjuvz",
                "QMCHGCR": "010 020 030 040 050 060 070 080 090 100 110 120",
                "QMUCHGCR": "002 010 020 030 040 050 060",
            },
            dev_code=1,
        )

        self.assertEqual(configuration["identity"]["serial"], "5535535553555")
        self.assertAlmostEqual(configuration["values"]["battery_recharge_voltage"], 22.0)
        self.assertAlmostEqual(configuration["values"]["bulk_charge_voltage"], 28.2)
        self.assertEqual(configuration["values"]["maximum_charge_current"], 60)
        self.assertIn("battery_redischarge_voltage", configuration["raw"]["inconsistencies"])
        self.assertEqual(
            protocols.build_easun_setting_command("battery_recharge_voltage", 22.0, configuration),
            "PBCV22.0",
        )
        self.assertEqual(
            protocols.build_easun_setting_command("maximum_charge_current", 60, configuration),
            "MCHGC060",
        )
        self.assertEqual(
            protocols.build_easun_setting_command("battery_type", 8, configuration),
            "PBT08",
        )
        self.assertEqual(
            protocols.build_easun_setting_command("buzzer", True, configuration),
            "PEa",
        )
        with self.assertRaisesRegex(ValueError, "not writable"):
            protocols.build_easun_setting_command(
                "battery_redischarge_voltage", 24.0, configuration
            )


if __name__ == "__main__":
    unittest.main()
