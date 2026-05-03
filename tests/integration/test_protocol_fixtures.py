"""Validate protocol fixtures used by integration and host tests."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fixtures.protocol_samples import (
    GROWATT_CAN_FRAME_SAMPLES,
    JKBMS_MODBUS_SAMPLES,
    PACE_MODBUS_V13_SAMPLE,
    PYLON_CAN_FRAME_SAMPLES,
    calculate_modbus_crc,
    get_can_frame_dict,
    get_modbus_frame_with_crc,
)


def test_modbus_crc_known_request_vector() -> None:
    request_without_crc = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x0A])
    assert calculate_modbus_crc(request_without_crc) == 0xCDC5


def test_modbus_fixture_responses_can_be_completed_with_valid_crc() -> None:
    for sample in JKBMS_MODBUS_SAMPLES:
        response_without_crc = sample["raw_response"][:-2]
        frame = get_modbus_frame_with_crc(
            sample["slave_id"],
            sample["function"],
            response_without_crc[2:],
        )

        assert frame[:-2] == response_without_crc
        assert calculate_modbus_crc(frame[:-2]) == frame[-2] | (frame[-1] << 8)


def test_can_fixture_shape_matches_twai_messages() -> None:
    all_samples = [*GROWATT_CAN_FRAME_SAMPLES, *PYLON_CAN_FRAME_SAMPLES]

    for sample in all_samples:
        frame = get_can_frame_dict(sample)

        assert 0 <= frame["identifier"] <= 0x7FF
        assert frame["data_length_code"] == len(frame["data"])
        assert 0 <= frame["data_length_code"] <= 8
        assert all(0 <= byte <= 0xFF for byte in frame["data"])


def test_growatt_fixture_soc_is_encoded_where_decoder_reads_it() -> None:
    for sample in GROWATT_CAN_FRAME_SAMPLES:
        frame = get_can_frame_dict(sample)

        assert frame["identifier"] == 0x313
        assert frame["data"][6] == sample["expected_soc"]


def test_pylon_fixture_contains_soc_and_pack_frames() -> None:
    frames_by_id = {sample["id"]: sample for sample in PYLON_CAN_FRAME_SAMPLES}

    assert 0x355 in frames_by_id
    assert 0x356 in frames_by_id
    assert frames_by_id[0x355]["data"][0] == frames_by_id[0x355]["expected_soc"]


def test_pace_v13_fixture_shape_matches_register_map() -> None:
    sample = PACE_MODBUS_V13_SAMPLE
    summary = sample["summary_registers"]
    cells_and_temps = sample["cells_and_temps"]
    cells = cells_and_temps[:16]
    temps = cells_and_temps[16:]

    assert sample["summary_start"] == 0x0000
    assert len(summary) == 13
    assert summary[1] == sample["expected_pack_voltage_cv"]
    assert summary[2] == sample["expected_soc"]
    assert summary[9] == sample["expected_warning_flags"]
    assert summary[10] == sample["expected_protection_flags"]
    assert summary[11] == sample["expected_status_flags"]

    assert sample["cells_start"] == 0x000F
    assert len(cells) == 16
    assert len(temps) == 6
    assert max(cells) == sample["expected_cell_max_mv"]
    assert cells[sample["expected_cell_max_idx"] - 1] == sample["expected_cell_max_mv"]
    assert min(cells) == sample["expected_cell_min_mv"]
    assert cells[sample["expected_cell_min_idx"] - 1] == sample["expected_cell_min_mv"]

    expected_temps = sample["expected_temp_deci_c"]
    assert temps == [
        expected_temps["battery_t1"],
        expected_temps["battery_t2"],
        expected_temps["battery_t4"],
        expected_temps["battery_t5"],
        expected_temps["mos"],
        expected_temps["environment"],
    ]


def test_pace_v13_fixture_can_build_valid_modbus_responses() -> None:
    sample = PACE_MODBUS_V13_SAMPLE

    for registers in (sample["summary_registers"], sample["cells_and_temps"]):
        data = bytes([len(registers) * 2])
        for value in registers:
            data += bytes([(value >> 8) & 0xFF, value & 0xFF])

        frame = get_modbus_frame_with_crc(sample["slave_id"], sample["function"], data)

        assert frame[0] == sample["slave_id"]
        assert frame[1] == sample["function"]
        assert frame[2] == len(registers) * 2
        assert calculate_modbus_crc(frame[:-2]) == frame[-2] | (frame[-1] << 8)
