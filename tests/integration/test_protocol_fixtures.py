"""Validate protocol fixtures used by integration and host tests."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fixtures.protocol_samples import (
    GROWATT_CAN_FRAME_SAMPLES,
    JKBMS_MODBUS_SAMPLES,
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
