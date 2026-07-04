import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "pace_modbus"
)
sys.path.insert(0, str(DECODER_DIR))

from pace_modbus import (  # noqa: E402
    describe_frame,
    describe_register,
    frame_summary,
    modbus_crc16,
    parse_frame,
)


def append_crc(payload):
    crc = modbus_crc16(payload)
    return bytes(payload + [crc & 0xFF, (crc >> 8) & 0xFF])


def build_request(slave, func, start, count):
    return append_crc([
        slave,
        func,
        (start >> 8) & 0xFF,
        start & 0xFF,
        (count >> 8) & 0xFF,
        count & 0xFF,
    ])


def build_response(slave, func, regs):
    payload = [slave, func, len(regs) * 2]
    for value in regs:
        payload.extend([(value >> 8) & 0xFF, value & 0xFF])
    return append_crc(payload)


def test_parse_pace_summary_poll_request():
    frame = parse_frame(build_request(0x01, 0x03, 0x0000, 0x000D))

    assert frame["type"] == "request"
    assert frame["slave"] == 0x01
    assert frame["func"] == 0x03
    assert frame["start"] == 0x0000
    assert frame["count"] == 13
    assert frame["crc_ok"]
    assert "start=0x0000 count=13 poll-block" in describe_frame(frame)
    assert "PACE Modbus req slave=0x01" in frame_summary(frame, "TX")


def test_parse_pace_summary_response_with_request_context():
    req = parse_frame(build_request(0x01, 0x03, 0x0000, 0x000D))
    regs = [
        0,        # 0.00 A
        5719,     # 57.19 V
        93,
        100,
        2800,     # 28.00 Ah
        4000,     # 40.00 Ah
        4000,     # 40.00 Ah design
        957,
        0,
        0,
        0,
        0x0F00,
        0,
    ]
    frame = parse_frame(build_response(0x01, 0x03, regs), req)
    decoded = describe_frame(frame)

    assert frame["type"] == "response"
    assert frame["crc_ok"]
    assert frame["registers"][0]["addr"] == 0x0000
    assert "pack_i=+0.00A" in decoded
    assert "pack_v=57.19V" in decoded
    assert "SOC=93%" in decoded
    assert "SOH=100%" in decoded
    assert "remain=28.00Ah" in decoded
    assert "full=40.00Ah" in decoded
    assert "design=40.00Ah" in decoded
    assert "cycles=957" in decoded
    assert "status=0x0F00" in decoded
    assert "charge MOS" in describe_register(0x000B, 0x0F00)


def test_parse_pace_packed_soc_soh_layout():
    req = parse_frame(build_request(0x01, 0x03, 0x0002, 0x0002))
    frame = parse_frame(build_response(0x01, 0x03, [0x5D00, 0x6400]), req)
    decoded = describe_frame(frame)

    assert "SOC=93%" in decoded
    assert "SOH=100%" in decoded
    assert "raw=0x5D00" in decoded


def test_parse_pace_cells_and_temps_response():
    req = parse_frame(build_request(0x01, 0x03, 0x000F, 0x0016))
    regs = [
        3577, 3573, 3572, 3575, 3573, 3573, 3572, 3571,
        3572, 3572, 3572, 3572, 3573, 3573, 3573, 3572,
        280, 281, 282, 283, 270, 250,
    ]
    frame = parse_frame(build_response(0x01, 0x03, regs), req)
    decoded = describe_frame(frame)

    assert frame["start"] == 0x000F
    assert frame["registers"][-1]["addr"] == 0x0024
    assert "cells count=16" in decoded
    assert "C01=3.577V" in decoded
    assert "C16=3.572V" in decoded
    assert "min=3.571V#8" in decoded
    assert "max=3.577V#1" in decoded
    assert "temp1=28.0C" in decoded
    assert "temp4=28.3C" in decoded
    assert "mos_temp=27.0C" in decoded
    assert "env_temp=25.0C" in decoded


def test_reject_bad_pace_crc():
    raw = bytearray(build_request(0x01, 0x03, 0x000F, 0x0016))
    raw[-1] ^= 0x01
    frame = parse_frame(raw)

    assert not frame["crc_ok"]
    assert frame["expected_crc"] != frame["crc"]


def test_sigrok_pace_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("pace_modbus", "pace_modbus.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("pace_modbus")

    assert module.Decoder.id == "pace_modbus"
    assert module.Decoder.inputs == ["uart"]


def test_sigrok_pace_decoder_tracks_request_and_response(monkeypatch):
    class FakeSrdDecoder:
        def register(self, output):
            return output

        def put(self, ss, es, output, data):
            self.captured.append((ss, es, output, data))

    stub_sigrokdecode = types.SimpleNamespace(Decoder=FakeSrdDecoder, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("pace_modbus", "pace_modbus.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("pace_modbus")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()

    req = build_request(0x01, 0x03, 0x000F, 0x0016)
    rsp = build_response(0x01, 0x03, [
        3577, 3573, 3572, 3575, 3573, 3573, 3572, 3571,
        3572, 3572, 3572, 3572, 3573, 3573, 3573, 3572,
        280, 281, 282, 283, 270, 250,
    ])
    stream = req + rsp
    for idx, byte in enumerate(stream):
        decoder.decode(idx, idx + 1, ("DATA", 0, (byte, [])))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("PACE Modbus req slave=0x01" in text for text in texts)
    assert any("PACE Modbus rsp slave=0x01" in text and "regs=0x000F..0x0024" in text for text in texts)
    assert any("cells count=16" in text for text in texts)
