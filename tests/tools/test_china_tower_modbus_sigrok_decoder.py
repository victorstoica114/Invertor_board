import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "china_tower_modbus"
)
sys.path.insert(0, str(DECODER_DIR))

from china_tower_modbus import (  # noqa: E402
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


def test_parse_china_tower_summary_poll_request():
    frame = parse_frame(build_request(0x01, 0x03, 0x0000, 0x000D))

    assert frame["type"] == "request"
    assert frame["slave"] == 0x01
    assert frame["func"] == 0x03
    assert frame["start"] == 0x0000
    assert frame["count"] == 13
    assert frame["crc_ok"]
    assert "start=0x0000 count=13 poll-block" in describe_frame(frame)
    assert "China Tower Modbus req slave=0x01" in frame_summary(frame, "TX")


def test_parse_china_tower_summary_response_with_request_context():
    req = parse_frame(build_request(0x01, 0x03, 0x0000, 0x000D))
    regs = [
        5715,       # 57.15 V
        16,
        0x6400,    # high-byte SOC layout
        0,
        0,
        0,
        24,
        25,
        29,
        3571,
        3572,
        3572,
        3571,
    ]
    frame = parse_frame(build_response(0x01, 0x03, regs), req)
    decoded = describe_frame(frame)

    assert frame["type"] == "response"
    assert frame["crc_ok"]
    assert frame["registers"][0]["addr"] == 0x0000
    assert "pack_v=57.15V" in decoded
    assert "cell_count=16" in decoded
    assert "SOC=100%" in decoded
    assert "temp1=24C" in decoded
    assert "mos_temp=29C" in decoded
    assert "cells count=4" in decoded


def test_parse_china_tower_cell_voltage_response():
    req = parse_frame(build_request(0x01, 0x03, 0x0009, 0x0010))
    regs = [3571, 3572, 3572, 3571, 3573, 3570, 3571, 3572,
            3571, 3571, 3572, 3573, 3571, 3572, 3573, 3572]
    frame = parse_frame(build_response(0x01, 0x03, regs), req)
    decoded = describe_frame(frame)

    assert frame["start"] == 0x0009
    assert frame["registers"][-1]["addr"] == 0x0018
    assert "cells count=16" in decoded
    assert "C01=3.571V" in decoded
    assert "C16=3.572V" in decoded
    assert "min=3.570V#6" in decoded
    assert "max=3.573V#5" in decoded


def test_parse_china_tower_flags_response():
    req = parse_frame(build_request(0x01, 0x03, 0x0019, 0x0003))
    frame = parse_frame(build_response(0x01, 0x03, [0x0000, 0x0000, 0x0F00]), req)
    decoded = describe_frame(frame)

    assert frame["registers"][2]["addr"] == 0x001B
    assert "warning=0x0000 (none)" in decoded
    assert "protection=0x0000 (none)" in decoded
    assert "status=0x0F00" in decoded
    assert "charge MOS" in describe_register(0x001B, 0x0F00)


def test_reject_bad_china_tower_crc():
    raw = bytearray(build_request(0x01, 0x03, 0x0009, 0x0010))
    raw[-1] ^= 0x01
    frame = parse_frame(raw)

    assert not frame["crc_ok"]
    assert frame["expected_crc"] != frame["crc"]


def test_sigrok_china_tower_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("china_tower_modbus", "china_tower_modbus.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("china_tower_modbus")

    assert module.Decoder.id == "china_tower_modbus"
    assert module.Decoder.inputs == ["uart"]


def test_sigrok_china_tower_decoder_tracks_request_and_response(monkeypatch):
    class FakeSrdDecoder:
        def register(self, output):
            return output

        def put(self, ss, es, output, data):
            self.captured.append((ss, es, output, data))

    stub_sigrokdecode = types.SimpleNamespace(Decoder=FakeSrdDecoder, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("china_tower_modbus", "china_tower_modbus.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("china_tower_modbus")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()

    req = build_request(0x01, 0x03, 0x0009, 0x0010)
    rsp = build_response(0x01, 0x03, [3571, 3572, 3572, 3571, 3573, 3570, 3571, 3572,
                                      3571, 3571, 3572, 3573, 3571, 3572, 3573, 3572])
    stream = req + rsp
    for idx, byte in enumerate(stream):
        decoder.decode(idx, idx + 1, ("DATA", 0, (byte, [])))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("China Tower Modbus req slave=0x01" in text for text in texts)
    assert any("China Tower Modbus rsp slave=0x01" in text and "regs=0x0009..0x0018" in text for text in texts)
    assert any("cells count=16" in text for text in texts)
