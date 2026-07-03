import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "growatt_rs485"
)
sys.path.insert(0, str(DECODER_DIR))

from growatt import describe_frame, describe_registers, frame_summary, modbus_crc16, parse_frame  # noqa: E402


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


def test_parse_growatt_request():
    frame = parse_frame(build_request(0x01, 0x03, 0x0010, 0x0008))

    assert frame["type"] == "request"
    assert frame["slave"] == 0x01
    assert frame["func"] == 0x03
    assert frame["start"] == 0x0010
    assert frame["count"] == 8
    assert frame["crc_ok"]
    assert "start=0x0010 count=8" in describe_frame(frame)
    assert "Growatt RS485 req slave=0x01" in frame_summary(frame, "TX")


def test_parse_growatt_response_with_request_context():
    req = parse_frame(build_request(0x01, 0x03, 0x0010, 0x0008))
    regs = [0, 0, 0, 0x00CB, 0x1424, 80, 5120, 0]
    frame = parse_frame(build_response(0x01, 0x03, regs), req)

    assert frame["type"] == "response"
    assert frame["crc_ok"]
    assert frame["start"] == 0x0010
    assert frame["registers"][3]["addr"] == 0x0013
    assert frame["registers"][5]["addr"] == 0x0015
    assert "SOC=80%" in describe_frame(frame)
    assert "pack_v=51.20V" in describe_frame(frame)
    assert "prot=0x1424" in describe_registers(frame["registers"])
    assert "regs=0x0010..0x0017" in frame_summary(frame, "RX")


def test_parse_growatt_cell_voltage_response():
    req = parse_frame(build_request(0x01, 0x03, 0x0071, 0x0003))
    frame = parse_frame(build_response(0x01, 0x03, [3570, 3569, 3571]), req)

    assert frame["registers"][0]["addr"] == 0x0071
    assert "cell01=3.570V" in describe_registers(frame["registers"]) or "cells count=3" in describe_frame(frame)
    assert "min=3.569V#2" in describe_frame(frame)
    assert "max=3.571V#3" in describe_frame(frame)


def test_reject_bad_growatt_crc():
    raw = bytearray(build_request(0x01, 0x03, 0x0010, 0x0008))
    raw[-1] ^= 0x01
    frame = parse_frame(raw)

    assert not frame["crc_ok"]
    assert frame["expected_crc"] != frame["crc"]


def test_sigrok_growatt_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("growatt_rs485", "growatt_rs485.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("growatt_rs485")

    assert module.Decoder.id == "growatt_rs485"
    assert module.Decoder.inputs == ["uart"]


def test_sigrok_growatt_decoder_tracks_request_and_response(monkeypatch):
    class FakeSrdDecoder:
        def register(self, output):
            return output

        def put(self, ss, es, output, data):
            self.captured.append((ss, es, output, data))

    stub_sigrokdecode = types.SimpleNamespace(Decoder=FakeSrdDecoder, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("growatt_rs485", "growatt_rs485.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("growatt_rs485")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()

    req = build_request(0x01, 0x03, 0x0010, 0x0008)
    rsp = build_response(0x01, 0x03, [0, 0, 0, 0x00CB, 0x1424, 80, 5120, 0])
    stream = req + rsp
    for idx, byte in enumerate(stream):
        decoder.decode(idx, idx + 1, ("DATA", 0, (byte, [])))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("Growatt RS485 req slave=0x01" in text for text in texts)
    assert any("Growatt RS485 rsp slave=0x01" in text and "regs=0x0010..0x0017" in text for text in texts)
    assert any("SOC=80%" in text for text in texts)
    assert any("pack_v=51.20V" in text for text in texts)
