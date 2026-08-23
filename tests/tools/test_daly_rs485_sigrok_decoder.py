import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "daly_rs485"
)
sys.path.insert(0, str(DECODER_DIR))

from daly_rs485 import (  # noqa: E402
    DALY_FRAME_LEN,
    VERSION,
    describe_frame,
    frame_complete,
    frame_summary,
    modbus_crc16,
    native_checksum,
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


def build_native(addr, cmd, data):
    raw = bytearray([0xA5, addr, cmd, 0x08] + list(data[:8]))
    while len(raw) < DALY_FRAME_LEN - 1:
        raw.append(0)
    raw.append(native_checksum(raw + b"\x00"))
    return bytes(raw)


def test_parse_daly_native_pack_frame():
    raw = build_native(0x01, 0x90, [0x02, 0x3A, 0, 0, 0x75, 0x30, 0x03, 0xE0])

    frame = parse_frame(raw)
    text = describe_frame(frame)

    assert frame["protocol"] == "native"
    assert frame["type"] == "rsp"
    assert frame["checksum_ok"]
    assert "pack V=57.0V" in text
    assert "SOC=99.2%" in text
    assert "Daly native rsp" in frame_summary(frame, "RX")


def test_parse_daly_native_cell_voltage_frame():
    raw = build_native(0x01, 0x95, [0x02, 0x0D, 0xF1, 0x0D, 0xF2, 0x0D, 0xF3, 0])
    frame = parse_frame(raw)
    text = describe_frame(frame)

    assert "frame=2" in text
    assert "C04=3.569V" in text
    assert "C06=3.571V" in text
    assert frame_complete(raw)


def test_parse_daly_modbus_request_and_cell_response_with_context():
    req = parse_frame(build_request(0x81, 0x03, 0x0000, 0x007F))
    regs = [
        3569, 3570, 3571, 3572, 3573, 3574, 3575, 3576,
        3576, 3575, 3574, 3573, 3572, 3571, 3570, 3569,
    ]
    rsp = parse_frame(build_response(0x51, 0x03, regs), req)
    text = describe_frame(rsp)

    assert req["type"] == "request"
    assert req["crc_ok"]
    assert "cells/info block" in describe_frame(req)
    assert rsp["type"] == "response"
    assert rsp["crc_ok"]
    assert rsp["registers"][0]["addr"] == 0
    assert "cells count=16" in text
    assert "C01=3.569V" in text
    assert "C08=3.576V" in text


def test_parse_daly_modbus_soc_and_temperature_candidates():
    req = parse_frame(build_request(0x81, 0x03, 0x0000, 0x007F))
    regs = [0] * 91
    regs[0:4] = [3571, 3572, 3573, 3574]
    regs[48:52] = [66, 67, 68, 69]
    regs[56] = 570
    regs[57] = 29871
    regs[58] = 992
    regs[61] = 4
    regs[90] = 70

    rsp = parse_frame(build_response(0x51, 0x03, regs), req)
    text = describe_frame(rsp)

    assert "SOC=99.2%" in text
    assert "Pack=57.0V" in text
    assert "I=-12.9A" in text
    assert "T1=26.0C" in text
    assert "T4=29.0C" in text
    assert "MOS=30.0C" in text


def test_sigrok_daly_rs485_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("daly_rs485", "daly_rs485.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("daly_rs485")

    assert module.Decoder.id == "daly_rs485"
    assert module.Decoder.inputs == ["uart"]
    assert VERSION in module.Decoder.name


def test_sigrok_daly_rs485_decoder_tracks_modbus_request_and_response(monkeypatch):
    class FakeSrdDecoder:
        def register(self, output):
            return output

        def put(self, ss, es, output, data):
            self.captured.append((ss, es, output, data))

    stub_sigrokdecode = types.SimpleNamespace(Decoder=FakeSrdDecoder, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("daly_rs485", "daly_rs485.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("daly_rs485")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()

    req = build_request(0x81, 0x03, 0x0000, 0x007F)
    rsp = build_response(0x51, 0x03, [3569, 3570, 3571, 3572])
    for idx, byte in enumerate(req + rsp):
        decoder.decode(idx, idx + 1, ("DATA", 0, (byte, [])))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("Daly Modbus req" in text for text in texts)
    assert any("Daly Modbus rsp" in text and "regs=0x0000..0x0003" in text for text in texts)
    assert any("cells count=4" in text for text in texts)
