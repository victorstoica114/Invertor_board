import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "voltronic_modbus"
)
sys.path.insert(0, str(DECODER_DIR))

from voltronic_modbus import (  # noqa: E402
    describe_frame,
    describe_register,
    frame_complete,
    frame_summary,
    modbus_crc16,
    parse_frame,
)


def append_crc(payload):
    crc = modbus_crc16(payload)
    return bytes(payload + [crc & 0xFF, (crc >> 8) & 0xFF])


def build_request(slave, func, start, count, function_first=False):
    if function_first:
        payload = [func, slave]
    else:
        payload = [slave, func]
    payload.extend([
        (start >> 8) & 0xFF,
        start & 0xFF,
        (count >> 8) & 0xFF,
        count & 0xFF,
    ])
    return append_crc(payload)


def build_standard_response(slave, func, regs, function_first=False):
    if function_first:
        payload = [func, slave, len(regs) * 2]
    else:
        payload = [slave, func, len(regs) * 2]
    for value in regs:
        payload.extend([(value >> 8) & 0xFF, value & 0xFF])
    return append_crc(payload)


def build_word_count_response(slave, func, regs, function_first=False):
    if function_first:
        payload = [func, slave]
    else:
        payload = [slave, func]
    payload.extend([0x00, len(regs)])
    for value in regs:
        payload.extend([(value >> 8) & 0xFF, value & 0xFF])
    return append_crc(payload)


def build_wide_byte_count_response(slave, func, regs, function_first=False):
    if function_first:
        payload = [func, slave]
    else:
        payload = [slave, func]
    payload.extend([0x00, len(regs) * 2])
    for value in regs:
        payload.extend([(value >> 8) & 0xFF, value & 0xFF])
    return append_crc(payload)


def test_parse_voltronic_classic_request():
    frame = parse_frame(build_request(0x01, 0x03, 0x0033, 1))

    assert frame["type"] == "request"
    assert frame["order"] == "classic"
    assert frame["slave"] == 0x01
    assert frame["func"] == 0x03
    assert frame["start"] == 0x0033
    assert frame["count"] == 1
    assert frame["crc_ok"]
    assert "start=0x0033 soc count=1" in describe_frame(frame)
    assert "Voltronic Modbus req slave=0x01" in frame_summary(frame, "TX")
    assert "soc" in frame_summary(frame, "TX")


def test_parse_prefers_known_poll_request_over_stale_pending_context():
    pending_req = parse_frame(build_request(0x01, 0x03, 0x0074, 1))
    frame = parse_frame(build_request(0x01, 0x03, 0x0001, 1), pending_req)

    assert frame["type"] == "request"
    assert frame["start"] == 0x0001
    assert frame["count"] == 1
    assert frame["crc_ok"]


def test_parse_voltronic_function_first_request_and_response():
    req = parse_frame(build_request(0x01, 0x03, 0x0010, 6, function_first=True))
    rsp = parse_frame(
        build_standard_response(0x01, 0x03, [4, 4528, 4559, 4618, 4509, 4517], function_first=True),
        req,
    )
    decoded = describe_frame(rsp)

    assert req["order"] == "function-first"
    assert rsp["type"] == "response"
    assert rsp["order"] == "function-first"
    assert rsp["format"] == "standard"
    assert rsp["registers"][0]["addr"] == 0x0010
    assert "cells count=5" in decoded
    assert "C01=4.528V" in decoded
    assert "C03=4.618V" in decoded


def test_parse_voltronic_single_register_wide_response():
    req = parse_frame(build_request(0x01, 0x03, 0x0033, 1))
    rsp = parse_frame(build_wide_byte_count_response(0x01, 0x03, [93]), req)

    assert rsp["format"] == "wide-byte-count"
    assert rsp["registers"][0]["addr"] == 0x0033
    assert "SOC=93%" in describe_frame(rsp)


def test_frame_complete_accepts_short_wide_response_with_zero_payload():
    req = parse_frame(build_request(0x01, 0x03, 0x0033, 1))
    rsp_raw = build_wide_byte_count_response(0x01, 0x03, [0])

    assert rsp_raw == bytes.fromhex("01 03 00 02 00 00 E4 0A")
    assert frame_complete(rsp_raw)

    rsp = parse_frame(rsp_raw, req)

    assert rsp["type"] == "response"
    assert rsp["format"] == "wide-byte-count"
    assert rsp["crc_ok"]
    assert rsp["registers"][0]["value"] == 0


def test_parse_voltronic_jk_word_count_cell_block():
    req = parse_frame(build_request(0x01, 0x03, 0x1200, 16))
    regs = [
        3571, 0, 3572, 0, 3572, 0, 3572, 0,
        3572, 0, 3571, 0, 3571, 0, 3572, 0,
    ]
    rsp = parse_frame(build_word_count_response(0x01, 0x03, regs), req)
    decoded = describe_frame(rsp)

    assert rsp["format"] == "word-count"
    assert "cells count=8" in decoded
    assert "C01=3.571V" in decoded
    assert "C08=3.572V" in decoded


def test_parse_voltronic_jk_runtime_block():
    req = parse_frame(build_request(0x01, 0x03, 0x128A, 40))
    regs = [0] * 40
    regs[0x128A - 0x128A] = 270
    regs[0x1290 - 0x128A] = 7270
    regs[0x1292 - 0x128A] = 10
    regs[0x129C - 0x128A] = 284
    regs[0x129E - 0x128A] = 277
    regs[0x12A0 - 0x128A] = 0x2342
    regs[0x12A1 - 0x128A] = 0x6400
    regs[0x12A6 - 0x128A] = 0x0064
    regs[0x12A8 - 0x128A] = 0x0000
    regs[0x12A9 - 0x128A] = 40000
    regs[0x12AC - 0x128A] = 0x0000
    regs[0x12AD - 0x128A] = 40000
    rsp = parse_frame(build_word_count_response(0x01, 0x03, regs), req)
    decoded = describe_frame(rsp)

    assert "MOS=27.0C" in decoded
    assert "pack_v=72.70V" in decoded
    assert "pack_i_candidate=+0.10A" in decoded
    assert "SOC=100%" in decoded
    assert "remain=40.00Ah" in decoded
    assert "full=40.00Ah" in decoded


def test_describe_voltronic_public_limits_and_status():
    assert "chg_v_limit=57.6V" in describe_register(0x0070, 576)
    assert "dsg_i_limit=180.0A" in describe_register(0x0073, 1800)
    assert "charge_enable" in describe_register(0x0074, 0x00C0)
    assert "discharge_enable" in describe_register(0x0074, 0x00C0)
    assert "unknown=0x0001" in describe_register(0x0074, 0x0001)


def test_sigrok_voltronic_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("voltronic_modbus", "voltronic_modbus.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("voltronic_modbus")

    assert module.Decoder.id == "voltronic_modbus"
    assert module.Decoder.inputs == ["uart"]


def test_sigrok_voltronic_decoder_tracks_request_and_response(monkeypatch):
    class FakeSrdDecoder:
        def register(self, output):
            return output

        def put(self, ss, es, output, data):
            self.captured.append((ss, es, output, data))

    stub_sigrokdecode = types.SimpleNamespace(Decoder=FakeSrdDecoder, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("voltronic_modbus", "voltronic_modbus.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("voltronic_modbus")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()

    stream = (
        build_request(0x01, 0x03, 0x0033, 1)
        + build_wide_byte_count_response(0x01, 0x03, [93])
    )
    for idx, byte in enumerate(stream):
        decoder.decode(idx, idx + 1, ("DATA", 0, (byte, [])))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("Voltronic Modbus req slave=0x01" in text for text in texts)
    assert any("Voltronic Modbus rsp slave=0x01" in text for text in texts)
    assert any("SOC=93%" in text for text in texts)
