import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "jkbms_modbus"
)
sys.path.insert(0, str(DECODER_DIR))

from jkbms_modbus import describe_frame, describe_register, frame_summary, modbus_crc16, parse_frame  # noqa: E402


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


def test_parse_jkbms_modbus_runtime_poll_request():
    frame = parse_frame(build_request(0x01, 0x03, 0x1200, 0x0010))

    assert frame["type"] == "request"
    assert frame["slave"] == 0x01
    assert frame["func"] == 0x03
    assert frame["start"] == 0x1200
    assert frame["count"] == 16
    assert frame["crc_ok"]
    assert "start=0x1200 count=16 poll-block" in describe_frame(frame)
    assert "JKBMS Modbus req slave=0x01" in frame_summary(frame, "TX")


def test_parse_jkbms_modbus_cell_response_with_request_context():
    req = parse_frame(build_request(0x01, 0x03, 0x1200, 0x0010))
    regs = [
        3571, 3572, 3570, 3573,
        3571, 3572, 3571, 3570,
        3571, 3572, 3570, 3571,
        3572, 3571, 3570, 3571,
    ]
    frame = parse_frame(build_response(0x01, 0x03, regs), req)
    decoded = describe_frame(frame)

    assert frame["type"] == "response"
    assert frame["crc_ok"]
    assert frame["start"] == 0x1200
    assert frame["registers"][0]["addr"] == 0x1200
    assert frame["registers"][-1]["addr"] == 0x121E
    assert "cells stride2 count=16" in decoded
    assert "C01=3.571V" in decoded
    assert "C04=3.573V" in decoded
    assert "min=3.570V#3" in decoded
    assert "max=3.573V#4" in decoded
    assert "regs=0x1200..0x121E" in frame_summary(frame, "RX")


def test_parse_jkbms_modbus_pack_status_response():
    req = parse_frame(build_request(0x01, 0x03, 0x128A, 0x0028))
    regs = [0] * 40
    regs[0] = 306       # 0x128A MOS temp, deci-C
    regs[3] = 0         # 0x1290 pack voltage high word
    regs[4] = 57150     # 0x1292 pack voltage low word, mV
    regs[7] = 0xFFFF    # 0x1298 pack current high word
    regs[8] = 0xFB3E    # 0x129A pack current low word: -1218 mA
    regs[9] = 309       # 0x129C battery temp 1, deci-C
    regs[10] = 304      # 0x129E battery temp 2, deci-C
    regs[14] = 0x0064   # 0x12A6 balance/SOC byte pair
    regs[17] = 0x0004   # 0x12AC full capacity high word
    regs[18] = 0x5CC0   # 0x12AE full capacity low word: 285.888 Ah
    regs[19] = 0        # 0x12B0 cycles high word
    regs[20] = 40       # 0x12B2 cycles low word

    frame = parse_frame(build_response(0x01, 0x03, regs), req)
    decoded = describe_frame(frame)

    assert frame["type"] == "response"
    assert frame["registers"][3]["addr"] == 0x1290
    assert "pack_v=57.150V" in decoded
    assert "pack_i=-1.218A" in decoded
    assert "SOC=100%" in decoded
    assert "MOS=30.6C" in decoded
    values = {reg["addr"]: reg["value"] for reg in frame["registers"]}
    assert "cycles=40" in describe_register(0x12B0, values[0x12B0], values)


def test_reject_bad_jkbms_modbus_crc():
    raw = bytearray(build_request(0x01, 0x03, 0x1200, 0x0010))
    raw[-1] ^= 0x01
    frame = parse_frame(raw)

    assert not frame["crc_ok"]
    assert frame["expected_crc"] != frame["crc"]


def test_sigrok_jkbms_modbus_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("jkbms_modbus", "jkbms_modbus.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("jkbms_modbus")

    assert module.Decoder.id == "jkbms_modbus"
    assert module.Decoder.inputs == ["uart"]


def test_sigrok_jkbms_modbus_decoder_tracks_request_and_response(monkeypatch):
    class FakeSrdDecoder:
        def register(self, output):
            return output

        def put(self, ss, es, output, data):
            self.captured.append((ss, es, output, data))

    stub_sigrokdecode = types.SimpleNamespace(Decoder=FakeSrdDecoder, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("jkbms_modbus", "jkbms_modbus.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("jkbms_modbus")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()

    req = build_request(0x01, 0x03, 0x1200, 0x0010)
    rsp = build_response(0x01, 0x03, [
        3571, 3572, 3570, 3573, 3571, 3572, 3571, 3570,
        3571, 3572, 3570, 3571, 3572, 3571, 3570, 3571,
    ])
    stream = req + rsp
    for idx, byte in enumerate(stream):
        decoder.decode(idx, idx + 1, ("DATA", 0, (byte, [])))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("JKBMS Modbus req slave=0x01" in text for text in texts)
    assert any("JKBMS Modbus rsp slave=0x01" in text and "regs=0x1200..0x121E" in text for text in texts)
    assert any("cells stride2 count=16" in text for text in texts)


def test_sigrok_jkbms_modbus_decoder_ignores_short_idle_fragments(monkeypatch):
    class FakeSrdDecoder:
        def register(self, output):
            return output

        def put(self, ss, es, output, data):
            self.captured.append((ss, es, output, data))

    stub_sigrokdecode = types.SimpleNamespace(
        Decoder=FakeSrdDecoder,
        OUTPUT_ANN=1,
        SRD_CONF_SAMPLERATE=2,
    )
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("jkbms_modbus", "jkbms_modbus.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("jkbms_modbus")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.options = {"inter_frame_gap_us": 2000}
    decoder.start()
    decoder.metadata(stub_sigrokdecode.SRD_CONF_SAMPLERATE, 200_000_000)

    decoder.decode(0, 1, ("DATA", 0, (0x00, [])))
    decoder.decode(500_000, 500_001, ("DATA", 0, (0x01, [])))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert not any("Incomplete JKBMS/Modbus frame" in text for text in texts)


def test_parse_jkbms_modbus_live_compact_cell_block():
    req = parse_frame(build_request(0x01, 0x03, 0x1210, 0x0010))
    regs = [
        3570, 3569, 3570, 3570,
        3570, 3571, 3570, 3570,
        3570, 3570, 3570, 3570,
        3570, 3570, 3570, 3570,
    ]
    frame = parse_frame(build_response(0x01, 0x03, regs), req)
    decoded = describe_frame(frame)

    assert frame["registers"][0]["addr"] == 0x1210
    assert "cells stride2 count=16" in decoded
    assert "C09=3.570V" in decoded
    assert "C10=3.569V" in decoded
    assert "C14=3.571V" in decoded
    assert "min=3.569V#10" in decoded
    assert "max=3.571V#14" in decoded


def test_jkbms_modbus_runtime_summary_avoids_false_pack_power():
    req = parse_frame(build_request(0x01, 0x03, 0x128A, 0x0028))
    regs = [0] * 40
    regs[5] = 0x0008    # 0x1294 high word can combine into a plausible but false power
    regs[6] = 0x00D4    # 0x1296 low word: 524.5 W if interpreted without context
    regs[7] = 0x0000    # 0x1298 current high
    regs[8] = 0x0063    # 0x129A current low: 99 mA, near idle
    regs[14] = 0x0000   # 0x12A6 balance/SOC word can be a zero placeholder

    frame = parse_frame(build_response(0x01, 0x03, regs), req)
    decoded = describe_frame(frame)

    assert "pack_v=" not in decoded
    assert "pack_p=+524.5W" not in decoded
    assert "pack_i=+0.099A" in decoded
    assert "SOC=0%" not in decoded


def test_jkbms_modbus_runtime_summary_does_not_invent_pack_power():
    req = parse_frame(build_request(0x01, 0x03, 0x128A, 0x0028))
    regs = [0] * 40
    regs[3] = 0x0000
    regs[4] = 57121      # 0x1290 / 0x1292 pack voltage, mV
    regs[5] = 0x0008
    regs[6] = 0x00D4     # 0x1294 / 0x1296 looks like false 524.5 W
    regs[7] = 0x0000
    regs[8] = 0x0063     # 0x1298 / 0x129A pack current, mA

    frame = parse_frame(build_response(0x01, 0x03, regs), req)
    decoded = describe_frame(frame)

    assert "pack_v=57.121V" in decoded
    assert "pack_i=+0.099A" in decoded
    assert "pack_p_calc=" not in decoded
    assert "pack_p=+524.5W" not in decoded


def test_jkbms_modbus_register_rows_keep_suspect_power_and_balance_as_candidates():
    values = {
        0x1294: 0x0008,
        0x1296: 0x00D4,
        0x12A0: 0x2343,
        0x12A2: 0x6400,
    }

    assert describe_register(0x1294, values[0x1294], values) == "pack_p_candidate=+524.5W"
    assert describe_register(0x12A4, 0x50D8) == "balance_current_candidate=20.696A implausible"
    assert describe_register(0x12A0, values[0x12A0], values) == "alarm_status_candidate=0x23436400 tentative"


def test_jkbms_modbus_runtime_summary_marks_alarm_status_as_candidate():
    req = parse_frame(build_request(0x01, 0x03, 0x128A, 0x0028))
    regs = [0] * 40
    regs[11] = 0x2343   # 0x12A0, live no-alarm candidate observed on JK Modbus
    regs[12] = 0x6400   # 0x12A2

    frame = parse_frame(build_response(0x01, 0x03, regs), req)
    decoded = describe_frame(frame)

    assert "alarm_candidate=0x23436400" in decoded
    assert "balance wire resistance fault" not in decoded
    assert "alarm=0x23436400" not in decoded


def test_jkbms_modbus_invalid_cell_index_word_is_raw():
    assert describe_register(0x1248, 0x0042) == "cell_idx raw=0x0042"
    assert describe_register(0x1248, 0x0110) == "cell_idx max#1 min#16"


def test_jkbms_modbus_does_not_promote_impossible_compact_cell_indexes():
    req = parse_frame(build_request(0x01, 0x03, 0x1230, 0x0010))
    regs = [
        0, 0, 0, 4,
        0xFFFF, 0x0DF6, 0, 4,
        0, 3, 0x0045, 0x0042,
        0x0045, 0, 0, 0,
    ]
    frame = parse_frame(build_response(0x01, 0x03, regs), req)
    decoded = describe_frame(frame)

    assert "C54" not in decoded
    assert "C59" not in decoded
    assert "C30=3.574V" in decoded
