import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "pylon_rs485"
)
CAN_DECODER_DIR = DECODER_DIR.parent / "pylon_can"
PULSEVIEW_DECODER_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode\decoders")
PULSEVIEW_SRD_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode")
sys.path.insert(0, str(DECODER_DIR))
sys.path.insert(0, str(CAN_DECODER_DIR))

from pylon import (VERSION, ascii_checksum, describe_info, describe_payload_texts,
                   frame_summary, parse_frame, status63_flags)  # noqa: E402
from pylon_can import describe_packet, frame_summary as can_frame_summary  # noqa: E402


def build_pylon_response(info_ascii, addr=0x02):
    info_len = len(info_ascii)
    n0 = (info_len >> 8) & 0x0F
    n1 = (info_len >> 4) & 0x0F
    n2 = info_len & 0x0F
    lchksum = (~(n0 + n1 + n2) + 1) & 0x0F
    body = "20{:02X}4600{:04X}{}".format(addr, (lchksum << 12) | info_len, info_ascii)
    return parse_frame("~{}{:04X}\r".format(body, ascii_checksum(body)))


def build_pylon_request(cid2, info_ascii="", addr=0x02):
    info_len = len(info_ascii)
    n0 = (info_len >> 8) & 0x0F
    n1 = (info_len >> 4) & 0x0F
    n2 = info_len & 0x0F
    lchksum = (~(n0 + n1 + n2) + 1) & 0x0F
    body = "20{:02X}46{:02X}{:04X}{}".format(addr, cid2, (lchksum << 12) | info_len, info_ascii)
    return parse_frame("~{}{:04X}\r".format(body, ascii_checksum(body)))


def test_parse_la2016_pylon_status_frame():
    frame = parse_frame("~20024600D01205E0B180017C076CC0F9B8\r")

    assert frame["ver"] == 0x20
    assert frame["addr"] == 0x02
    assert frame["cid1"] == 0x46
    assert frame["code"] == 0x00
    assert frame["length_field"] == 0xD012
    assert frame["length_ok"]
    assert frame["checksum"] == 0xF9B8
    assert frame["checksum_ok"]
    assert frame["info_bytes"] == [0x05, 0xE0, 0xB1, 0x80, 0x01, 0x7C, 0x07, 0x6C, 0xC0]


def test_describe_pylon_status_63():
    frame = parse_frame("~20024600D01205E0B180017C076CC0F9B8\r")

    assert "0x63 status=0xC0" in describe_info(frame, 0x63)
    assert "charge=ON" in status63_flags(0xC0)
    assert "discharge=ON" in status63_flags(0xC0)
    assert "balance=OFF" in status63_flags(0xC0)
    assert "Pylon rsp addr=0x02 OK cid2=0x63 chk=OK" in frame_summary(frame, "RX", 0x63)


def test_describe_pylon_analog_61_uses_millivolt_pack_voltage():
    payload = (
        "DF360000640000000064640DF400010DF30002"
        "0BA40BAE00000BA800000BA60BA3"
    )
    frame = build_pylon_response(payload)

    decoded = describe_info(frame, 0x61)

    assert "0x61 V=57.142V" in decoded
    assert "SOC=100%" in decoded
    assert "SOH=100%" in decoded
    assert "cell_max=3.572V#1" in decoded
    assert "cell_min=3.571V#2" in decoded
    assert "MOS=24.9C" in decoded
    assert "T5=24.8C" in decoded
    assert "0x61 payload" in describe_payload_texts(frame, 0x61)[0]


def test_describe_pylon_42_cell_list_response():
    frame = build_pylon_response("040DF10DF20DF30DF4")

    decoded = describe_info(frame, 0x42)

    assert "0x42 cells simple layout count=4" in decoded
    assert "min=3.569V#1" in decoded
    assert "max=3.572V#4" in decoded
    assert "C01=3.569V" in decoded
    assert "C04=3.572V" in decoded


def test_describe_pylon_42_empty_response_is_explicit():
    frame = build_pylon_response("")

    assert "0x42 empty response/no payload" in describe_info(frame, 0x42)
    assert "empty" in frame_summary(frame, "RX", 0x42)


def test_describe_pylon_42_request_selector():
    frame = build_pylon_request(0x42, "FF")

    assert "request cell information selector=0xFF aggregate/all packs" in describe_info(frame)
    assert "Pylon req addr=0x02 cid2=0x42 cell information" in frame_summary(frame, "TX")


def test_describe_pylon_62_zero_flags():
    frame = build_pylon_response("00000000")

    assert "0x62 flags=0x00000000 (no flags set)" in describe_info(frame, 0x62)


def test_reject_bad_pylon_checksum():
    frame = parse_frame("~20024600D01205E0B180017C076CC0F9B9\r")

    assert not frame["checksum_ok"]
    assert frame["expected_checksum"] == 0xF9B8


def test_sigrok_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("pylon_rs485", "pylon_rs485.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("pylon_rs485")

    assert module.Decoder.id == "pylon_rs485"
    assert VERSION in module.Decoder.name
    assert VERSION in module.Decoder.longname


def test_sigrok_decoder_emits_annotations_for_uart_frame(monkeypatch):
    class FakeSrdDecoder:
        def register(self, output):
            return output

        def put(self, ss, es, output, data):
            self.captured.append((ss, es, output, data))

    stub_sigrokdecode = types.SimpleNamespace(Decoder=FakeSrdDecoder, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("pylon_rs485", "pylon_rs485.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("pylon_rs485")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()

    frame = b"~20024600D01205E0B180017C076CC0F9B8\r"
    for idx, byte in enumerate(frame):
        decoder.decode(idx, idx + 1, ("DATA", 0, (byte, [])))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("Pylon rsp addr=0x02 OK" in text for text in texts)
    assert any("CHK 0xF9B8 OK" in text for text in texts)
    assert any("0x63 status=0xC0" in text for text in texts)
    assert any(VERSION in row[1] for row in module.Decoder.annotation_rows)


def test_pylon_can_describes_live_jk_frames():
    packet_356 = ("standard", 0x356, "data", 8, [0x54, 0x16, 0x00, 0x00, 0xEB, 0x00, 0x00, 0x00])
    packet_370 = ("standard", 0x370, "data", 8, [0x17, 0x00, 0x17, 0x00, 0xF5, 0x0D, 0xF4, 0x0D])
    packet_371 = ("standard", 0x371, "data", 8, [0x04, 0x00, 0x01, 0x00, 0x01, 0x00, 0x04, 0x00])

    assert "0x356 pack V=57.16V" in describe_packet(packet_356)
    assert "0x370 JK ext" in describe_packet(packet_370)
    assert "cell_max=3.573V" in describe_packet(packet_370)
    assert "cell_min=3.572V" in describe_packet(packet_370)
    assert "cellMax#1" in describe_packet(packet_371)
    assert "cellMin#4" in describe_packet(packet_371)
    assert "Pylon CAN 0x370" in can_frame_summary(packet_370)


def test_sigrok_can_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(CAN_DECODER_DIR.parent))

    for name in ("can", "can.pd", "pylon_can", "pylon_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("pylon_can")

    assert module.Decoder.id == "pylon_can"
    assert module.Decoder.inputs == ["logic"]
    assert any(option["id"] == "input_mode" for option in module.Decoder.options)


def test_sigrok_can_decoder_derives_bus_level_from_raw_can_lines(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(CAN_DECODER_DIR.parent))

    for name in ("can", "can.pd", "pylon_can", "pylon_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("pylon_can")
    decoder = module.Decoder()

    decoder.options = {"input_mode": "rx/canl-direct"}
    assert decoder.derive_can_rx(1) == 1
    assert decoder.derive_can_rx(0) == 0

    decoder.options = {"input_mode": "canh-inverted"}
    assert decoder.derive_can_rx(0) == 1
    assert decoder.derive_can_rx(1) == 0

    decoder.options = {"input_mode": "canh-canl-diff"}
    assert decoder.derive_can_rx(1, 0) == 0
    assert decoder.derive_can_rx(0, 1) == 1
    assert decoder.derive_can_rx(1, 1) == 1


def test_sigrok_can_decoder_emits_annotations_from_internal_can_packet(monkeypatch):
    class FakeSrdDecoder:
        def register(self, output):
            return output

        def put(self, ss, es, output, data):
            self.captured.append((ss, es, output, data))

    stub_sigrokdecode = types.SimpleNamespace(Decoder=FakeSrdDecoder, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(CAN_DECODER_DIR.parent))

    for name in ("can", "can.pd", "pylon_can", "pylon_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("pylon_can")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()
    decoder.ss_packet = 10
    decoder.es_packet = 20

    decoder.putpy(("standard", 0x370, "data", 8,
                   [0x17, 0x00, 0x17, 0x00, 0xF5, 0x0D, 0xF4, 0x0D]))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("Pylon CAN 0x370" in text for text in texts)
    assert any("17 00 17 00 F5 0D F4 0D" in text for text in texts)
    assert any("cell_max=3.573V" in text for text in texts)
