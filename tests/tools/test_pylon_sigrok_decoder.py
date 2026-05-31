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
sys.path.insert(0, str(DECODER_DIR))

from pylon import ascii_checksum, describe_info, frame_summary, parse_frame, status63_flags  # noqa: E402


def build_pylon_response(info_ascii, addr=0x02):
    info_len = len(info_ascii)
    n0 = (info_len >> 8) & 0x0F
    n1 = (info_len >> 4) & 0x0F
    n2 = info_len & 0x0F
    lchksum = (~(n0 + n1 + n2) + 1) & 0x0F
    body = "20{:02X}4600{:04X}{}".format(addr, (lchksum << 12) | info_len, info_ascii)
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
    payload = "DF360000640000000064640DF400010DF30002" + ("00" * 14)
    frame = build_pylon_response(payload)

    decoded = describe_info(frame, 0x61)

    assert "0x61 V=57.142V" in decoded
    assert "SOC=100%" in decoded
    assert "SOH=100%" in decoded
    assert "cell_max=3.572V#1" in decoded
    assert "cell_min=3.571V#2" in decoded


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

    texts = [item[3][1][0] for item in decoder.captured]

    assert any("Pylon rsp addr=0x02 OK" in text for text in texts)
    assert any("CHK 0xF9B8 OK" in text for text in texts)
    assert any("0x63 status=0xC0" in text for text in texts)
