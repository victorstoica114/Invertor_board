import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "jkbms_rs485_native"
)
sys.path.insert(0, str(DECODER_DIR))

from jkbms_native import describe_frame, frame_summary, parse_frame  # noqa: E402


READ_ALL_REQUEST = bytes([
    0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00,
    0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x68, 0x00, 0x00, 0x01, 0x29,
])

SAMPLE_RESPONSE = bytes([
    0x4E, 0x57, 0x00, 0x3A, 0x00, 0x00, 0x00, 0x00,
    0x06, 0x00, 0x01,
    0x79, 0x06, 0x01, 0x11, 0xB0, 0x02, 0x11, 0xC0,
    0x80, 0x00, 0x1D,
    0x81, 0x00, 0x1C,
    0x82, 0x00, 0x1E,
    0x83, 0x1C, 0x6C,
    0x84, 0x80, 0x0A,
    0x85, 0x64,
    0x86, 0x03,
    0x87, 0x03, 0xBD,
    0x8A, 0x00, 0x02,
    0x8B, 0x24, 0x04,
    0x8C, 0x00, 0x03,
    0xAA, 0x00, 0x00, 0x00, 0x28,
    0x68, 0x00, 0x00, 0x00, 0x00,
])


def test_parse_jkbms_native_read_all_request():
    frame = parse_frame(READ_ALL_REQUEST)

    assert frame["length"] == 21
    assert frame["source"] == 0x03
    assert frame["frame_type"] == 0x00
    assert frame["checksum_ok"]
    assert "JKBMS native req" in frame_summary(frame, "TX")


def test_parse_jkbms_native_response_decodes_cells_pack_and_status():
    frame = parse_frame(SAMPLE_RESPONSE)
    text = describe_frame(frame)

    assert frame["length"] == 60
    assert not frame["checksum_ok"]
    assert "0x79 cells count=2" in text
    assert "min=4.528V#1" in text
    assert "max=4.544V#2" in text
    assert "pack_v=72.76V" in text
    assert "pack_i=+0.10A" in text
    assert "SOC=100%" in text
    assert "Charge voltage high" in text
    assert "charge=ON" in text
    assert "discharge=ON" in text


def test_sigrok_jkbms_native_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("jkbms_rs485_native", "jkbms_rs485_native.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("jkbms_rs485_native")

    assert module.Decoder.id == "jkbms_rs485_native"
    assert module.Decoder.inputs == ["uart"]


def test_sigrok_jkbms_native_decoder_emits_annotations(monkeypatch):
    class FakeSrdDecoder:
        def register(self, output):
            return output

        def put(self, ss, es, output, data):
            self.captured.append((ss, es, output, data))

    stub_sigrokdecode = types.SimpleNamespace(Decoder=FakeSrdDecoder, OUTPUT_ANN=1)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("jkbms_rs485_native", "jkbms_rs485_native.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("jkbms_rs485_native")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()

    stream = READ_ALL_REQUEST + SAMPLE_RESPONSE
    for idx, byte in enumerate(stream):
        decoder.decode(idx, idx + 1, ("DATA", 0, (byte, [])))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("JKBMS native req" in text for text in texts)
    assert any("JKBMS native rsp" in text for text in texts)
    assert any("SOC=100%" in text for text in texts)
    assert any("pack_v=72.76V" in text for text in texts)
