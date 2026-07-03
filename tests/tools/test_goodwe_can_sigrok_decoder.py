import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "goodwe_can"
)
PULSEVIEW_DECODER_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode\decoders")
PULSEVIEW_SRD_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode")
sys.path.insert(0, str(DECODER_DIR))

from goodwe_can import describe_packet, frame_summary  # noqa: E402


def test_goodwe_can_describes_classic_goodwe_frames():
    limits = describe_packet(("standard", 0x456, "data", 8,
                              [0x40, 0x02, 0xE8, 0x03, 0xB0, 0x04, 0xC2, 0x01]))
    soc = describe_packet(("standard", 0x457, "data", 4,
                           [0x66, 0x03, 0xE8, 0x03]))
    pack = describe_packet(("standard", 0x458, "data", 8,
                            [0x3B, 0x02, 0x19, 0x00, 0x07, 0x01, 0, 0]))

    assert "0x456 GoodWe limits" in limits
    assert "chgV=57.6V" in limits
    assert "chgI=+100.0A" in limits
    assert "disI=+120.0A" in limits
    assert "lowV=45.0V" in limits
    assert "SOC=87.0% (87%)" in soc
    assert "SOH=100.0% (100%)" in soc
    assert "V=57.1V" in pack
    assert "I=+2.5A" in pack
    assert "temp=26.3C" in pack


def test_goodwe_can_describes_jk_pylon_like_dialect_frames():
    limits = describe_packet(("standard", 0x351, "data", 8,
                              [0x9E, 0x02, 0x7C, 0x01, 0x6C, 0x07, 0xC6, 0x01]))
    soc = describe_packet(("standard", 0x355, "data", 8,
                           [0x63, 0x00, 0x64, 0x00, 0, 0, 0, 0]))
    pack = describe_packet(("standard", 0x356, "data", 8,
                            [0x53, 0x16, 0x00, 0x00, 0x33, 0x01, 0, 0]))
    temp_cell = describe_packet(("standard", 0x370, "data", 8,
                                 [0x1F, 0x00, 0x1E, 0x00, 0xF5, 0x0D, 0xF3, 0x0D]))
    indexes = describe_packet(("standard", 0x371, "data", 8,
                               [0x04, 0, 0x03, 0, 0x01, 0, 0x0F, 0]))

    assert "JK/Pylon dialect limits" in limits
    assert "chgV=67.0V" in limits
    assert "chgI=38.0A" in limits
    assert "disI=190.0A" in limits
    assert "lowV=45.4V" in limits
    assert "SOC=99%" in soc
    assert "SOH=100%" in soc
    assert "V=57.15V" in pack
    assert "temp=30.7C" in pack
    assert "temp max=31.0C" in temp_cell
    assert "cell_max=3.573V" in temp_cell
    assert "cell_min=3.571V" in temp_cell
    assert "cell_max_idx=1" in indexes
    assert "cell_min_idx=15" in indexes


def test_goodwe_can_describes_identity_status_and_raw_frames():
    identity = describe_packet(("standard", 0x35E, "data", 8,
                                [ord("J"), ord("K"), ord("-"), ord("B"),
                                 ord("M"), ord("S"), 0, 0]))
    status = describe_packet(("standard", 0x35C, "data", 8,
                              [0xC0, 0, 0, 0, 0, 0, 0, 0]))
    modules = describe_packet(("standard", 0x453, "data", 2, [0x01, 0x00]))

    assert "identity 'JK-BMS'" in identity
    assert "state=0xC0" in status
    assert "charge=ON" in status
    assert "discharge=ON" in status
    assert "0x453 GoodWe modules raw=01 00" in modules


def test_goodwe_can_frame_summary_includes_visible_version():
    packet = ("standard", 0x458, "data", 8, [0x3B, 0x02, 0, 0, 0, 0, 0, 0])

    text = frame_summary(packet)

    assert "GoodWe CAN v2026.07.03a 0x458" in text
    assert "GoodWe pack" in text


def test_sigrok_goodwe_can_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "goodwe_can", "goodwe_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("goodwe_can")

    assert module.Decoder.id == "goodwe_can"
    assert module.Decoder.name == "GoodWe CAN v2026.07.03a"
    assert module.Decoder.inputs == ["logic"]
    assert any(option["id"] == "input_mode" for option in module.Decoder.options)


def test_sigrok_goodwe_can_decoder_derives_bus_level_from_raw_can_lines(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "goodwe_can", "goodwe_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("goodwe_can")
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


def test_sigrok_goodwe_can_decoder_emits_annotations_from_internal_can_packet(monkeypatch):
    class FakeSrdDecoder:
        def register(self, output):
            return output

        def put(self, ss, es, output, data):
            self.captured.append((ss, es, output, data))

    stub_sigrokdecode = types.SimpleNamespace(Decoder=FakeSrdDecoder, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "goodwe_can", "goodwe_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("goodwe_can")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()
    decoder.ss_packet = 10
    decoder.es_packet = 20

    decoder.putpy(("standard", 0x356, "data", 8,
                   [0x53, 0x16, 0x00, 0x00, 0x33, 0x01, 0, 0]))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("GoodWe CAN v2026.07.03a 0x356" in text for text in texts)
    assert any("53 16 00 00 33 01 00 00" in text for text in texts)
    assert any("V=57.15V" in text for text in texts)
