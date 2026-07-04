import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "sma_can"
)
PULSEVIEW_DECODER_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode\decoders")
PULSEVIEW_SRD_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode")
sys.path.insert(0, str(DECODER_DIR))

from sma_can import describe_packet, frame_summary  # noqa: E402


def test_sma_can_describes_limit_frame_351():
    packet = (
        "standard",
        0x351,
        "data",
        8,
        [0x9E, 0x02, 0x7C, 0x01, 0x6C, 0x07, 0xC6, 0x01],
    )

    text = describe_packet(packet)

    assert "0x351 SMA limits" in text
    assert "chgV=67.0V" in text
    assert "chgI=38.0A" in text
    assert "disI=190.0A" in text
    assert "lowV=45.4V" in text
    assert "SMA CAN v2026.07.04a 0x351" in frame_summary(packet)


def test_sma_can_describes_soc_and_pack_frames():
    soc = describe_packet(("standard", 0x355, "data", 8,
                           [0x5D, 0x00, 0x64, 0x00, 0, 0, 0, 0]))
    pack = describe_packet(("standard", 0x356, "data", 8,
                            [0x4E, 0x16, 0xFF, 0xFF, 0xF0, 0x00, 0, 0]))

    assert "SOC=93%" in soc
    assert "SOH=100%" in soc
    assert "V=57.10V" in pack
    assert "I=-0.1A" in pack
    assert "temp=24.0C" in pack


def test_sma_can_describes_identity_and_raw_frames():
    manufacturer = describe_packet(("standard", 0x35E, "data", 8,
                                    [ord("S"), ord("M"), ord("A"), 0, 0, 0, 0, 0]))
    alarms = describe_packet(("standard", 0x359, "data", 8,
                              [0x01, 0x00, 0x20, 0x00, 0, 0, 0, 0]))
    battery_info = describe_packet(("standard", 0x35F, "data", 8,
                                    [0x10, 0, 0x20, 0, 0, 0, 0, 0]))

    assert "manufacturer 'SMA'" in manufacturer
    assert "0x359 SMA alarms/status raw=01 00 20 00 00 00 00 00" in alarms
    assert "u16=[1, 32, 0, 0]" in alarms
    assert "0x35F SMA battery info" in battery_info


def test_sigrok_sma_can_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "sma_can", "sma_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("sma_can")

    assert module.Decoder.id == "sma_can"
    assert module.Decoder.name == "SMA CAN v2026.07.04a"
    assert module.Decoder.inputs == ["logic"]
    assert any(option["id"] == "input_mode" for option in module.Decoder.options)


def test_sigrok_sma_can_decoder_derives_bus_level_from_raw_can_lines(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "sma_can", "sma_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("sma_can")
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


def test_sigrok_sma_can_decoder_emits_annotations_from_internal_can_packet(monkeypatch):
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

    for name in ("can", "can.pd", "sma_can", "sma_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("sma_can")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()
    decoder.ss_packet = 10
    decoder.es_packet = 20

    decoder.putpy(("standard", 0x356, "data", 8,
                   [0x4E, 0x16, 0x00, 0x00, 0xF0, 0x00, 0, 0]))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("SMA CAN v2026.07.04a 0x356" in text for text in texts)
    assert any("4E 16 00 00 F0 00 00 00" in text for text in texts)
    assert any("V=57.10V" in text for text in texts)
