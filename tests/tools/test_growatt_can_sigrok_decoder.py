import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "growatt_can"
)
PULSEVIEW_DECODER_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode\decoders")
PULSEVIEW_SRD_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode")
sys.path.insert(0, str(DECODER_DIR))

from growatt_can import describe_packet, frame_summary  # noqa: E402


def test_growatt_can_describes_main_313_frame():
    packet = (
        "standard",
        0x313,
        "data",
        8,
        [0x14, 0x78, 0x00, 0x64, 0x00, 0xFA, 75, 98],
    )

    text = describe_packet(packet)

    assert "0x313 pack" in text
    assert "V=52.40V" in text
    assert "I=+10.0A" in text
    assert "Tavg=25.0C" in text
    assert "SOC=75%" in text
    assert "SOH=98%" in text
    assert "Growatt CAN 0x313" in frame_summary(packet)


def test_growatt_can_accepts_little_endian_live_style_313_frame():
    packet = (
        "standard",
        0x313,
        "data",
        8,
        [0x66, 0x1C, 0x00, 0x00, 0x1F, 0x01, 92, 99],
    )

    text = describe_packet(packet)

    assert "V=72.70V" in text
    assert "I=+0.0A" in text
    assert "Tavg=28.7C" in text
    assert "SOC=92%" in text


def test_growatt_can_describes_cell_extremes_319_frame():
    packet = (
        "standard",
        0x319,
        "data",
        8,
        [0xC0, 0x12, 0x0A, 0x11, 0x7C, 3, 12, 0],
    )

    text = describe_packet(packet)

    assert "0x319" in text
    assert "cell_max=4.618V#3" in text
    assert "cell_min=4.476V#12" in text
    assert "dV=142mV" in text


def test_sigrok_growatt_can_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "growatt_can", "growatt_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("growatt_can")

    assert module.Decoder.id == "growatt_can"
    assert module.Decoder.inputs == ["logic"]
    assert any(option["id"] == "input_mode" for option in module.Decoder.options)


def test_sigrok_growatt_can_decoder_derives_bus_level_from_raw_can_lines(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "growatt_can", "growatt_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("growatt_can")
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


def test_sigrok_growatt_can_decoder_emits_annotations_from_internal_can_packet(monkeypatch):
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

    for name in ("can", "can.pd", "growatt_can", "growatt_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("growatt_can")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()
    decoder.ss_packet = 10
    decoder.es_packet = 20

    decoder.putpy(("standard", 0x313, "data", 8,
                   [0x14, 0x78, 0x00, 0x64, 0x00, 0xFA, 75, 98]))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("Growatt CAN 0x313" in text for text in texts)
    assert any("14 78 00 64 00 FA 4B 62" in text for text in texts)
    assert any("SOC=75%" in text for text in texts)
