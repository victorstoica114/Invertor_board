import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "jkbms_can"
)
PULSEVIEW_DECODER_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode\decoders")
PULSEVIEW_SRD_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode")
sys.path.insert(0, str(DECODER_DIR))

from jkbms_can import describe_packet, frame_summary, is_known_frame_id  # noqa: E402


def test_jkbms_can_describes_native_battery_status_frame():
    packet = (
        "extended",
        0x02F4,
        "data",
        8,
        [0xD7, 0x02, 0xA0, 0x0F, 80, 0, 100, 0],
    )

    text = describe_packet(packet)

    assert "0x2F0 pack" in text
    assert "V=72.7V" in text
    assert "I=+0.0A" in text
    assert "SOC=80%" in text
    assert "JKBMS CAN 0x2F4" in frame_summary(packet)
    assert "node=0" in frame_summary(packet)


def test_jkbms_can_describes_cell_extremes_and_extended_cell_voltage_frames():
    extremes = (
        "extended",
        0x04F4,
        "data",
        8,
        [0x0A, 0x12, 3, 0x7C, 0x11, 12, 0, 0],
    )
    cells = (
        "extended",
        0x18E028F4,
        "data",
        8,
        [0xB0, 0x11, 0xCF, 0x11, 0x0A, 0x12, 0x9D, 0x11],
    )

    assert "cell_max=4.618V#3" in describe_packet(extremes)
    assert "cell_min=4.476V#12" in describe_packet(extremes)
    assert "C01=4.528V" in describe_packet(cells)
    assert "C03=4.618V" in describe_packet(cells)
    assert is_known_frame_id(0x18E028F0)
    assert is_known_frame_id(0x18E028F4)


def test_jkbms_can_describes_extended_temperatures():
    packet = (
        "extended",
        0x18F228F4,
        "data",
        8,
        [0x1F, 76, 77, 79, 75, 78, 0, 0],
    )

    text = describe_packet(packet)

    assert "mask=0x1F" in text
    assert "T1=26.0C" in text
    assert "T5=28.0C" in text


def test_sigrok_jkbms_can_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "jkbms_can", "jkbms_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("jkbms_can")

    assert module.Decoder.id == "jkbms_can"
    assert module.Decoder.inputs == ["logic"]
    assert any(option["id"] == "input_mode" for option in module.Decoder.options)


def test_sigrok_jkbms_can_decoder_derives_bus_level_from_raw_can_lines(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "jkbms_can", "jkbms_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("jkbms_can")
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


def test_sigrok_jkbms_can_decoder_emits_annotations_from_internal_can_packet(monkeypatch):
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

    for name in ("can", "can.pd", "jkbms_can", "jkbms_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("jkbms_can")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()
    decoder.ss_packet = 10
    decoder.es_packet = 20

    decoder.putpy(("extended", 0x02F4, "data", 8,
                   [0xD7, 0x02, 0xA0, 0x0F, 80, 0, 100, 0]))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("JKBMS CAN 0x2F4" in text for text in texts)
    assert any("D7 02 A0 0F 50 00 64 00" in text for text in texts)
    assert any("SOC=80%" in text for text in texts)
