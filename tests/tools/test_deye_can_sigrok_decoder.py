import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "deye_can"
)
PULSEVIEW_DECODER_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode\decoders")
PULSEVIEW_SRD_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode")
sys.path.insert(0, str(DECODER_DIR))

from deye_can import describe_packet, frame_summary  # noqa: E402


def test_deye_can_describes_limit_frame_351():
    packet = (
        "standard",
        0x351,
        "data",
        8,
        [0x30, 0x02, 0x2C, 0x01, 0x90, 0x01, 0xC2, 0x01],
    )

    text = describe_packet(packet)

    assert "0x351 limits" in text
    assert "chgV=56.0V" in text
    assert "chgI=30.0A" in text
    assert "disI=40.0A" in text
    assert "lowV=45.0V" in text
    assert "Deye CAN v2026.07.03a 0x351" in frame_summary(packet)


def test_deye_can_describes_soc_pack_and_state_frames():
    soc = describe_packet(("standard", 0x355, "data", 4, [0x63, 0x00, 0x64, 0x00]))
    pack = describe_packet(("standard", 0x356, "data", 8,
                            [0x4C, 0x16, 0xFF, 0xFF, 0xF5, 0x00, 0, 0]))
    state = describe_packet(("standard", 0x35C, "data", 8,
                             [0xE0, 0, 0, 0, 0, 0, 0, 0]))

    assert "SOC=99%" in soc
    assert "SOH=100%" in soc
    assert "V=57.08V" in pack
    assert "I=-0.1A" in pack
    assert "temp=24.5C" in pack
    assert "charge=ON" in state
    assert "discharge=ON" in state
    assert "balance=ON" in state


def test_deye_can_describes_temperature_cell_and_index_frames():
    temp_cell = describe_packet(("standard", 0x370, "data", 8,
                                 [0xFA, 0x00, 0xF0, 0x00, 0xF4, 0x0D, 0xF1, 0x0D]))
    indexes = describe_packet(("standard", 0x371, "data", 8,
                               [0x02, 0, 0x04, 0, 0x05, 0, 0x0A, 0]))

    assert "temp max=25.0C" in temp_cell
    assert "min=24.0C" in temp_cell
    assert "cell_max=3.572V" in temp_cell
    assert "cell_min=3.569V" in temp_cell
    assert "temp_max_sensor=2" in indexes
    assert "temp_min_sensor=4" in indexes
    assert "cell_max_idx=5" in indexes
    assert "cell_min_idx=10" in indexes


def test_deye_can_describes_identity_and_module_info():
    identity = describe_packet(("standard", 0x35E, "data", 8,
                                [ord("J"), ord("K"), ord("-"), ord("B"),
                                 ord("M"), ord("S"), 0, 0]))
    module = describe_packet(("standard", 0x359, "data", 8,
                              [0, 0, 0, 0, 1, ord("J"), ord("K"), 0]))

    assert "identity 'JK-BMS'" in identity
    assert "modules=1" in module
    assert "tag='JK'" in module


def test_sigrok_deye_can_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "deye_can", "deye_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("deye_can")

    assert module.Decoder.id == "deye_can"
    assert module.Decoder.name == "Deye CAN v2026.07.03a"
    assert module.Decoder.inputs == ["logic"]
    assert any(option["id"] == "input_mode" for option in module.Decoder.options)


def test_sigrok_deye_can_decoder_derives_bus_level_from_raw_can_lines(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "deye_can", "deye_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("deye_can")
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


def test_sigrok_deye_can_decoder_emits_annotations_from_internal_can_packet(monkeypatch):
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

    for name in ("can", "can.pd", "deye_can", "deye_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("deye_can")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()
    decoder.ss_packet = 10
    decoder.es_packet = 20

    decoder.putpy(("standard", 0x356, "data", 8,
                   [0x4C, 0x16, 0x00, 0x00, 0xF5, 0x00, 0, 0]))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("Deye CAN v2026.07.03a 0x356" in text for text in texts)
    assert any("4C 16 00 00 F5 00 00 00" in text for text in texts)
    assert any("V=57.08V" in text for text in texts)
