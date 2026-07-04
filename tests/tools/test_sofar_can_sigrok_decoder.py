import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "sofar_can"
)
PULSEVIEW_DECODER_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode\decoders")
PULSEVIEW_SRD_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode")
sys.path.insert(0, str(DECODER_DIR))

from sofar_can import describe_packet, frame_summary  # noqa: E402


def test_sofar_can_describes_limit_frame_351():
    packet = (
        "standard",
        0x351,
        "data",
        8,
        [0x9E, 0x02, 0x7C, 0x01, 0x6C, 0x07, 0xC6, 0x01],
    )

    text = describe_packet(packet)

    assert "0x351 Sofar limits" in text
    assert "chgV=67.0V" in text
    assert "chgI=38.0A" in text
    assert "disI=190.0A" in text
    assert "lowV=45.4V" in text
    assert "Sofar CAN v2026.07.04a 0x351" in frame_summary(packet)


def test_sofar_can_describes_soc_pack_and_status_frames():
    soc = describe_packet(("standard", 0x355, "data", 8,
                           [0x46, 0x00, 0x64, 0x00, 0, 0, 0, 0]))
    pack = describe_packet(("standard", 0x356, "data", 8,
                            [0xFA, 0x15, 0x00, 0x00, 0xF6, 0x00, 0, 0]))
    status = describe_packet(("standard", 0x35C, "data", 8,
                              [0xC0, 0, 0, 0, 0, 0, 0, 0]))

    assert "SOC=70%" in soc
    assert "SOH=100%" in soc
    assert "V=56.26V" in pack
    assert "I=+0.0A" in pack
    assert "temp=24.6C" in pack
    assert "charge=ON" in status
    assert "discharge=ON" in status
    assert "balance=OFF" in status


def test_sofar_can_describes_temperature_cell_and_index_frames():
    extremes = describe_packet(("standard", 0x370, "data", 8,
                                [0x18, 0x00, 0x17, 0x00, 0xF9, 0x0D, 0xF3, 0x0D]))
    indexes = describe_packet(("standard", 0x371, "data", 8,
                               [0x01, 0, 0x02, 0, 0x01, 0, 0x02, 0]))

    assert "temp max=24.0C" in extremes
    assert "min=23.0C" in extremes
    assert "cell_max=3.577V" in extremes
    assert "cell_min=3.571V" in extremes
    assert "temp_max_sensor=1" in indexes
    assert "temp_min_sensor=2" in indexes
    assert "cell_max_idx=1" in indexes
    assert "cell_min_idx=2" in indexes


def test_sofar_can_describes_identity_and_raw_frames():
    brand = describe_packet(("standard", 0x35E, "data", 8,
                             [ord("S"), ord("O"), ord("F"), ord("A"), ord("R"), 0, 0, 0]))
    module = describe_packet(("standard", 0x35F, "data", 8,
                              [0x10, 0, 0x20, 0, 0, 0, 0, 0]))
    info = describe_packet(("standard", 0x359, "data", 8,
                            [0, 0, 0, 0, 1, ord("B"), ord("M"), ord("S")]))

    assert "brand 'SOFAR'" in brand
    assert "0x35F Sofar module" in module
    assert "u16=[16, 32, 0, 0]" in module
    assert "modules=1" in info
    assert "tag='BMS'" in info


def test_sigrok_sofar_can_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "sofar_can", "sofar_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("sofar_can")

    assert module.Decoder.id == "sofar_can"
    assert module.Decoder.name == "Sofar CAN v2026.07.04a"
    assert module.Decoder.inputs == ["logic"]
    assert any(option["id"] == "input_mode" for option in module.Decoder.options)


def test_sigrok_sofar_can_decoder_derives_bus_level_from_raw_can_lines(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "sofar_can", "sofar_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("sofar_can")
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


def test_sigrok_sofar_can_decoder_emits_annotations_from_internal_can_packet(monkeypatch):
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

    for name in ("can", "can.pd", "sofar_can", "sofar_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("sofar_can")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()
    decoder.ss_packet = 10
    decoder.es_packet = 20

    decoder.putpy(("standard", 0x356, "data", 8,
                   [0xFA, 0x15, 0x00, 0x00, 0xF6, 0x00, 0, 0]))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("Sofar CAN v2026.07.04a 0x356" in text for text in texts)
    assert any("FA 15 00 00 F6 00 00 00" in text for text in texts)
    assert any("V=56.26V" in text for text in texts)
