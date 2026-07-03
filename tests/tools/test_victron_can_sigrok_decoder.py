import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "victron_can"
)
PULSEVIEW_DECODER_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode\decoders")
PULSEVIEW_SRD_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode")
sys.path.insert(0, str(DECODER_DIR))

from victron_can import VERSION, describe_packet, frame_summary  # noqa: E402


def test_victron_can_describes_live_limit_frame_351():
    packet = (
        "standard",
        0x351,
        "data",
        8,
        [0x9E, 0x02, 0x7C, 0x01, 0x6C, 0x07, 0xC6, 0x01],
    )

    text = describe_packet(packet)

    assert "0x351 limits" in text
    assert "chgV=67.0V" in text
    assert "chgI=38.0A" in text
    assert "disI=190.0A" in text
    assert "lowV=45.4V" in text
    assert "charge=ON" in text
    assert "discharge=ON" in text
    assert "Victron CAN v2026.07.03a 0x351" in frame_summary(packet)


def test_victron_can_describes_live_soc_pack_frames():
    soc = describe_packet(("standard", 0x355, "data", 8,
                           [0x63, 0x00, 0x64, 0x00, 0x8E, 0x26, 0x00, 0x00]))
    pack = describe_packet(("standard", 0x356, "data", 8,
                            [0x53, 0x16, 0x00, 0x00, 0x36, 0x01, 0x00, 0x00]))

    assert "SOC=99%" in soc
    assert "SOH=100%" in soc
    assert "raw_tail=8E 26 00 00" in soc
    assert "V=57.15V" in pack
    assert "I=+0.0A" in pack
    assert "temp=31.0C" in pack


def test_victron_can_describes_live_cell_temp_and_ascii_frames():
    cell = describe_packet(("standard", 0x373, "data", 8,
                            [0xF4, 0x0D, 0xF5, 0x0D, 0x2F, 0x01, 0x30, 0x01]))
    manufacturer = describe_packet(("standard", 0x35E, "data", 8,
                                    [ord("J"), ord("K"), ord("-"), ord("B"),
                                     ord("M"), ord("S"), 0, 0]))
    text = describe_packet(("standard", 0x374, "data", 8,
                            [0x30, 0x30, 0x3A, 0x3A, 0x30, 0x34, 0, 0]))

    assert "tentative Pylon-style" in cell
    assert "cell_min=3.572V" in cell
    assert "cell_max=3.573V" in cell
    assert "t1=30.3C" in cell
    assert "t2=30.4C" in cell
    assert "ASCII 'JK-BMS'" in manufacturer
    assert "ASCII '00::04'" in text


def test_victron_can_keeps_unknown_frames_raw():
    vendor = describe_packet(("standard", 0x35A, "data", 8,
                              [0xA8, 0xA8, 0x02, 0x00, 0xA8, 0xA8, 0x02, 0x02]))
    battery = describe_packet(("standard", 0x35F, "data", 8,
                               [0x69, 0x4C, 0x0F, 0x26, 0x27, 0x00, 0x00, 0x00]))

    assert "vendor/raw" in vendor
    assert "A8 A8 02 00 A8 A8 02 02" in vendor
    assert "battery/raw" in battery
    assert "u16=[" in battery


def test_sigrok_victron_can_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "victron_can", "victron_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("victron_can")

    assert module.Decoder.id == "victron_can"
    assert module.Decoder.name == "Victron CAN {}".format(VERSION)
    assert module.Decoder.inputs == ["logic"]
    assert any(option["id"] == "input_mode" for option in module.Decoder.options)


def test_sigrok_victron_can_decoder_derives_bus_level_from_raw_can_lines(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "victron_can", "victron_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("victron_can")
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


def test_sigrok_victron_can_decoder_emits_annotations_from_internal_can_packet(monkeypatch):
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

    for name in ("can", "can.pd", "victron_can", "victron_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("victron_can")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()
    decoder.ss_packet = 10
    decoder.es_packet = 20

    decoder.putpy(("standard", 0x356, "data", 8,
                   [0x53, 0x16, 0x00, 0x00, 0x36, 0x01, 0, 0]))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]

    assert any("Victron CAN v2026.07.03a 0x356" in text for text in texts)
    assert any("53 16 00 00 36 01 00 00" in text for text in texts)
    assert any("V=57.15V" in text for text in texts)
