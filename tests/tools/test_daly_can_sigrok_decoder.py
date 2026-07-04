import importlib
import sys
import types
from pathlib import Path


DECODER_DIR = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "sigrok"
    / "decoders"
    / "daly_can"
)
PULSEVIEW_DECODER_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode\decoders")
PULSEVIEW_SRD_DIR = Path(r"C:\Program Files\sigrok\PulseView\share\libsigrokdecode")
sys.path.insert(0, str(DECODER_DIR))

from daly_can import VERSION, describe_packet, frame_summary, is_known_frame_id  # noqa: E402


def test_daly_can_describes_request_and_response_ids():
    req = ("extended", 0x18900140, "data", 8, [0] * 8)
    rsp = ("extended", 0x18904001, "data", 8,
           [0x02, 0x3A, 0x00, 0x00, 0x75, 0x30, 0x03, 0xE0])

    assert is_known_frame_id(0x18900140)
    assert is_known_frame_id(0x18904001)
    assert "request pack voltage/current/SOC" in describe_packet(req)
    assert "id=0x18900140" in frame_summary(req)
    assert "req" in frame_summary(req)

    text = describe_packet(rsp)
    assert "pack V=57.0V" in text
    assert "I=+0.0A" in text
    assert "SOC=99.2%" in text
    assert "rsp" in frame_summary(rsp)


def test_daly_can_describes_cell_voltage_and_temperature_frames():
    cells = ("extended", 0x18954001, "data", 8,
             [0x01, 0x0D, 0xF1, 0x0D, 0xF2, 0x0D, 0xF3, 0x00])
    temps = ("extended", 0x18964001, "data", 8,
             [0x01, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48])

    assert "C01=3.569V" in describe_packet(cells)
    assert "C03=3.571V" in describe_packet(cells)
    assert "T01=26.0C" in describe_packet(temps)
    assert "T07=32.0C" in describe_packet(temps)


def test_daly_can_describes_status_and_alarms():
    status = ("extended", 0x18944001, "data", 8,
              [16, 2, 1, 1, 0, 0x03, 0xBD, 0])
    alarms = ("extended", 0x18984001, "data", 8,
              [0x12, 0x34, 0x56, 0x78, 0x01, 0x02, 0x03, 0])

    assert "cells=16" in describe_packet(status)
    assert "temps=2" in describe_packet(status)
    assert "cycles=957" in describe_packet(status)
    assert "alarm=0x12345678" in describe_packet(alarms)
    assert "warning=0x010203" in describe_packet(alarms)


def test_sigrok_daly_can_package_exports_decoder(monkeypatch):
    stub_sigrokdecode = types.SimpleNamespace(Decoder=object, OUTPUT_ANN=1, OUTPUT_PYTHON=2)
    monkeypatch.setitem(sys.modules, "sigrokdecode", stub_sigrokdecode)
    sys.path.insert(0, str(PULSEVIEW_SRD_DIR))
    sys.path.insert(0, str(PULSEVIEW_DECODER_DIR))
    sys.path.insert(0, str(DECODER_DIR.parent))

    for name in ("can", "can.pd", "daly_can", "daly_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("daly_can")

    assert module.Decoder.id == "daly_can"
    assert module.Decoder.inputs == ["logic"]
    assert VERSION in module.Decoder.name
    assert VERSION in module.Decoder.longname
    assert any(option["id"] == "input_mode" for option in module.Decoder.options)


def test_sigrok_daly_can_decoder_emits_annotations_from_internal_can_packet(monkeypatch):
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

    for name in ("can", "can.pd", "daly_can", "daly_can.pd"):
        sys.modules.pop(name, None)

    module = importlib.import_module("daly_can")
    decoder = module.Decoder()
    decoder.captured = []
    decoder.start()
    decoder.ss_packet = 10
    decoder.es_packet = 20

    decoder.putpy(("extended", 0x18904001, "data", 8,
                   [0x02, 0x3A, 0, 0, 0x75, 0x30, 0x03, 0xE0]))

    texts = [item[3][1][0] for item in decoder.captured if item[2] == stub_sigrokdecode.OUTPUT_ANN]
    assert any("Daly CAN" in text and "0x18904001" in text for text in texts)
    assert any("02 3A 00 00 75 30 03 E0" in text for text in texts)
    assert any("SOC=99.2%" in text for text in texts)
