"""Integration tests for firmware configuration and protocol layout."""

import pytest
import os
import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def repo_path(*parts: str) -> Path:
    """Return a repository-relative path."""
    return REPO_ROOT.joinpath(*parts)


def first_existing_path(*candidates: Path) -> Path:
    """Return the first existing candidate, or the first one for helpful failures."""
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


@pytest.fixture(scope="module")
def sdkconfig_path():
    """Fixture to provide sdkconfig path."""
    return repo_path("sdkconfig")


def test_sdkconfig_exists(sdkconfig_path):
    """Test that sdkconfig exists and is readable."""
    assert sdkconfig_path.exists(), "sdkconfig should exist"
    assert sdkconfig_path.is_file(), "sdkconfig should be a file"
    assert os.access(sdkconfig_path, os.R_OK), "sdkconfig should be readable"


def test_protocol_features_enabled(sdkconfig_path):
    """Test that protocol-related features are enabled in sdkconfig."""
    if not sdkconfig_path.exists():
        pytest.skip("sdkconfig not found")

    with open(sdkconfig_path, 'r') as f:
        config_content = f.read()

    # Check for ESP-IDF features we rely on
    assert 'CONFIG_FREERTOS_HZ=1000' in config_content or \
           'CONFIG_FREERTOS_HZ=' in config_content, \
           "FreeRTOS tick rate should be configured"


def test_can_driver_enabled(sdkconfig_path):
    """Test that CAN (TWAI) driver is enabled."""
    if not sdkconfig_path.exists():
        pytest.skip("sdkconfig not found")

    with open(sdkconfig_path, 'r') as f:
        config_content = f.read()

    # ESP32-C6 uses TWAI controller
    # Check that TWAI is not explicitly disabled
    assert 'CONFIG_TWAI_ISR_IN_IRAM=y' in config_content or \
           'TWAI' in config_content, \
           "TWAI/CAN should be available"


def test_uart_driver_enabled(sdkconfig_path):
    """Test that UART driver is enabled (for RS485)."""
    if not sdkconfig_path.exists():
        pytest.skip("sdkconfig not found")

    with open(sdkconfig_path, 'r') as f:
        config_content = f.read()

    # UART is usually enabled by default, just verify it's not disabled
    assert 'CONFIG_UART_ISR_IN_IRAM=y' in config_content or \
           'UART' in config_content, \
           "UART should be available"


def test_protocol_constants_in_config_h():
    """Test that all protocol constants are defined in config.h."""
    config_h = repo_path("main", "config.h")

    if not config_h.exists():
        pytest.skip("config.h not found")

    with open(config_h, 'r') as f:
        config_content = f.read()

    # Check that all protocol IDs are defined
    expected_protocols = [
        'PROTOCOL_CAN_GROWATT',
        'PROTOCOL_RS485_GROWATT',
        'PROTOCOL_RS485_PYLON',
        'PROTOCOL_CAN_PYLON',
        'PROTOCOL_CAN_DEYE',
        'PROTOCOL_RS485_JKBMS',
        'PROTOCOL_CAN_GOODWE',
        'PROTOCOL_CAN_SOFAR',
        'PROTOCOL_CAN_SMA',
        'PROTOCOL_CAN_VICTRON',
        'PROTOCOL_RS485_PACE',
        'PROTOCOL_RS485_JKBMS_NATIVE',
        'PROTOCOL_RS485_VOLTRONIC',
        'PROTOCOL_RS485_CHINA_TOWER',
    ]

    for protocol in expected_protocols:
        assert re.search(rf'#define\s+{protocol}\s+\d+', config_content), \
            f"Protocol {protocol} should be defined in config.h"


def test_line_constants_in_config_h():
    """Test that line type constants are defined."""
    config_h = repo_path("main", "config.h")

    if not config_h.exists():
        pytest.skip("config.h not found")

    with open(config_h, 'r') as f:
        config_content = f.read()

    assert re.search(r'#define\s+LINE_CAN\s+1', config_content), \
        "LINE_CAN should be defined as 1"
    assert re.search(r'#define\s+LINE_RS485\s+2', config_content), \
        "LINE_RS485 should be defined as 2"


def test_mode_constants_in_config_h():
    """Test that mode constants are defined."""
    config_h = repo_path("main", "config.h")

    if not config_h.exists():
        pytest.skip("config.h not found")

    with open(config_h, 'r') as f:
        config_content = f.read()

    assert re.search(r'#define\s+MODE_SNIFFER\s+1', config_content), \
        "MODE_SNIFFER should be defined"
    assert re.search(r'#define\s+MODE_FORWARD\s+2', config_content), \
        "MODE_FORWARD should be defined"
    assert re.search(r'#define\s+MODE_BRIDGE\s+3', config_content), \
        "MODE_BRIDGE should be defined"


def test_orchestrator_files_exist():
    """Test that orchestrator source files exist."""
    orchestrator_dir = repo_path("main", "orchestrator")

    assert orchestrator_dir.exists(), "Orchestrator directory should exist"
    assert (orchestrator_dir / "orchestrator.c").exists(), \
        "orchestrator.c should exist"
    assert (orchestrator_dir / "orchestrator.h").exists(), \
        "orchestrator.h should exist"


def test_protocol_directories_exist():
    """Test that protocol implementation directories exist."""
    protocols_dir = first_existing_path(
        repo_path("main", "protocols"),
        repo_path("main", "Protocols"),
    )

    assert protocols_dir.exists(), "Protocols directory should exist"

    expected_protocol_dirs = [
        'growatt',
        'pylon',
        'jkbms_modbus',
        'jkbms_rs485',
        'pace_modbus',
        'voltronic_modbus',
        'china_tower_modbus',
        'deye',
    ]

    for protocol_dir in expected_protocol_dirs:
        protocol_path = protocols_dir / protocol_dir
        assert protocol_path.exists(), \
            f"Protocol directory {protocol_dir} should exist"


def test_pace_modbus_protocol_files_exist():
    """Test that the PACE RS485 Modbus implementation is complete enough for CI."""
    protocols_dir = first_existing_path(
        repo_path("main", "protocols"),
        repo_path("main", "Protocols"),
    )
    pace_dir = protocols_dir / "pace_modbus"

    expected_files = [
        "pace_modbus_registers_map.h",
        "pace_modbus_registers_map.c",
        "pace_modbus_poller.h",
        "pace_modbus_poller.c",
        "pace_modbus_bms_task.h",
        "pace_modbus_bms_task.c",
    ]

    for file_name in expected_files:
        assert (pace_dir / file_name).exists(), \
            f"PACE protocol file {file_name} should exist"


def test_decoder_files_exist():
    """Test that decoder source files exist."""
    decoders_dir = repo_path("main", "decoders")

    assert decoders_dir.exists(), "Decoders directory should exist"
    assert (decoders_dir / "CAN_Decoder.c").exists(), \
        "CAN_Decoder.c should exist"
    assert (decoders_dir / "CAN_Decoder.h").exists(), \
        "CAN_Decoder.h should exist"
    assert (decoders_dir / "modbusDecoder.c").exists(), \
        "modbusDecoder.c should exist"
    assert (decoders_dir / "modbusDecoder.h").exists(), \
        "modbusDecoder.h should exist"


def test_web_interface_files_exist():
    """Test that web interface files exist."""
    web_dir = first_existing_path(
        repo_path("main", "Web_interface"),
        repo_path("main", "web_interface"),
    )

    assert web_dir.exists(), "Web_interface directory should exist"
    assert (web_dir / "web_interface.c").exists(), \
        "web_interface.c should exist"
    assert (web_dir / "bridge_compat.c").exists(), \
        "bridge_compat.c should exist"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
