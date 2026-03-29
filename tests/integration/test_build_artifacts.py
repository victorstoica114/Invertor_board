from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def build_dir():
    path = Path("build")
    if not path.exists():
        pytest.skip("Build outputs missing; run `idf.py build` before integration tests.")
    return path


def test_expected_binaries_exist(build_dir: Path) -> None:
    expected = [
        build_dir / "project-name.bin",
        build_dir / "bootloader" / "bootloader.bin",
        build_dir / "partition_table" / "partition-table.bin",
    ]
    missing = [str(p) for p in expected if not p.exists()]
    assert not missing, f"Missing build artifacts: {', '.join(missing)}"


def test_sdkconfig_matches_target() -> None:
    sdkconfig = Path("sdkconfig")
    if not sdkconfig.exists():
        pytest.skip("sdkconfig missing; run `idf.py build` to generate it.")
    content = sdkconfig.read_text(encoding="utf-8")
    assert "CONFIG_IDF_TARGET_ESP32C6=y" in content, "sdkconfig target does not match esp32c6"
