"""Post-build checks for ESP-IDF firmware artifacts."""

import os
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def repo_path(*parts: str) -> Path:
    return REPO_ROOT.joinpath(*parts)


def test_build_directory_exists() -> None:
    build_dir = repo_path("build")

    assert build_dir.exists(), "build directory should exist after 'idf.py build'"
    assert build_dir.is_dir(), "build should be a directory"


def test_build_artifacts_exist() -> None:
    build_dir = repo_path("build")

    elf_outputs = list(build_dir.glob("*.elf"))
    bin_outputs = list(build_dir.glob("*.bin"))

    assert elf_outputs or bin_outputs, "at least one of .elf or .bin should exist"


def test_sdkconfig_exists_and_is_readable() -> None:
    sdkconfig_path = repo_path("sdkconfig")

    assert sdkconfig_path.exists(), "sdkconfig should exist"
    assert sdkconfig_path.is_file(), "sdkconfig should be a file"
    assert os.access(sdkconfig_path, os.R_OK), "sdkconfig should be readable"


def test_build_output_size_reasonable() -> None:
    build_dir = repo_path("build")
    bin_outputs = list(build_dir.glob("*.bin"))

    assert bin_outputs, "binary file should exist after 'idf.py build'"

    file_size = bin_outputs[0].stat().st_size

    assert 100 * 1024 < file_size < 4 * 1024 * 1024, (
        f"binary size {file_size} bytes seems unusual"
    )
