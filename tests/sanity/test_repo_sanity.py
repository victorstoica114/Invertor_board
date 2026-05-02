"""Fast repository and CI sanity checks."""

import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def _read_repo_file(*parts: str) -> str:
    return REPO_ROOT.joinpath(*parts).read_text(encoding="utf-8")


def test_ci_uses_pinned_esp_idf_6_image() -> None:
    ci = _read_repo_file(".gitlab-ci.yml")

    assert "image: espressif/idf:v6.0.1" in ci
    assert "espressif/idf:release-v5.5" not in ci
    assert "idf.py --version" in ci


def test_esp_idf_6_json_dependency_uses_component_manager() -> None:
    cmake = _read_repo_file("main", "CMakeLists.txt")
    manifest = _read_repo_file("main", "idf_component.yml")

    assert "espressif/cjson" in manifest
    assert "        json" not in cmake


def test_esp_idf_6_split_driver_dependencies_are_declared() -> None:
    cmake = _read_repo_file("main", "CMakeLists.txt")

    for component in ("esp_driver_gpio", "esp_driver_uart", "esp_driver_twai"):
        assert component in cmake


def test_ci_declares_sanity_unit_and_integration_jobs() -> None:
    ci = _read_repo_file(".gitlab-ci.yml")

    for stage in ("sanity", "unit", "integration", "build"):
        assert f"  - {stage}" in ci

    for job in ("sanity_tests:", "unit_tests:", "integration_tests:", "build_firmware:"):
        assert job in ci


def test_ci_keeps_build_as_final_stage() -> None:
    ci = _read_repo_file(".gitlab-ci.yml")
    stages: list[str] = []
    in_stages_block = False

    for line in ci.splitlines():
        if line == "stages:":
            in_stages_block = True
            continue
        if in_stages_block and line and not line.startswith(" "):
            break
        if in_stages_block and line.startswith("  - "):
            stages.append(line.removeprefix("  - ").strip())

    assert stages == ["sanity", "unit", "integration", "build"]


def test_required_test_entrypoints_exist() -> None:
    expected_paths = (
        "tests/sanity/test_repo_sanity.py",
        "tests/unit/test_host_unit_coverage.py",
        "tests/unit/test_can_decoder.c",
        "tests/unit/test_modbus_decoder.c",
        "tests/unit/test_route_selection.c",
        "tests/integration/test_firmware_configuration.py",
        "tests/integration/test_protocol_fixtures.py",
        "tests/integration/fixtures/protocol_samples.py",
        "tests/firmware_build/test_build_artifacts.py",
    )

    missing = [path for path in expected_paths if not REPO_ROOT.joinpath(path).exists()]
    assert not missing


def test_gitignore_covers_generated_test_artifacts() -> None:
    gitignore = _read_repo_file(".gitignore")

    for pattern in ("tests/.build/", ".pytest_cache/", "*.gcda", "*.gcno", "*.gcov"):
        assert pattern in gitignore


def test_generated_outputs_are_not_tracked() -> None:
    result = subprocess.run(
        ["git", "ls-files"],
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert result.returncode == 0, result.stderr

    tracked_files = result.stdout.splitlines()
    generated = [
        path
        for path in tracked_files
        if path.startswith(("build/", "tests/.build/", ".pytest_cache/"))
        or path.endswith((".pyc", ".gcda", ".gcno", ".gcov"))
    ]

    assert not generated
