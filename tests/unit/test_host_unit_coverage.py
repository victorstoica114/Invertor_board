"""
Build and run the host-based C unit tests with coverage instrumentation.

These tests deliberately compile production C sources on the host instead of
requiring ESP-IDF hardware test support. On developer machines without a C
compiler they skip; in CI they fail fast so missing coverage is visible.
"""

import os
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
BUILD_ROOT = Path(
    os.environ.get("HOST_TEST_BUILD_DIR", REPO_ROOT / "tests" / ".build" / "host_unit")
)
COVERAGE_ROOT = BUILD_ROOT / "coverage"


@dataclass(frozen=True)
class HostCTestTarget:
    name: str
    sources: tuple[str, ...]


HOST_C_TESTS = (
    HostCTestTarget(
        name="can_decoder",
        sources=(
            "tests/unit/test_can_decoder.c",
            "tests/unit/host_stubs.c",
            "main/decoders/CAN_Decoder.c",
            "main/protocols/pylon/pylon_can_protocol.c",
            "main/protocols/deye/deye_can_protocol.c",
            "main/protocols/jkbms_can/jkbms_can_protocol.c",
        ),
    ),
    HostCTestTarget(
        name="modbus_decoder",
        sources=(
            "tests/unit/test_modbus_decoder.c",
            "main/decoders/modbusDecoder.c",
        ),
    ),
    HostCTestTarget(
        name="jkbms_modbus_freshness",
        sources=(
            "tests/unit/test_jkbms_modbus_freshness.c",
            "main/protocols/jkbms_modbus/jkbms_modbus_freshness.c",
            "main/decoders/modbusDecoder.c",
        ),
    ),
    HostCTestTarget(
        name="jkbms_modbus_alerts",
        sources=(
            "tests/unit/test_jkbms_modbus_alerts.c",
            "main/protocols/jkbms_modbus/jkbms_modbus_alerts.c",
        ),
    ),
    HostCTestTarget(
        name="jkbms_rs485_native",
        sources=(
            "tests/unit/test_jkbms_rs485_native.c",
            "main/protocols/jkbms_rs485/jkbms_rs485_native.c",
        ),
    ),
    HostCTestTarget(
        name="pace_modbus",
        sources=(
            "tests/unit/test_pace_modbus.c",
            "tests/unit/pace_modbus_stubs.c",
            "main/protocols/pace_modbus/pace_modbus_bms_task.c",
            "main/protocols/pace_modbus/pace_modbus_poller.c",
            "main/protocols/pace_modbus/pace_modbus_registers_map.c",
            "main/decoders/modbusDecoder.c",
        ),
    ),
    HostCTestTarget(
        name="rs485_growatt",
        sources=(
            "tests/unit/test_rs485_growatt.c",
            "tests/unit/pace_modbus_stubs.c",
            "main/protocols/rs485_growatt/rs485_growatt_bms_task.c",
            "main/protocols/rs485_growatt/rs485_growatt_modbus_poller.c",
            "main/protocols/rs485_growatt/rs485_growatt_registers_map.c",
            "main/decoders/modbusDecoder.c",
        ),
    ),
    HostCTestTarget(
        name="voltronic_modbus",
        sources=(
            "tests/unit/test_voltronic_modbus.c",
            "tests/unit/pace_modbus_stubs.c",
            "main/protocols/voltronic_modbus/voltronic_modbus_bms_task.c",
            "main/protocols/voltronic_modbus/voltronic_modbus_poller.c",
            "main/protocols/voltronic_modbus/voltronic_modbus_registers_map.c",
            "main/decoders/modbusDecoder.c",
        ),
    ),
    HostCTestTarget(
        name="china_tower_modbus",
        sources=(
            "tests/unit/test_china_tower_modbus.c",
            "tests/unit/pace_modbus_stubs.c",
            "main/protocols/china_tower_modbus/china_tower_modbus_bms_task.c",
            "main/protocols/china_tower_modbus/china_tower_modbus_poller.c",
            "main/protocols/china_tower_modbus/china_tower_modbus_registers_map.c",
            "main/decoders/modbusDecoder.c",
        ),
    ),
    HostCTestTarget(
        name="wow_modbus",
        sources=(
            "tests/unit/test_wow_modbus.c",
            "tests/unit/pace_modbus_stubs.c",
            "main/protocols/wow_modbus/wow_modbus_bms_task.c",
            "main/protocols/pace_modbus/pace_modbus_bms_task.c",
            "main/protocols/pace_modbus/pace_modbus_poller.c",
            "main/protocols/pace_modbus/pace_modbus_registers_map.c",
            "main/decoders/modbusDecoder.c",
        ),
    ),
    HostCTestTarget(
        name="route_selection",
        sources=(
            "tests/unit/test_route_selection.c",
            "tests/unit/route_selection_stubs.c",
            "main/orchestrator/orchestrator.c",
        ),
    ),
    HostCTestTarget(
        name="pylon_rs485_bridge",
        sources=(
            "tests/unit/test_pylon_rs485_bridge.c",
            "tests/unit/pylon_rs485_bridge_stubs.c",
            "main/protocols/pylon/pylon_rs485_bridge.c",
        ),
    ),
    HostCTestTarget(
        name="battery_can_protocols",
        sources=(
            "tests/unit/test_battery_and_can_protocols.c",
            "main/protocols/common/battery_model.c",
            "main/protocols/pylon/pylon_can_protocol.c",
            "main/protocols/deye/deye_can_protocol.c",
            "main/protocols/jkbms_can/jkbms_can_protocol.c",
        ),
    ),
)

COMMON_CFLAGS = (
    "-std=c11",
    "-O0",
    "-g",
    "-Wall",
    "-Wextra",
    "-D_POSIX_C_SOURCE=200809L",
    "-DHOST_TEST",
    "-ffunction-sections",
    "-fdata-sections",
    "--coverage",
)
COMMON_LDFLAGS = (
    "-Wl,--gc-sections",
    "--coverage",
)
COMMON_INCLUDES = (
    "main",
    "tests/unit",
    "tests/unit/esp_stub",
)


def _is_ci() -> bool:
    return os.environ.get("CI", "").lower() in {"1", "true", "yes"} or "GITLAB_CI" in os.environ


def _require_gcc() -> str:
    cc = os.environ.get("CC") or shutil.which("gcc") or shutil.which("cc")
    if cc:
        return cc

    message = "gcc is required for host C integration tests and coverage"
    if _is_ci():
        pytest.fail(message)
    pytest.skip(message)


def _run(cmd: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def _object_path(target_dir: Path, source: str) -> Path:
    src = Path(source)
    return target_dir / f"{src.stem}.o"


def _compile_target(cc: str, target: HostCTestTarget) -> Path:
    target_dir = BUILD_ROOT / target.name
    if target_dir.exists():
        shutil.rmtree(target_dir)
    target_dir.mkdir(parents=True, exist_ok=True)

    include_args = [f"-I{REPO_ROOT / include_dir}" for include_dir in COMMON_INCLUDES]
    objects: list[Path] = []

    for source in target.sources:
        src_path = REPO_ROOT / source
        obj_path = _object_path(target_dir, source)
        cmd = [
            cc,
            *COMMON_CFLAGS,
            *include_args,
            "-c",
            str(src_path),
            "-o",
            str(obj_path),
        ]
        result = _run(cmd, target_dir)
        assert result.returncode == 0, f"failed compiling {source}\n{result.stdout}"
        objects.append(obj_path)

    exe_suffix = ".exe" if os.name == "nt" else ""
    exe_path = target_dir / f"{target.name}{exe_suffix}"
    cmd = [cc, *COMMON_LDFLAGS, *(str(obj) for obj in objects), "-o", str(exe_path)]
    result = _run(cmd, target_dir)
    assert result.returncode == 0, f"failed linking {target.name}\n{result.stdout}"
    return exe_path


@pytest.fixture(scope="session")
def host_c_test_results() -> dict[str, Path]:
    cc = _require_gcc()
    if BUILD_ROOT.exists():
        shutil.rmtree(BUILD_ROOT)
    BUILD_ROOT.mkdir(parents=True, exist_ok=True)
    COVERAGE_ROOT.mkdir(parents=True, exist_ok=True)

    results: dict[str, Path] = {}
    for target in HOST_C_TESTS:
        exe_path = _compile_target(cc, target)
        result = _run([str(exe_path)], exe_path.parent)
        assert result.returncode == 0, f"{target.name} failed\n{result.stdout}"
        results[target.name] = exe_path

    return results


def test_host_c_unit_suites_pass(host_c_test_results: dict[str, Path]) -> None:
    assert set(host_c_test_results) == {target.name for target in HOST_C_TESTS}


def test_host_c_coverage_data_created(host_c_test_results: dict[str, Path]) -> None:
    for name, exe_path in host_c_test_results.items():
        gcda_files = list(exe_path.parent.glob("*.gcda"))
        assert gcda_files, f"{name} did not produce gcov data"


def test_host_c_gcov_summary(host_c_test_results: dict[str, Path]) -> None:
    gcov = os.environ.get("GCOV") or shutil.which("gcov")
    if not gcov:
        if _is_ci():
            pytest.fail("gcov is required to emit host C coverage summaries")
        pytest.skip("gcov is not installed")

    summaries: list[str] = []
    for target in HOST_C_TESTS:
        target_dir = host_c_test_results[target.name].parent
        for source in target.sources:
            obj_path = _object_path(target_dir, source)
            result = _run(
                [gcov, "-b", "-c", "-o", str(target_dir), str(obj_path)],
                COVERAGE_ROOT,
            )
            assert result.returncode == 0, f"gcov failed for {source}\n{result.stdout}"
            summaries.append(f"## {target.name}: {source}\n{result.stdout.strip()}\n")

    (COVERAGE_ROOT / "host-c-coverage.txt").write_text("\n".join(summaries), encoding="utf-8")
