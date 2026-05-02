# Test Suite for ESP32 CAN/RS485 Bridge

This directory contains host-based unit tests, pytest integration tests, protocol fixtures, and coverage plumbing for the firmware.

## Structure

```text
tests/
  unit/
    test_can_decoder.c
    test_modbus_decoder.c
    test_route_selection.c
    esp_stub/
    host_stubs.c
  integration/
    test_build_artifacts.py
    test_host_unit_coverage.py
    test_protocol_fixtures.py
    fixtures/protocol_samples.py
  requirements.txt
```

## Local Setup

Install Python dependencies:

```bash
python -m pip install -r tests/requirements.txt
```

The host C integration tests also need `gcc` and `gcov`. If they are missing on a developer machine, those tests skip; in CI they fail so coverage cannot silently disappear.

## Run Integration Tests

```bash
python -m pytest tests/integration -v
```

These tests verify:

- expected build artifacts and firmware configuration
- protocol constants and repository structure
- protocol fixture shape and Modbus CRC helpers
- host compilation/execution of C unit tests with coverage instrumentation

## Host C Coverage

`tests/integration/test_host_unit_coverage.py` compiles the C unit tests with `--coverage`, runs them, and writes gcov output under:

```text
tests/.build/host_unit/coverage/
```

To generate a Cobertura XML and HTML report manually after running pytest:

```bash
python -m gcovr --root . \
  --object-directory tests/.build/host_unit \
  --filter main \
  --filter tests/unit \
  --xml-pretty --output tests/.build/host_unit/coverage/cobertura.xml \
  --html-details tests/.build/host_unit/coverage/html/index.html \
  --print-summary
```

## Direct Host Unit Builds

The pytest integration suite is the preferred runner, but individual C tests can still be compiled directly when `gcc` is available:

```bash
gcc -O0 -g -Wall -Wextra -I../../main -I. -Iesp_stub \
  -o /tmp/test_modbus_decoder \
  test_modbus_decoder.c ../../main/decoders/modbusDecoder.c
/tmp/test_modbus_decoder
```

## CI

`.gitlab-ci.yml` builds the ESP32-C6 firmware, runs all pytest integration tests, and publishes:

- JUnit test report: `tests/integration/junit.xml`
- Cobertura coverage report: `tests/.build/host_unit/coverage/cobertura.xml`
- HTML/gcov coverage artifacts under `tests/.build/host_unit/coverage/`

## Testing Philosophy

Prefer fast host tests for protocol parsing, cache behavior, route selection, and edge cases. Keep hardware timing, UART electrical behavior, and real CAN/RS485 bus characteristics out of host tests unless there is a dedicated hardware-in-loop setup.
