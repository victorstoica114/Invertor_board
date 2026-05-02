# Test Suite for ESP32 CAN/RS485 Bridge

This directory contains sanity checks, host-based unit tests, integration tests, protocol fixtures, and coverage plumbing for the firmware.

## Structure

```text
tests/
  sanity/
    test_repo_sanity.py
  unit/
    test_can_decoder.c
    test_modbus_decoder.c
    test_route_selection.c
    test_host_unit_coverage.py
    esp_stub/
    host_stubs.c
  integration/
    test_firmware_configuration.py
    test_protocol_fixtures.py
    fixtures/protocol_samples.py
  firmware_build/
    test_build_artifacts.py
  requirements.txt
```

## Local Setup

Install Python dependencies:

```bash
python -m pip install -r tests/requirements.txt
```

The host C unit tests also need `gcc` and `gcov`. If they are missing on a developer machine, those tests skip; in CI they fail so coverage cannot silently disappear.

## Run Sanity Tests

```bash
python -m pytest tests/sanity -v
```

Sanity tests verify the repository shape, CI image pinning, expected test entrypoints, ignored generated outputs, and absence of tracked build/coverage artifacts.

## Run Unit Tests

```bash
python -m pytest tests/unit/test_host_unit_coverage.py -v
```

Unit tests compile and run the host C test targets for CAN decoder, Modbus decoder, and route selection. They also create gcov data for coverage.

## Run Integration Tests

```bash
python -m pytest tests/integration -v
```

Integration tests verify:

- firmware configuration
- protocol constants and repository structure
- protocol fixture shape and Modbus CRC helpers

## Run Build Tests

```bash
idf.py set-target esp32c6
idf.py build
python -m pytest tests/firmware_build -v
```

Build tests run after the firmware build and verify generated `.elf`/`.bin` artifacts, `sdkconfig`, and binary size.

## Host C Coverage

`tests/unit/test_host_unit_coverage.py` compiles the C unit tests with `--coverage`, runs them, and writes gcov output under:

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

`.gitlab-ci.yml` separates the test layers:

- `sanity_tests`: fast repo and CI checks
- `unit_tests`: host C unit tests and Cobertura coverage
- `integration_tests`: firmware configuration and protocol fixture checks
- `build_firmware`: final ESP32-C6 firmware build and build artifact checks

CI publishes JUnit reports for all pytest layers and Cobertura/HTML coverage artifacts from `unit_tests`.

## Testing Philosophy

Prefer fast host tests for protocol parsing, cache behavior, route selection, and edge cases. Keep hardware timing, UART electrical behavior, and real CAN/RS485 bus characteristics out of host tests unless there is a dedicated hardware-in-loop setup.
