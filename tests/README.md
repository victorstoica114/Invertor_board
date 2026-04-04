# Test Suite for ESP32 CAN/RS485 Bridge

This directory contains the test suite for the ESP32 CAN/RS485 bridge firmware.

## Test Structure

```
tests/
├── unit/                  # Unit tests for individual components
│   ├── test_can_decoder.c        # CAN decoder tests
│   ├── test_modbus_decoder.c     # Modbus decoder tests
│   └── test_route_selection.c    # Route selection logic tests
├── integration/           # Integration tests
│   ├── test_build_artifacts.py   # Build verification tests
│   └── fixtures/                 # Test data fixtures
│       └── protocol_samples.py   # Protocol frame samples
└── stubs/                 # Mock/stub implementations (future)
```

## Running Tests

### Prerequisites

- ESP-IDF v5.5 or later
- Python 3.9+ with pytest installed

### Unit Tests

Unit tests are designed to test individual components in isolation. They use the Unity test framework included with ESP-IDF.

**Note**: Current unit tests are source-code level tests. Full ESP-IDF component testing requires additional configuration.

To verify test compilation:
```bash
cd tests/unit
gcc -c test_*.c -I../../main -I. -DUNITY_INCLUDE_CONFIG_H -Wno-error
```

### Integration Tests

Integration tests verify build configuration and protocol setup using pytest.

```bash
# Install dependencies
pip install pytest pytest-html

# Run integration tests
cd tests/integration
pytest test_build_artifacts.py -v
```

### Running All Tests in CI

The GitLab CI pipeline automatically runs all tests on every commit:

```bash
# Build stage
idf.py set-target esp32c6
idf.py build

# Test stage (runs automatically after build)
cd tests/integration
pytest test_build_artifacts.py -v
```

## Test Coverage

### Unit Tests

| Component | Test File | Coverage |
|-----------|-----------|----------|
| CAN Decoder | `test_can_decoder.c` | Frame parsing, cache, freshness |
| Modbus Decoder | `test_modbus_decoder.c` | Framing, CRC, register cache |
| Route Selection | `test_route_selection.c` | Protocol mapping, config validation |

### Integration Tests

| Test | Purpose |
|------|---------|
| Build Artifacts | Verify firmware builds with correct configuration |
| Protocol Constants | Verify all protocols are defined |
| File Structure | Verify all required source files exist |

## Protocol Fixtures

The `fixtures/protocol_samples.py` module provides pre-captured protocol frames for testing:

- **Growatt CAN frames**: SOC, voltage, current data
- **Pylon CAN frames**: Battery status frames
- **JKBMS Modbus responses**: Register read responses
- **Growatt Modbus responses**: BMS status registers

These fixtures can be used to create realistic test scenarios without requiring actual hardware.

## Adding New Tests

### Adding a Unit Test

1. Create a new test file in `tests/unit/` following the naming convention `test_*.c`
2. Include the Unity framework: `#include "unity.h"`
3. Implement setup/teardown functions
4. Write test functions following the pattern `test_<component>_<scenario>()`
5. Add test to `app_main()` using `RUN_TEST()`

Example:
```c
#include "unity.h"
#include "your_component.h"

void setUp(void) {
    /* Setup before each test */
}

void tearDown(void) {
    /* Cleanup after each test */
}

void test_component_basic_functionality(void) {
    TEST_ASSERT_EQUAL(expected, actual);
}

void app_main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_component_basic_functionality);
    UNITY_END();
}
```

### Adding an Integration Test

1. Create a new test file in `tests/integration/` following the naming convention `test_*.py`
2. Use pytest fixtures and assertions
3. Add to CI pipeline if needed

Example:
```python
import pytest

def test_feature_works():
    """Test that feature works as expected."""
    assert True, "Feature should work"
```

## Test Philosophy

### What to Test

- **Protocol decoders**: Frame parsing, cache management, data extraction
- **Route selection**: Configuration validation, protocol mapping
- **Data flow**: Telemetry aggregation, source attribution
- **Edge cases**: Invalid input, stale data, buffer overflows

### What NOT to Test

- **Hardware-specific behavior**: Real CAN bus timing, UART characteristics
- **ESP-IDF internals**: FreeRTOS, driver implementations
- **Third-party libraries**: Already tested by their maintainers

### Test Quality Guidelines

1. **Isolated**: Each test should test one thing
2. **Repeatable**: Same input → same output
3. **Fast**: Unit tests should run in milliseconds
4. **Clear**: Test name and assertions should explain what's being tested
5. **Independent**: Tests should not depend on execution order

## Known Limitations

### Current Limitations

1. **Unit tests require mocking**: Many components depend on ESP-IDF drivers that need to be mocked for true unit testing
2. **No hardware-in-loop**: Cannot test real CAN/RS485 communication
3. **Limited timing tests**: Cannot test real-time behavior without hardware

### Future Improvements

1. **Component test framework**: Use ESP-IDF component testing for full unit test support
2. **Protocol simulators**: Python scripts that simulate BMS/inverter devices
3. **Hardware abstraction layer**: Make components more testable through abstraction
4. **Code coverage**: Add gcov support for coverage metrics
5. **Performance benchmarks**: Add timing and throughput tests

## Continuous Integration

Tests run automatically on every push to GitLab:

1. **Build stage**: Firmware is compiled for ESP32-C6
2. **Test stage**:
   - Unit tests verify component behavior
   - Integration tests verify build configuration
   - Static analysis checks for code quality issues
3. **Report stage**: Test results are collected and made available as artifacts

See `.gitlab-ci.yml` for full CI configuration.

## Troubleshooting

### Tests fail to compile

- Ensure ESP-IDF is properly installed and sourced
- Check that include paths are correct
- Verify ESP-IDF version (v5.5+ required)

### Tests fail during execution

- Check test logs for specific assertion failures
- Verify test fixtures are valid
- Ensure hardware dependencies are properly mocked

### CI pipeline fails

- Check GitLab CI logs for specific errors
- Verify `.gitlab-ci.yml` syntax
- Ensure Docker image `espressif/idf:release-v5.5` is accessible

## Contributing

When adding new features or fixing bugs:

1. **Write tests first** (TDD approach recommended)
2. **Update existing tests** if behavior changes
3. **Add integration tests** for new protocol support
4. **Document test coverage** in this README
5. **Ensure CI passes** before submitting PR

## References

- [Unity Test Framework](https://github.com/ThrowTheSwitch/Unity)
- [ESP-IDF Testing Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/unit-tests.html)
- [pytest Documentation](https://docs.pytest.org/)
