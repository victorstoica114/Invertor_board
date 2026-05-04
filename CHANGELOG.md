# Changelog

This file tracks notable project-level changes and field-relevant known issues for the ESP32 CAN/RS485 bridge firmware.

The goal is practical maintenance:

- record protocol behavior changes that affect real hardware
- document why a fix was needed, not just that it happened
- keep a short list of known issues so future debugging starts faster

## Current Main

### Added

- Web UI support for `Fake Inverter Data` runtime override.
- Shared `battery_model` layer used by inverter-facing synthetic/projection paths.
- Project documentation for the `JKBMS_MODBUS -> RS485_PYLON` integration behavior.
- `PACE_RS485_MODBUS_V1.3` BMS poller/decoder for PACE RS485 Modbus sources.
- `RS485_PACE -> RS485_PYLON` bridge-mode route for Pylon-compatible inverter responders.
- Web/API telemetry for PACE all-cell voltage lists, individual temperature registers, protections, alarms/faults, warnings, and raw status flags.
- README telemetry quality standard for future protocol integrations, using `PACE_RS485_MODBUS_V1.3 -> RS485_PYLON` as the reference behavior.
- Experimental `JKBMS_RS485_NATIVE` BMS poller/decoder for JK native binary RS485 frames, including all-cell voltage, temperature, status, and alarm-bit telemetry.
- `RS485_JKBMS_NATIVE -> RS485_PYLON` and `RS485_JKBMS_NATIVE -> RS485_GROWATT` bridge-mode route selection through the existing synthetic responder paths.
- GitLab CI pipeline for automatic ESP32-C6 builds and host-side test execution on push/merge request.
- Separate CI suites for sanity, unit, integration, and firmware-build validation, with JUnit artifacts and coverage reports.
- Host-side regression tests for `JKBMS_MODBUS` source freshness and Modbus decoder cache timestamps.
- Public `main/secrets.example.h` template for Wi-Fi credentials, with local `main/secrets.h` ignored by Git.
- Custom `8MB` single-app partition table for ESP32-C6-WROOM-1-N8 modules.

### Changed

- Generic `RS485_PYLON` synthetic `0x61` generation was made more conservative for non-native Pylon sources.
- Generic `RS485_PYLON` synthetic `0x63` generation now defaults to a permissive `0xC0` status when explicit native Pylon charge/discharge bits are not available.
- Debug/investigation logs added during root-cause analysis were removed after the fix was validated.
- `JKBMS_MODBUS` bridge publishing now uses the newest real Modbus response timestamp instead of refreshing stale data with the current publish time.
- Web telemetry stale timeout was reduced to `10s`, so disconnected or stale BMS data disappears from the UI faster.
- CI now targets ESP-IDF `v6.0.1` and the ESP32-C6 target explicitly.
- GitLab CI jobs are routed to the local project runner through the `ubuntu` runner tag.
- Python CI dependencies are installed into an isolated local virtual environment when possible, with cache reuse for faster repeat runs.
- Firmware CI sources ESP-IDF from the local `gitlab-runner` installation and installs missing `cmake`/`ninja` tools into the ESP-IDF Python environment when needed.
- README build, test, coverage, and GitLab runner instructions were refreshed for the ESP-IDF 6.0.1 workflow.
- Local VS Code ESP-IDF settings were removed from version control and ignored because they contain machine-specific paths.
- Flash size is configured for `8MB`, and the app partition was expanded from `1MB` to `7MB`.
- `bms_decoded_packet_t` now carries richer decoded telemetry for protocols that expose per-cell voltages, per-sensor temperatures, warning/protection/fault masks, status flags, and balance flags.
- Pylon synthetic status generation can now use explicit PACE MOSFET charge/discharge flags while keeping the conservative generic fallback for non-native sources.

### Fixed

- Fixed inverter fault on `JKBMS_MODBUS -> RS485_PYLON`.

Root cause:

- live JK data projected the generic `balance` flag into synthetic Pylon `0x63`
- this produced `0xE0`
- the affected inverter accepted `0xC0` but faulted on `0xE0`

Final rule kept in code:

- for generic/non-native Pylon sources without explicit native charge/discharge bits, do not project the generic `balance` flag into Pylon `0x63`
- use `0xC0` instead

- Added protection against noisy/invalid `JKBMS` average-cell voltage registers.

Behavior kept in code:

- when `cellAvgMv` is inconsistent with decoded cell min/max values from the same snapshot, it is not trusted as the pack-voltage source
- pack voltage falls back to a cell-extremes-derived estimate

- Fixed ESP-IDF 6 component requirements by declaring the required `cJSON`, GPIO, UART, and TWAI driver components explicitly.
- Removed the unused managed `led_strip` dependency from the firmware build.
- Suppressed the legacy TWAI deprecation warning through sdkconfig until the TWAI API migration is done.
- Fixed GitLab CI failures caused by shared-runner selection, Docker image `IDF_PATH` conflicts, missing `pip`, Ubuntu PEP 668 restrictions, and missing local `cmake`/`ninja`.
- Fixed stale `JKBMS_MODBUS` decoder data being republished after the BMS cable was disconnected.
- Fixed near-full application partition warnings on ESP32-C6-WROOM-1-N8 by using the available `8MB` flash.
- Removed real Wi-Fi credentials from tracked source files.
- Fixed early PACE web telemetry showing identical temperature values by mapping PACE temperature registers to their real source labels (`MOS`, `Battery T1`, `Battery T2`, `Battery T4`, `Battery T5`) instead of copying a generic average temperature.

### Operational Notes

- `Fake Inverter Data` remains useful as a field diagnostic tool when validating inverter-side protocol behavior independently of live BMS decoding.
- For future `JKBMS -> Pylon` work, compare synthetic `0x63` semantics first before chasing pack-voltage formatting.
- The ESP32-C6-WROOM-1-N8 build now leaves roughly `86%` of the `7MB` app partition free.
- Keep real local Wi-Fi credentials in `main/secrets.h`; commit only `main/secrets.example.h`.
- Future BMS protocol work should match the PACE integration standard: decode real source fields, expose raw diagnostics, add host tests, build firmware, and verify against live `/api/telemetry` plus the vendor/BMS app.

## Known Issues

### JKBMS average-cell register can glitch

Observed in field logs:

- `cellAvgMv` can occasionally jump to unrealistic values while decoded cell min/max remain coherent
- examples seen during debugging included values such as `2818mV` and `3584mV`

Current mitigation:

- inconsistent `cellAvgMv` is rejected for pack-voltage derivation

Remaining note:

- root cause inside the JK register behavior/decoder path is not fully characterized yet

### Runtime mode hot-switching is not a supported workflow

- changing major bridge/sniffer/forward mode combinations should still be treated as a restart/reboot operation

### Some protocol implementations are still scaffold/partial

- `pylon_bms_task.c` and `pylon_inverter_task.c` remain placeholders
- several CAN protocol folders are present mainly for maps/constants and are not yet full end-to-end pipelines

## Maintenance Rule

When a hardware bug is solved in a way that depends on protocol semantics, add a short note here and, when helpful, a matching note in `README.md`.
