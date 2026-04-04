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

### Changed

- Generic `RS485_PYLON` synthetic `0x61` generation was made more conservative for non-native Pylon sources.
- Generic `RS485_PYLON` synthetic `0x63` generation now defaults to a permissive `0xC0` status when explicit native Pylon charge/discharge bits are not available.
- Debug/investigation logs added during root-cause analysis were removed after the fix was validated.

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

### Operational Notes

- `Fake Inverter Data` remains useful as a field diagnostic tool when validating inverter-side protocol behavior independently of live BMS decoding.
- For future `JKBMS -> Pylon` work, compare synthetic `0x63` semantics first before chasing pack-voltage formatting.

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
