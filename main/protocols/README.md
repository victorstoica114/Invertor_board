# Protocols Layout

This folder contains the active protocol implementation used by the firmware.

## Naming Rules

- Each protocol folder must provide a canonical `*_registers_map.h`.
- Runtime/task code should include `*_registers_map.h` (not legacy aliases).
- Common, protocol-agnostic models live in `common/`.

## Current Structure

- `common/`: shared types and models (for example universal battery model).
- `<protocol>/`: protocol-specific maps, codecs/parsers, and tasks.

Examples:

- `growatt/growatt_registers_map.h`
- `jkbms_modbus/jkbms_modbus_registers_map.h`
- `pylon/pylon_registers_map.h`

## Cleanup Policy

- Remove obsolete alias headers (`*_can_map.h`, `*_register_map.h`) when no code references remain.
- Keep route-level components outside protocol folders when possible.
