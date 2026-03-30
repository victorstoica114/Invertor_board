# Protocols Layout

This folder contains the active protocol implementation used by the firmware.

## Naming Rules

- Each protocol folder must provide a canonical `*_registers_map.h`.
- Runtime/task code should include `*_registers_map.h` (not legacy aliases).
- Common, protocol-agnostic models live in `common/`.

## New Protocol Template

When adding a new protocol, create:

- `main/protocols/<protocol_name>/`
- `main/protocols/<protocol_name>/<protocol_name>_registers_map.h` (mandatory)
- `main/protocols/<protocol_name>/<protocol_name>_registers_map.c` (optional)
- protocol runtime files as needed:
- `<protocol_name>_bms_task.c/.h`
- `<protocol_name>_inverter_task.c/.h`
- `<protocol_name>_can_protocol.c/.h`
- `<protocol_name>_modbus_poller.c/.h`

Integration checklist:

1. Add `.c` files into `main/CMakeLists.txt`.
2. Wire protocol ID/configuration in `main/config.h` and `main/runtime_settings.c`.
3. Expose protocol options in `main/Web_interface/web_interface.c`.
4. Update route selection/mapping in `main/orchestrator/orchestrator.c`.
5. Keep includes on canonical `*_registers_map.h` only.

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
