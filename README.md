# ESP32 CAN/RS485 Bridge Firmware

This repository contains an ESP-IDF firmware for an ESP32 board that bridges, translates, decodes, and exposes battery/inverter communication over CAN and RS485.

The project is used as a practical protocol bridge between BMS and inverter, with runtime configuration through a web UI.

See also: [CHANGELOG.md](CHANGELOG.md)

## Target Hardware

- MCU family: Espressif ESP32
- Current target in this repository: **ESP32-C6**
- Suggested IDF target command: `idf.py set-target esp32c6`

## Short Description

Main goals:

- acquire battery data from CAN or RS485 protocols
- normalize and forward data toward inverter-side protocols
- support special translator routes (not only same-protocol pass-through)
- provide runtime settings via web API/UI
- expose telemetry + decoded logs for diagnostics

## Current Capabilities (High Level)

Implemented and actively used:

- `RS485_GROWATT` BMS polling/decoding (`Modbus`) + queue publish
- `JKBMS_MODBUS` BMS polling/decoding (`Modbus`) + rich snapshot
- `GROWATT` inverter CAN sender (publishes frame `0x322`)
- `CAN_GROWATT | CAN_PYLON | CAN_GOODWE | CAN_SOFAR | CAN_SMA | CAN_VICTRON -> RS485_GROWATT` translator (`main/rs485_can_bridge.c`)
- `RS485_JKBMS -> RS485_GROWATT` translator
- `RS485_PYLON <-> RS485_PYLON` bridge/responder
- `CAN_PYLON -> RS485_PYLON` synthetic responder/bridge
- CAN snapshot decoders for Growatt-like, Pylon, and Deye frame sets
- web UI + API for runtime config and telemetry

Partially implemented / scaffold:

- generic `PROTOCOL_ID_PYLON` orchestrator tasks (`pylon_bms_task.c`, `pylon_inverter_task.c`) are scaffold placeholders
- `CAN_SOFAR`, `CAN_SMA`, `CAN_VICTRON` have protocol IDs and register-map headers, but no dedicated full task pipeline yet
- `CAN_DEYE` has active CAN decode path, but is not currently in the dedicated `CAN -> RS485_GROWATT` special-route selector

Current bridge-mode route matrix:

| Route | Condition | Status |
| --- | --- | --- |
| CAN -> RS485_GROWATT translator | `bms_line=CAN`, `inv_line=RS485`, `inv_protocol=RS485_GROWATT`, `bms_protocol in {CAN_GROWATT,CAN_PYLON,CAN_GOODWE,CAN_SOFAR,CAN_SMA,CAN_VICTRON}` | Active |
| RS485_JKBMS -> RS485_GROWATT translator | `bms_line=RS485`, `inv_line=RS485`, `bms_protocol=JKBMS_MODBUS`, `inv_protocol=RS485_GROWATT` | Active |
| Pylon RS485 bridge | `RS485_PYLON<->RS485_PYLON` or `CAN_PYLON->RS485_PYLON` | Active |
| Generic orchestrator route | any other valid combination | Active, depends on protocol task maturity |

## Runtime Modes

Modes are selected from runtime settings (`mode`):

- `MODE_BRIDGE = 3`
- uses orchestrator + protocol tasks/translators
- this is the main production mode
- `MODE_FORWARD = 2`
- direct forward tasks (`CAN1->CAN2`, `RS485_1->RS485_2`) with optional decode logging
- `MODE_SNIFFER = 1`
- passive sniff/decode tasks on both CAN + both RS485 ports

Important:

- hot switching between modes is not supported at runtime in current build
- changing mode usually requires restart/reboot path

## Route Selection in Bridge Mode

Bridge mode route selection is done in `orchestratorStartFromRuntime(...)`:

1. `CAN -> RS485_GROWATT` translator route
2. `RS485_JKBMS -> RS485_GROWATT` translator route
3. `Pylon RS485 bridge` route (`RS485_PYLON<->RS485_PYLON` or `CAN_PYLON->RS485_PYLON`)
4. fallback generic orchestrator route (`protocol_id_t` based)

## Integration Notes

### `JKBMS_MODBUS -> RS485_PYLON`

For generic BMS sources such as `JKBMS_MODBUS`, the synthetic Pylon `0x63` status byte must stay conservative.

What we found during field debugging:

- the inverter accepted the bridge immediately when the web `Fake Inverter Data` override was active
- the same inverter stayed in fault with live JK data even when voltage, `SOC`, and current looked valid
- the decisive difference was the synthetic Pylon `0x63` status byte

Required rule:

- when the source is generic/non-native Pylon and there are no explicit native Pylon charge/discharge bits available, do not project the JK `balance` flag into Pylon `0x63`
- default the synthetic status to `0xC0` (`charge + discharge enabled`) instead of `0xE0`

Why this matters:

- projecting generic `balance=1` produced `0xE0`
- the affected inverter accepted `0xC0` but faulted on `0xE0`

Implementation reference:

- `main/Protocols/pylon/pylon_rs485_bridge.c`
- `buildCanDerivedInfo63(...)`

Additional robustness kept in place:

- `JKBMS` `cellAvgMv` is rejected as a pack-voltage source when it is inconsistent with decoded cell min/max values in the same snapshot
- fallback pack voltage is then derived from decoded cell extremes, avoiding false jumps caused by noisy or invalid average-cell registers

## Protocol IDs and Line IDs

Defined in `main/config.h`:

- `LINE_CAN = 1`
- `LINE_RS485 = 2`
- `PROTOCOL_CAN_GROWATT = 1`
- `PROTOCOL_RS485_GROWATT = 2`
- `PROTOCOL_RS485_PYLON = 3`
- `PROTOCOL_CAN_PYLON = 4`
- `PROTOCOL_CAN_DEYE = 5`
- `PROTOCOL_RS485_JKBMS = 6`
- `PROTOCOL_CAN_GOODWE = 7`
- `PROTOCOL_CAN_SOFAR = 8`
- `PROTOCOL_CAN_SMA = 9`
- `PROTOCOL_CAN_VICTRON = 10`

## Folder Structure (AI-Oriented)

Top-level:

- `main/` active firmware component
- `build/` ESP-IDF build artifacts (ignored)
- `vendor/` local/vendor files (ignored)
- `Protocol_Tasks/` legacy/auxiliary material

Inside `main/`:

- `main.c`
- app entrypoint (`app_main`), initializes drivers, mode manager, web task
- `config.h`, `config.c`
- compile-time switches, pin mapping, protocol/line IDs, exclusion lists, LED task
- `runtime_settings.h/.c`
- NVS-backed runtime settings (`bridge_cfg` namespace)
- `orchestrator/`
- route decision + queue orchestration across BMS and inverter tasks
- `modes/`
- mode manager + CAN forward/sniffer helper
- `decoders/`
- `CAN_Decoder` cache/snapshot/decode
- `modbusDecoder` stream decoder and register cache
- `protocols/`
- active protocol implementations and register maps
- `Drivers/`
- CAN + RS485 low-level init and TX helpers
- `Web_interface/`
- web server, web UI, JSON API, bridge compatibility layer
- `rs485_can_bridge.c/.h`
- specialized translator/responder for Growatt-oriented RS485 inverter side
- `old/`
- archived legacy code (not part of active CMake sources)

## Active vs Legacy Code Areas

Active source tree is defined by `main/CMakeLists.txt` and currently uses:

- `main/protocols/...` (lowercase)
- `main/modes/...`
- `main/decoders/...`
- `main/orchestrator/...`

Legacy/cleanup leftovers still present in repo:

- `main/Protocols/`, `main/BMS_Protocols/`, `main/Inverter_Protocols/`, `main/old/`

These are useful for reference/history, but are not the primary active implementation path.

## Protocol Implementation Status

`main/protocols/growatt/`

- `growatt_bms_task.c`: adapter/wrapper over `rs485_growatt` BMS task
- `growatt_inverter_task.c`: active CAN transmit task (`0x322`)
- `growatt_registers_map.h/.c`: register + frame mapping constants

`main/protocols/rs485_growatt/`

- active Modbus poller + decoder integration for Growatt BMS over RS485

`main/protocols/jkbms_modbus/`

- active JK BMS Modbus poller + decoder + rich snapshot extraction

`main/protocols/pylon/`

- `pylon_rs485_bridge.c`: active bridge/responder for Pylon RS485 routes
- `pylon_can_protocol.c`: active CAN snapshot decode for Pylon frames
- `pylon_bms_task.c`, `pylon_inverter_task.c`: scaffold tasks (placeholder logs)
- `pylon_registers_map.h`: canonical register map header

`main/protocols/deye/`

- `deye_can_protocol.c`: active CAN snapshot decode for Deye frame set

`main/protocols/goodwe|sofar|sma|victron/`

- currently map/header-focused; not full dedicated task pipelines
- used as protocol constants and integration hints for translator/routing logic

## Adding a New Protocol (AI-Safe Rules)

If Codex Web (or any contributor) adds a new protocol, use this mandatory layout:

- folder: `main/protocols/<protocol_name>/`
- required file: `main/protocols/<protocol_name>/<protocol_name>_registers_map.h`
- optional companion: `main/protocols/<protocol_name>/<protocol_name>_registers_map.c`
- protocol task files, if needed:
- `<protocol_name>_bms_task.c/.h`
- `<protocol_name>_inverter_task.c/.h`
- `<protocol_name>_can_protocol.c/.h`
- `<protocol_name>_modbus_poller.c/.h`

Minimum integration checklist (so files are actually used):

1. add new source files in `main/CMakeLists.txt`
2. include only canonical map header `*_registers_map.h` from runtime code
3. add/extend protocol ID in `main/config.h` (if new ID is needed)
4. update validation in `main/runtime_settings.c`
5. update protocol selection UI lists in `main/Web_interface/web_interface.c`
6. update routing/protocol mapping in `main/orchestrator/orchestrator.c`
7. update decoders/translators if the new protocol needs telemetry, bridging, or synthetic responses

## Important Core Files and Responsibilities

- `main/main.c`
- startup sequence and mode start
- `main/modes/mode_manager.c`
- behavior for `bridge/forward/sniffer`
- `main/orchestrator/orchestrator.c`
- bridge route decision and task orchestration
- `main/rs485_can_bridge.c`
- CAN->RS485 Growatt responder; JKBMS->RS485 Growatt responder; synthetic snapshot export
- `main/protocols/pylon/pylon_rs485_bridge.c`
- Pylon-specific RS485 responder/forwarder and CAN->RS485 synthetic support
- `main/decoders/CAN_Decoder.c`
- CAN cache, protocol-aware snapshots, universal model updates
- `main/decoders/modbusDecoder.c`
- Modbus stream framing + register cache + decode snapshots
- `main/Web_interface/web_interface.c`
- web UI and API endpoints
- `main/Web_interface/bridge_compat.c`
- compatibility facade between telemetry producers and web API

## Data Model

Shared data structures:

- `bms_decoded_packet_t` in `main/orchestrator/protocol_types.h`
- minimal normalized packet between BMS-side and inverter-side tasks
- `universal_battery_model_t` in `main/protocols/common/universal_battery_model.h`
- broader cross-protocol model used especially by synthetic bridges
- `bridgeTelemetrySnapshot_t` in `main/Web_interface/web_bridge_api.h`
- web-facing telemetry snapshot

## Web API

Implemented endpoints:

- `GET /`
- in-device HTML UI (Telemetry / Settings / Logs tabs)
- `GET /api/telemetry`
- JSON telemetry snapshot
- `GET /api/settings`
- JSON runtime settings
- `POST /api/settings`
- saves runtime settings to NVS and applies them
- `GET /api/logs`
- text decoded logs snapshot

Runtime settings are persisted in NVS namespace `bridge_cfg`.

## Build and Run

Typical ESP-IDF flow:

```bash
idf.py set-target esp32c6
idf.py build
idf.py -p <PORT> flash monitor
```

Notes:

- project name in CMake is `project-name`
- this repo currently enables `idf_build_set_property(MINIMAL_BUILD ON)`

## Configuration Notes

Most compile-time toggles are in `main/config.h`, including:

- pin mapping for CAN and RS485
- decoder verbosity flags
- bridge/forward feature flags
- RS485 half-duplex behavior flags
- default Wi-Fi/web settings

Runtime settings from web API/UI override operational route/mode choices.

## Testing

The project includes a comprehensive test suite to prevent regressions and ensure protocol compatibility.

### Test Structure

```
tests/
├── unit/                  # Unit tests for components (Unity framework)
├── integration/           # Integration tests (pytest)
└── fixtures/              # Protocol sample data
```

### Running Tests

```bash
# Build firmware first
idf.py set-target esp32c6
idf.py build

# Run integration tests
cd tests/integration
pip install pytest pytest-html
pytest test_build_artifacts.py -v
```

### Test Coverage

- **CAN Decoder**: Frame parsing, cache management, freshness checks
- **Modbus Decoder**: Framing, CRC validation, register caching
- **Route Selection**: Protocol mapping, configuration validation
- **Build Verification**: Feature flags, file structure, protocol definitions

See `tests/README.md` for detailed testing documentation.

### Continuous Integration

GitLab CI automatically runs tests on every commit:
- Build verification for ESP32-C6
- Integration tests with pytest
- Static analysis with cppcheck

## AI Onboarding Cheat Sheet (for Codex Web)

If you want an AI coding agent to be productive quickly, give it this context:

1. active code is what appears in `main/CMakeLists.txt`
2. primary runtime routing starts in `main/main.c`, `main/modes/mode_manager.c`, and `main/orchestrator/orchestrator.c`
3. do not start from `main/old` unless explicitly asked
4. protocol code of interest is under `main/protocols/`
5. decoders are in `main/decoders/`
6. web config/telemetry API is in `main/Web_interface/`
7. keep canonical protocol map naming as `*_registers_map.h`

Suggested first-read file order for AI:

1. `main/config.h`
2. `main/runtime_settings.h` + `main/runtime_settings.c`
3. `main/main.c`
4. `main/modes/mode_manager.c`
5. `main/orchestrator/orchestrator.c`
6. `main/rs485_can_bridge.c`
7. `main/protocols/pylon/pylon_rs485_bridge.c`
8. `main/decoders/CAN_Decoder.c`
9. `main/decoders/modbusDecoder.c`
10. `main/Web_interface/web_interface.c`

## Known Cleanup Direction

Repository is in active cleanup/refactor:

- protocol code consolidation under `main/protocols/` is ongoing
- mode files were moved under `main/modes/`
- decoders were moved under `main/decoders/`
- legacy uppercase protocol folders remain for reference during transition

If you continue cleanup, prefer small mechanical moves + include/CMake fixes, then validate route behavior.
