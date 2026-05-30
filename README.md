# ESP32 CAN/RS485 Bridge Firmware

This repository contains an ESP-IDF firmware for an ESP32 board that bridges, translates, decodes, and exposes battery/inverter communication over CAN and RS485.

The project is used as a practical protocol bridge between BMS and inverter, with runtime configuration through a web UI.

See also: [CHANGELOG.md](CHANGELOG.md)

CAN protocol implementation backlog: [docs/can_protocols.md](docs/can_protocols.md)

## Target Hardware

- MCU family: Espressif ESP32
- Current target in this repository: **ESP32-C6**
- Field-tested module: **ESP32-C6-WROOM-1-N8** (`8MB` flash)
- ESP-IDF version used by local builds and CI: **v6.0.1**
- Suggested IDF target command: `idf.py set-target esp32c6`
- Current partition layout: single `7MB` factory app partition, no OTA

## Short Description

Main goals:

- acquire battery data from CAN or RS485 protocols
- normalize and forward data toward inverter-side protocols
- support special translator routes (not only same-protocol pass-through)
- provide runtime settings via web API/UI
- expose telemetry + decoded logs for diagnostics

## Current Capabilities (High Level)

Implemented / available:

- `RS485_GROWATT` BMS polling/decoding (`Modbus`) + queue publish, including JK UART profile `006` cell voltages and Growatt warning/error bits
- `JKBMS_MODBUS` BMS polling/decoding (`Modbus`) + rich snapshot, with `9600` and `115200` RS485 variants
- `JKBMS_RS485_NATIVE` experimental BMS polling/decoding (`4E 57` native RS485 read-all frames) + rich snapshot
- `PACE_RS485_MODBUS_V1.3` BMS polling/decoding (`Modbus`) + rich snapshot
- `VOLTRONIC_MODBUS` BMS polling/decoding for JK UART profile `007 - Voltronic_Inverter_and_BMS_485` + rich snapshot
- `CHINA_TOWER_MODBUS` BMS polling/decoding for JK UART profile `008 - China tower shared battery cabinet V2.0` + pack/cell/temperature snapshot
- `WOW_MODBUS` BMS polling/decoding for JK UART profile `009 - WOW_RS485_Modbus_V1.3` + PACE-compatible pack/cell/temperature snapshot
- `DALY_RS485` BMS polling/decoding for the Daly proprietary RS485 protocol + rich snapshot
- experimental `DALY_CAN` BMS polling/decoding for the Daly proprietary CAN protocol
- `GROWATT` inverter CAN sender (publishes frame `0x322`)
- `CAN_GROWATT | CAN_PYLON | CAN_GOODWE | CAN_SOFAR | CAN_SMA | CAN_VICTRON -> RS485_GROWATT` translator (`main/protocols/rs485_growatt/rs485_growatt_bridge.c`)
- `RS485_JKBMS -> RS485_GROWATT` translator (`JKBMS_MODBUS`, `JKBMS_MODBUS_115200`, and `JKBMS_RS485_NATIVE`)
- `RS485_JKBMS -> RS485_PYLON` translator/responder (`JKBMS_MODBUS`, `JKBMS_MODBUS_115200`, and `JKBMS_RS485_NATIVE`)
- `RS485_GROWATT -> RS485_PYLON` translator/responder, intended for JK UART profile `006 - Growatt_BMS_RS485_Protocol_1xSxxP_ESS_Rev2.01`
- `RS485_PACE -> RS485_PYLON` translator/responder
- `RS485_VOLTRONIC -> RS485_PYLON` translator/responder, intended for JK UART profile `007 - Voltronic_Inverter_and_BMS_485`
- `RS485_CHINA_TOWER -> RS485_PYLON` translator/responder, intended for JK UART profile `008 - China tower shared battery cabinet V2.0`
- `RS485_WOW -> RS485_PYLON` translator/responder, intended for JK UART profile `009 - WOW_RS485_Modbus_V1.3`
- `RS485_DALY -> RS485_PYLON` translator/responder
- experimental `DALY_CAN -> RS485_PYLON` translator/responder
- `RS485_PYLON <-> RS485_PYLON` bridge/responder, including the `RS485_PYLON_115200` variant
- `CAN_PYLON -> RS485_PYLON` synthetic responder/bridge, including the `RS485_PYLON_115200` variant
- `JKBMS_CAN_250K -> RS485_PYLON` synthetic responder/bridge for JK app profile `000 - JK BMS CAN Protocol (250K) V2.0`
- `CAN_DEYE -> RS485_PYLON` synthetic responder/bridge for JK app profile `001 - Deye Low-voltage hybrid inverter CAN`
- `CAN_PYLON -> CAN_PYLON` direct forward mode for split CAN ports, field-tested as `JKBMS Pylon CAN on CAN1 -> EASUN Pylon CAN on CAN2`
- CAN snapshot decoders for Growatt-like, Pylon, Deye, and JK BMS CAN frame sets
- web UI + API for runtime config and telemetry
- runtime `Fake Inverter Data` override for inverter-facing synthetic routes, Pylon RS485 passthrough, and Pylon CAN forward frames

Partially implemented / scaffold:

- generic `PROTOCOL_ID_PYLON` orchestrator tasks (`pylon_bms_task.c`, `pylon_inverter_task.c`) are scaffold placeholders
- `CAN_SOFAR`, `CAN_SMA`, `CAN_VICTRON` have protocol IDs and register-map headers, but no dedicated full task pipeline yet
- `CAN_DEYE` has active CAN decode path, but is not currently in the dedicated `CAN -> RS485_GROWATT` special-route selector
- `DALY_CAN` has a native poller and `RS485_PYLON` synthetic responder route, but the field BMS tested on 2026-05-30 did not ACK CAN traffic on either CAN port
- optional EASUN/Pylon 24V diagnostic CAN sender exists for isolated inverter tests, but is disabled by default (`EASUN_PYLON_24V_DIAG_SENDER_ENABLE=0`)

Current bridge-mode route matrix:

| Route | Condition | Status |
| --- | --- | --- |
| CAN -> RS485_GROWATT translator | `bms_line=CAN`, `inv_line=RS485`, `inv_protocol=RS485_GROWATT`, `bms_protocol in {CAN_GROWATT,CAN_PYLON,CAN_GOODWE,CAN_SOFAR,CAN_SMA,CAN_VICTRON}` | Active |
| RS485_JKBMS -> RS485_GROWATT translator | `bms_line=RS485`, `inv_line=RS485`, `bms_protocol in {JKBMS_MODBUS,JKBMS_MODBUS_115200,JKBMS_RS485_NATIVE}`, `inv_protocol=RS485_GROWATT` | Active for Modbus; native is experimental |
| RS485_JKBMS -> RS485_PYLON translator | `bms_line=RS485`, `inv_line=RS485`, `bms_protocol in {JKBMS_MODBUS,JKBMS_MODBUS_115200,JKBMS_RS485_NATIVE}`, `inv_protocol in {RS485_PYLON,RS485_PYLON_115200}` | Active for Modbus; native is experimental |
| RS485_GROWATT -> RS485_PYLON translator | `bms_line=RS485`, `inv_line=RS485`, `bms_protocol=RS485_GROWATT`, `inv_protocol=RS485_PYLON` | Active; covers JK UART profile `006` |
| RS485_PACE -> RS485_PYLON translator | `bms_line=RS485`, `inv_line=RS485`, `bms_protocol=PACE_RS485_MODBUS`, `inv_protocol=RS485_PYLON` | Active |
| RS485_VOLTRONIC -> RS485_PYLON translator | `bms_line=RS485`, `inv_line=RS485`, `bms_protocol=VOLTRONIC_MODBUS`, `inv_protocol=RS485_PYLON` | Active; covers JK UART profile `007` |
| RS485_CHINA_TOWER -> RS485_PYLON translator | `bms_line=RS485`, `inv_line=RS485`, `bms_protocol=CHINA_TOWER_MODBUS`, `inv_protocol=RS485_PYLON` | Active; covers JK UART profile `008` |
| RS485_WOW -> RS485_PYLON translator | `bms_line=RS485`, `inv_line=RS485`, `bms_protocol=WOW_MODBUS`, `inv_protocol=RS485_PYLON` | Active initial implementation; covers JK UART profile `009` with PACE-compatible map |
| RS485_DALY -> RS485_PYLON translator | `bms_line=RS485`, `inv_line=RS485`, `bms_protocol=DALY_RS485`, `inv_protocol=RS485_PYLON` | Active |
| DALY_CAN -> RS485_PYLON translator | `bms_line=CAN`, `inv_line=RS485`, `bms_protocol=DALY_CAN`, `inv_protocol=RS485_PYLON` | Experimental; protocol task implemented, live CAN link not validated |
| Pylon RS485 bridge | `RS485_PYLON<->RS485_PYLON`, `CAN_PYLON->RS485_PYLON`, `CAN_DEYE->RS485_PYLON`, or `JKBMS_CAN_250K->RS485_PYLON`, with `RS485_PYLON_115200` accepted on RS485 sides | Active |
| Generic orchestrator route | any other valid combination | Active, depends on protocol task maturity |

## Runtime Modes

Modes are selected from runtime settings (`mode`):

- `MODE_BRIDGE = 3`
- uses orchestrator + protocol tasks/translators
- this is the main production mode
- `MODE_FORWARD = 2`
- direct forward tasks (`CAN1->CAN2`, `RS485_1->RS485_2`) with optional decode logging
- field-tested CAN forward case: `JKBMS Pylon CAN -> EASUN Pylon CAN`, with JKBMS on `CAN1`, EASUN on `CAN2`, both at `500 kbit/s`
- when runtime `Fake Inverter Data` is enabled, Pylon CAN forward modifies outgoing inverter-facing frames before transmit (`0x351`, `0x355`, `0x356`, `0x35C`, and `0x373` when present)
- `MODE_SNIFFER = 1`
- passive sniff/decode tasks on both CAN + both RS485 ports

Important:

- hot switching between modes is not supported at runtime in current build
- changing mode usually requires restart/reboot path

## Route Selection in Bridge Mode

Bridge mode route selection is done in `orchestratorStartFromRuntime(...)`:

1. `CAN -> RS485_GROWATT` translator route
2. `RS485_JKBMS -> RS485_GROWATT` translator route
3. `RS485_JKBMS -> RS485_PYLON` translator route
4. `RS485_GROWATT -> RS485_PYLON` translator route
5. `RS485_PACE -> RS485_PYLON` translator route
6. `RS485_VOLTRONIC -> RS485_PYLON` translator route
7. `RS485_CHINA_TOWER -> RS485_PYLON` translator route
8. `RS485_WOW -> RS485_PYLON` translator route
9. `RS485_SEPLOS -> RS485_PYLON` translator route
10. `RS485_DALY -> RS485_PYLON` translator route
11. `DALY_CAN -> RS485_PYLON` translator route
12. `Pylon RS485 bridge` route (`RS485_PYLON<->RS485_PYLON` or `CAN_PYLON->RS485_PYLON`)
13. fallback generic orchestrator route (`protocol_id_t` based)

## Integration Notes

### Field Status: EASUN Pylon CAN

As of the 2026-05-30 field session, the EASUN inverter CAN path is validated as
a split-port forward route.

Bench setup that worked:

- JKBMS is configured for `PYLON CAN` and connected to `CAN1`.
- EASUN inverter is configured for `PYLON CAN` and connected to `CAN2`.
- Both CAN ports use the normal Pylon bitrate, `500 kbit/s`.
- The active firmware mode is forward mode: `CAN1 -> CAN2`.
- Current field config uses `BMS_protocol=PROTOCOL_CAN_PYLON`,
  `Inverter_protocol=PROTOCOL_CAN_PYLON`, `BMS_PORT=1`, `Inverter_PORT=2`.

Observed frames:

- JKBMS emits normal Pylon frames such as `0x351`, `0x355`, `0x356`, `0x359`,
  `0x35C`, `0x35E`, `0x370`, and `0x371`.
- `0x35E` identifies the source as `JK-BMS`.
- EASUN emits repeated `0x305 [00 00 00 00 00 00 00 00]` frames when its CAN
  hardware is healthy.

Important hardware finding:

- The initial "EASUN is silent/dead" symptom was caused by the inverter CAN
  transceiver hardware, not by the Pylon CAN firmware path.
- After the inverter CAN transceiver was repaired, EASUN traffic was visible and
  the direct forward route was accepted.

Current forwarding rule:

- The firmware forwards the JKBMS Pylon CAN data directly.
- No voltage division/scaling is active in the field config
  (`CAN_FORWARD_PYLON_16S_TO_8S_ENABLE=0`), even though EASUN is a 24V inverter.
- The optional `16S->8S` scaler remains behind the compile-time macro if a
  future inverter test proves it is needed.
- Do not clamp live forwarded voltage/current/SOC limits unless the inverter
  rejects the direct JKBMS values; these values can be adjusted on the JKBMS side.

Fake-data behavior:

- Web `Fake Inverter Data` now affects Pylon CAN forward mode, not only
  synthetic bridge/responders.
- In forward mode, fake data edits outgoing Pylon CAN frames before transmission
  to the inverter. The most important frame for EASUN behavior is `0x355`
  (`SOC`/`SOH`).
- Setting fake `SOC=0%` was field-confirmed to influence EASUN behavior.
- If fake `SOC=0%` makes EASUN stop/sleep, CAN TX failures or no-ACK counters on
  the inverter-facing bus can be expected during that test.
- Fake data is runtime-only. Reboot, reset, or flashing clears it and it must be
  reapplied from the web UI or `/api/fake_bms`.

### Known Field Issue: Seplos RS485

As of the 2026-05-24 field session, the Seplos RS485 issue was traced to a
hardware-layer requirement: the tested Seplos BMS does not provide RS485 bias
resistors on its interface. The bridge hardware used during testing also does
not provide enough fail-safe bias on that bus, so the idle RS485 line can float.
In that state the firmware sees noise-like bytes or short invalid chunks even
though the same Seplos BMS communicates normally with an inverter that provides
proper bus bias.

This is a hardware compatibility issue, not primarily a Pylon/Seplos protocol
decoder problem.

Observed on the bench:

- Seplos communicates with the inverter when connected directly.
- With the bridge configured as `RS485_PYLON <-> RS485_PYLON`, the inverter
  sends valid Pylon requests on `RS485_2` (`0x61` and `0x63`), and the firmware
  forwards those requests to `RS485_1`.
- `RS485_1` raw captures after forwarding do not contain a valid Pylon ASCII
  frame (`~...`), a valid Seplos ASCII frame, or a coherent Modbus RTU response.
  Captured bytes were mostly `00`, `FF`, `05`, `EE`, and other non-frame data.
- The same result was observed with `RS485_1` in ESP-IDF half-duplex mode,
  manual direction control, and inverted manual direction control.
- Active Seplos proprietary polling at `9600` also did not produce valid
  responses; only single-byte or short non-frame raw chunks were captured.
- Adding external RS485 bias resistors to the Seplos-facing bus made the link
  behave normally enough for valid Pylon-style Seplos responses to be captured
  and decoded.

Hardware requirement / diagnostic recommendation:

- The Seplos-facing RS485 bus must have a fail-safe bias network. Do not rely on
  the Seplos BMS to provide bias.
- Add bias resistors on the bridge board revision, or fit external bias on the
  Seplos RS485 pair during bench testing.
- Termination alone is not a substitute for bias; the idle A/B state must be
  driven to a known level.
- Keep `RS485_2` as the known-good inverter side unless the hardware wiring is
  intentionally changed.
- If Seplos appears to work when connected directly to the inverter but not via
  the bridge, check RS485 bias before changing protocol code, baud rate, UART
  inversion, or direction-control settings.

### Experimental Field Status: Daly CAN

As of the 2026-05-30 field session, `DALY_CAN` is implemented as a native CAN
poller, but it is not live-validated with the tested Daly BMS.

Protocol assumptions currently used by the firmware:

- CAN bitrate is `250 kbit/s`.
- The request identifier is extended CAN ID `0x18{DataID}{BMS_ID}40`.
- The tested hardware uses `BMS_ID = 1`, so the SOC/current/voltage request is
  `0x18900140`.
- The expected response for that request is `0x18904001`.
- Requests use 8 reserved data bytes, matching
  `datasheets/daly-can-communications-protocol-v1-0-pr_69416dce77488f5846f10633ac80e389.pdf`.

Additional compatibility test already tried:

- A DLC-0 request was tested because the public `dbus-serialbattery`
  discussion #561 shows examples such as `cansend can0 18950140#`.

Observed on the bench:

- `CAN1` and `CAN2` were both tested at `250 kbit/s`.
- The request ID was logged as `0x18900140`.
- No valid Daly CAN frames were received.
- TWAI status showed a repeated no-ACK pattern: `txErr=128`, increasing
  `txFail`/bus error count, `rxErr=0`, and `rxMiss=0`.

Current interpretation:

- The firmware-side protocol framing matches the available Daly CAN datasheet
  and the public Linux CAN examples.
- The remaining failure looks below the decoder/protocol layer: CAN wiring,
  termination/common ground, BMS CAN enable/profile, direct BMS CAN port versus
  Daly interface board, or a Daly unit whose labelled CAN port is not active.
- Do not spend more protocol-debug time on Daly CAN until a USB-CAN or inverter
  capture proves that the BMS ACKs/responds on the tested CAN pair.

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

- `main/protocols/pylon/pylon_rs485_bridge.c`
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
- `PROTOCOL_RS485_PACE = 11`
- `PROTOCOL_RS485_JKBMS_NATIVE = 12`
- `PROTOCOL_RS485_VOLTRONIC = 13`
- `PROTOCOL_RS485_CHINA_TOWER = 14`
- `PROTOCOL_RS485_WOW = 15`
- `PROTOCOL_RS485_JKBMS_115200 = 16`
- `PROTOCOL_RS485_PYLON_115200 = 17`
- `PROTOCOL_CAN_JKBMS_250K = 18`
- `PROTOCOL_RS485_SEPLOS = 19`
- `PROTOCOL_RS485_SEPLOS_19200 = 20`
- `PROTOCOL_RS485_DALY = 21`
- `PROTOCOL_CAN_DALY = 22`

Baud-rate / bitrate detail:

- `PROTOCOL_RS485_JKBMS` and `PROTOCOL_RS485_PYLON` use `9600`
- `PROTOCOL_RS485_JKBMS_115200` and `PROTOCOL_RS485_PYLON_115200` use `115200`
- the `115200` variants reuse the same protocol decoders/responders; only the selected RS485 port baud changes
- `PROTOCOL_RS485_SEPLOS_19200` uses `19200`; `PROTOCOL_RS485_SEPLOS` and `PROTOCOL_RS485_DALY` use `9600`
- `PROTOCOL_CAN_JKBMS_250K` and `PROTOCOL_CAN_DALY` reinitialize the selected CAN port at `250 kbit/s`; other CAN profiles currently default to `500 kbit/s`

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
- `protocols/rs485_growatt/rs485_growatt_bridge.c/.h`
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
- decodes pack voltage, SOC, the single live pack temperature exposed by `0x0018`, per-cell voltages, cell min/max, Growatt `0x0014` error/protection bits, `0x0022` warning bits, and raw `0x0013` status flags when the BMS exposes them
- web telemetry intentionally shows only one temperature for this protocol; the Growatt RS485 status block does not expose separate MOS/T1/T2/T4/T5 live temperatures

`main/protocols/jkbms_modbus/`

- active JK BMS Modbus poller + decoder + rich snapshot extraction

`main/protocols/jkbms_rs485/`

- experimental JK BMS native RS485 poller for binary frames starting with `4E 57`
- decodes pack voltage/current/power, `SOC`, `SOH` fallback, cycles, capacity, per-cell voltages, cell min/max/avg/delta, tube/MOS temperature, battery temperature, box temperature, raw alarm bits, charge/discharge MOS status, and balance status
- supports `RS485_JKBMS -> RS485_PYLON` and `RS485_JKBMS -> RS485_GROWATT` bridge-mode translation through the shared battery model
- field note: on the tested JK BMS setup, the same RS485 port responds correctly to `JKBMS_MODBUS` at `9600`, but does not respond to the native `4E 57` read-all request, so `JKBMS_MODBUS` remains the recommended setting for that hardware

`main/protocols/pace_modbus/`

- active PACE BMS RS485 Modbus V1.3 poller + decoder
- supports `RS485_PACE -> RS485_PYLON` bridge-mode translation
- web/API telemetry includes pack values, all cell voltages, individual temperature registers, protections, alarms/faults, warnings, raw status flags, and Pylon status output

`main/protocols/voltronic_modbus/`

- active Voltronic RS485 Modbus poller + decoder for JK UART profile `007 - Voltronic_Inverter_and_BMS_485`
- polls the Seplos-compatible public Voltronic core registers as single `0x03` reads; the tested Seplos firmware rejects broad reads but answers single-register requests with a 16-bit byte-count response (`01 03 00 02 ... CRC`)
- keeps decoding support for standard Modbus responses, Voltronic function-first responses, and JK-compatible word-count responses in the shared register cache
- supports `RS485_VOLTRONIC -> RS485_PYLON` bridge-mode translation through the shared synthetic Pylon responder path
- web/API telemetry includes the confirmed live Seplos Voltronic pack values (`SOC`, voltage/current, capacity, charge/discharge limits/status) and raw protocol flags; the tested Seplos Voltronic profile did not expose per-cell voltages through this map

`main/protocols/china_tower_modbus/`

- active China Tower shared battery cabinet RS485 Modbus poller + decoder for JK UART profile `008`
- polls the observed live map around `0x0000` for pack voltage, cell count, `SOC`, and runtime fields, with per-cell millivolts starting at `0x0009`; raw candidate warning/protection/status fields are captured at `0x0019..0x001B`
- supports `RS485_CHINA_TOWER -> RS485_PYLON` bridge-mode translation through the shared synthetic Pylon responder path
- web/API telemetry includes pack voltage, `SOC`, all cell voltages, cell min/max/delta, and the three live temperature registers observed in the compact summary block; only `MOS`, `Temp 1`, and `Temp 2` are labeled for this profile because the exact battery-sensor assignment is not yet confirmed
- named alert bits for China Tower profile `008` are not confirmed yet; non-zero raw candidate warning/protection bits are surfaced as unknown bits until they can be correlated with the JK app

`main/protocols/wow_modbus/`

- active WOW RS485 Modbus poller + decoder for JK UART profile `009 - WOW_RS485_Modbus_V1.3`
- public WOW/SRNE register documentation is sparse, so the first implementation deliberately uses the PACE-compatible V1.3 register blocks as a separate protocol path instead of sharing cache interpretation with other pollers
- supports `RS485_WOW -> RS485_PYLON` bridge-mode translation through the shared synthetic Pylon responder path
- web/API telemetry includes pack values, all cell voltages, PACE-style temperature labels, warning/protection/fault/status fields, and raw diagnostics when the source exposes them
- field validation on a BMS configured to JK UART profile `009` confirmed stable pack, cell, temperature, and Pylon-responder behavior; alert naming remains conservative until more fault-state captures are available

`main/protocols/daly_rs485/`

- active Daly proprietary RS485 poller + decoder at `9600`
- decodes pack voltage/current/power, `SOC`, `SOH`, cycles, capacity, cell extremes, per-cell voltages where exposed, temperature sensors, MOS state, balance state, and failure/status bits
- supports `RS485_DALY -> RS485_PYLON` bridge-mode translation through the shared synthetic Pylon responder path
- field validation confirmed that telemetry remains available after the PC Daly application is stopped, and stale-source timeout is `10s`

`main/protocols/daly_can/`

- experimental Daly proprietary CAN poller + decoder at `250 kbit/s`
- uses extended request IDs in the Daly format `0x18{DataID}{BMS_ID}40`, with `BMS_ID=1` on the field hardware
- uses the same Daly snapshot/model path as `DALY_RS485`, so successful CAN telemetry can feed `DALY_CAN -> RS485_PYLON`
- not live-validated yet: the 2026-05-30 field BMS did not ACK either DLC-8 datasheet requests or DLC-0 Linux CAN example requests on CAN1/CAN2

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

### Telemetry Quality Standard for New Protocols

`PACE_RS485_MODBUS_V1.3 -> RS485_PYLON` is the reference for how new protocol integrations should look and behave in the web UI/API.

For every new BMS protocol, aim to expose a complete, inspectable telemetry snapshot:

- runtime/source/protocol validity, age, stale state, and inverter-facing status
- pack voltage, current, power, `SOC`, `SOH`, cycle count, and capacity fields when the source protocol provides them
- cell min/max, delta, indexes, average, and the full per-cell voltage list instead of only synthesized extremes
- real temperature registers mapped to their source labels; do not copy one generic/average temperature into every displayed sensor slot
- protections, alarms/faults, warnings, balance state, and raw status/protocol flags decoded into human-readable names where the protocol map is known
- unknown bits should still be visible as raw hex/unknown bit labels so field debugging can continue without firmware changes

Validation expectation:

1. add or extend host tests for the register/frame decoder, including cell arrays, temperature arrays, warning/protection/fault bits, and status flags
2. run the Python/host test suite
3. build the ESP-IDF firmware
4. flash the board when hardware is available
5. verify `/api/telemetry` and the web UI against the vendor/BMS app screenshots or live source data

Avoid temporary UI polish that hides missing decode work. If the source protocol exposes a value, prefer decoding and showing it explicitly; if it is not known yet, show `None`, `-`, or a raw diagnostic value instead of fabricating a plausible value.

## Important Core Files and Responsibilities

- `main/main.c`
- startup sequence and mode start
- `main/modes/mode_manager.c`
- behavior for `bridge/forward/sniffer`
- `main/orchestrator/orchestrator.c`
- bridge route decision and task orchestration
- `main/protocols/rs485_growatt/rs485_growatt_bridge.c`
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
- `GET /api/fake_bms`
- JSON runtime fake inverter data model
- `POST /api/fake_bms`
- enables/disables runtime fake inverter data and updates override values

Runtime settings are persisted in NVS namespace `bridge_cfg`.

## Build and Run

Use ESP-IDF **v6.0.1** and target `esp32c6`.

Before the first local build, create a private secrets file for your Wi-Fi
credentials:

```bash
cp main/secrets.example.h main/secrets.h
```

Typical shell flow:

```bash
. /path/to/esp-idf/export.sh
idf.py set-target esp32c6
idf.py build
idf.py -p <PORT> flash monitor
```

Notes:

- project name in CMake is `project-name`
- this repo currently enables `idf_build_set_property(MINIMAL_BUILD ON)`
- flash size is configured as `8MB`
- the generated flash command should include `--flash-size 8MB`
- the custom partition table is `partitions_8mb_singleapp.csv`

### Local Windows / Codex Access Notes

These notes are for future Codex sessions on the current Windows development
machine, so they do not have to rediscover the local ESP-IDF setup.

Known local setup:

- workspace: `C:\Users\Admin\Documents\test-CAN+RS485`
- ESP-IDF path used by the working build: `C:\esp\v6.0.1\esp-idf`
- ESP-IDF Python: `C:\Espressif\tools\python\v6.0.1\venv\Scripts\python.exe`
- Ninja: `C:\Espressif\tools\ninja\1.12.1\ninja.EXE`
- ESP32-C6 serial port used during field tests: `COM11`
- web UI used during field tests: `http://192.168.141.151/`
- opening the serial monitor can reset the board; runtime-only fake inverter data must be reapplied after reset/flash

Fast incremental build from a Codex/PowerShell shell:

```powershell
$env:IDF_PATH='C:\esp\v6.0.1\esp-idf'
$env:PATH='C:\Espressif\tools\ccache\4.12.1\ccache-4.12.1-windows-x86_64;C:\Espressif\tools\riscv32-esp-elf\esp-15.2.0_20251204\riscv32-esp-elf\bin;C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\cmake\4.0.3\bin;C:\Espressif\tools\python\v6.0.1\venv\Scripts;' + $env:PATH
& 'C:\Espressif\tools\ninja\1.12.1\ninja.EXE' -C build
```

Known harmless build warnings in the Codex sandbox:

- `fatal: detected dubious ownership in repository at 'C:/esp/v6.0.1/esp-idf'`
- `ESP_ROM_ELF_DIR environment variable is not defined`

The build can still complete and generate `build\project-name.bin`; verify the
final Ninja result and binary-size line before assuming failure.

Fast flash without rebuilding:

```powershell
& 'C:\Espressif\tools\python\v6.0.1\venv\Scripts\python.exe' 'C:\esp\v6.0.1\esp-idf\components\esptool_py\esptool\esptool.py' --chip esp32c6 -p COM11 -b 460800 --before default-reset --after hard-reset write_flash --flash-mode dio --flash-freq 80m --flash-size 8MB 0x0 build\bootloader\bootloader.bin 0x8000 build\partition_table\partition-table.bin 0x10000 build\project-name.bin
```

Serial monitor options:

```powershell
& 'C:\Espressif\tools\python\v6.0.1\venv\Scripts\python.exe' 'C:\esp\v6.0.1\esp-idf\tools\idf_monitor.py' -p COM11 -b 115200 --toolchain-prefix riscv32-esp-elf- --target esp32c6 'C:\Users\Admin\Documents\test-CAN+RS485\build\project-name.elf'
```

Quick fake-SOC API test from PowerShell:

```powershell
$payload = @{
  enabled = $true
  pack_voltage_v = 57.16
  pack_current_a = 0
  soc_pct = 0
  soh_pct = 100
  cycles = 0
  charge_voltage_limit_v = 57.6
  charge_current_limit_a = 100
  discharge_current_limit_a = 100
  cell_max_v = 3.58
  cell_min_v = 3.57
  cell_max_idx = 1
  cell_min_idx = 1
  temp_mos_c = 25
  temp_t1_c = 25
  temp_t2_c = 25
  temp_t4_c = 25
  temp_t5_c = 25
  charge_enabled = $true
  discharge_enabled = $true
  balance_enabled = $false
  protocol_state = 192
} | ConvertTo-Json
Invoke-RestMethod -Uri 'http://192.168.141.151/api/fake_bms' -Method Post -Body $payload -ContentType 'application/json'
```

For a short non-interactive log capture that closes the port automatically:

```powershell
@'
import serial
import sys
import time

s = serial.Serial('COM11', 115200, timeout=0.2)
end = time.time() + 8
while time.time() < end:
    data = s.read(4096)
    if data:
        sys.stdout.write(data.decode('utf-8', 'replace'))
        sys.stdout.flush()
s.close()
'@ | & 'C:\Espressif\tools\python\v6.0.1\venv\Scripts\python.exe' -
```

If `COM11` reports `Access is denied`, a previous monitor process is usually
still holding the port. Check for ESP-IDF Python monitor processes before
starting a new monitor. During field testing, stopping stale
`C:\Espressif\tools\python\v6.0.1\venv\Scripts\python.exe` monitor processes
released the port.

Useful web/API checks:

```powershell
(Invoke-WebRequest -Uri http://192.168.141.151/api/settings -UseBasicParsing -TimeoutSec 8).Content
(Invoke-WebRequest -Uri http://192.168.141.151/api/telemetry -UseBasicParsing -TimeoutSec 8).Content
(Invoke-WebRequest -Uri http://192.168.141.151/api/logs -UseBasicParsing -TimeoutSec 8).Content
```

Current known-good RS485 sanity route on the field bench:

- `bms_line=RS485`, `bms_protocol=RS485_PYLON`, `bms_port=1`
- `inverter_line=RS485`, `inverter_protocol=RS485_PYLON`, `inverter_port=2`
- this route produced valid Pylon `0x61`, `0x62`, and `0x63` decoded logs on
  `RS485_1`, confirming that the physical RS485 port and direction control work

## Configuration Notes

Most compile-time toggles are in `main/config.h`, including:

- pin mapping for CAN and RS485
- decoder verbosity flags
- bridge/forward feature flags
- RS485 half-duplex behavior flags
- default Wi-Fi/web settings

Local Wi-Fi credentials must live in `main/secrets.h`, which is ignored by Git.
Start from the public template:

```bash
cp main/secrets.example.h main/secrets.h
```

Then edit `main/secrets.h` for your device/network. If the local secrets file is
missing, the firmware still builds using placeholder values from `main/config.h`.

Runtime settings from web API/UI override operational route/mode choices.

## Public Repository Hygiene

Tracked files should not contain local credentials, machine-specific IDE paths,
or generated build artifacts.

- `main/secrets.h` is local-only and ignored
- `main/secrets.example.h` is the public template
- `.vscode/`, `build/`, `managed_components/`, `.pytest_cache/`, and
  `pytest-cache-files-*/` are ignored
- run `git status --ignored --short` when checking for generated artifacts before
  making the repository public

## Testing

The project has four test layers. GitLab CI runs them as separate jobs, and the local commands below mirror those jobs.

### Test Structure

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

### Python Test Environment

```bash
python -m venv .venv
. .venv/bin/activate
python -m pip install -r tests/requirements.txt
```

On Windows, activate the venv with the matching PowerShell/CMD command for your shell.

### Sanity Tests

```bash
python -m pytest tests/sanity -v
```

Sanity tests check repository shape, GitLab CI settings, ESP-IDF image/version pinning, runner tag configuration, test entrypoints, and ignored generated artifacts.

### Unit Tests

```bash
python -m pytest tests/unit/test_host_unit_coverage.py -v
```

The host C unit suite compiles and runs CAN decoder, Modbus decoder, and route-selection tests on the host. It also emits gcov data under `tests/.build/host_unit/coverage/`.

Host unit tests require `gcc` and `gcov`. Missing tools skip on developer machines, but fail in CI so coverage cannot silently disappear.

### Integration Tests

```bash
python -m pytest tests/integration -v
```

Integration tests verify firmware configuration, protocol constants, repository structure, protocol fixture shape, and Modbus CRC helpers.

### Firmware Build Tests

```bash
. /path/to/esp-idf/export.sh
idf.py set-target esp32c6
idf.py build
python -m pytest tests/firmware_build -v
```

Build tests run after `idf.py build` and verify generated `.elf`/`.bin` artifacts, `sdkconfig`, and binary size.

### Coverage Report

After unit tests, generate coverage reports manually with:

```bash
python -m gcovr --root . \
  --object-directory tests/.build/host_unit \
  --filter main \
  --filter tests/unit \
  --xml-pretty --output tests/.build/host_unit/coverage/cobertura.xml \
  --html-details tests/.build/host_unit/coverage/html/index.html \
  --print-summary
```

See `tests/README.md` for more detailed testing documentation.

### Continuous Integration

GitLab CI runs on every commit/merge-request source and is split into these stages:

1. `sanity`
2. `unit`
3. `integration`
4. `build`

Jobs:

- `sanity_tests`: fast repository and CI checks
- `unit_tests`: host C unit tests plus Cobertura/HTML coverage artifacts
- `integration_tests`: firmware configuration and protocol fixture checks
- `build_firmware`: final ESP32-C6 firmware build plus post-build artifact checks

The pipeline definition lives in [`.gitlab-ci.yml`](./.gitlab-ci.yml).

Current CI expectations:

- ESP-IDF image/version is pinned to `espressif/idf:v6.0.1`
- project runner tag is `ubuntu`
- preferred executor is a local Linux **shell** runner
- `IDF_CCACHE_ENABLE=1` and `.ccache/` are used for faster rebuilds
- Python dependencies are installed into `.venv` for Python-only jobs
- firmware build job sources ESP-IDF from `/home/gitlab-runner/esp/esp-idf/export.sh` when running on the local shell runner
- if `cmake`/`ninja` are missing from the ESP-IDF environment, CI installs them into the ESP-IDF Python environment

### GitLab Runner Setup

The project currently targets a project runner named `ubuntu-runner` with tag:

```text
ubuntu
```

Required runner behavior:

- executor: `shell`
- OS: Linux amd64
- tag: `ubuntu`
- runner is allowed to run jobs from this project

Install ESP-IDF for the `gitlab-runner` user:

```bash
sudo -u gitlab-runner -H bash -lc 'mkdir -p ~/esp && cd ~/esp && git clone -b v6.0.1 --recursive https://github.com/espressif/esp-idf.git esp-idf'
sudo -u gitlab-runner -H bash -lc 'cd ~/esp/esp-idf && ./install.sh esp32c6'
sudo -u gitlab-runner -H bash -lc 'cd /tmp && . ~/esp/esp-idf/export.sh && idf.py --version'
```

The last command should print:

```text
ESP-IDF v6.0.1
```

In GitLab project settings:

1. Go to `Settings > CI/CD > Runners` in the GitLab project.
2. Ensure the project runner is online.
3. Ensure the runner tag is `ubuntu`.
4. Keep `Run untagged jobs` optional; the CI file explicitly uses the `ubuntu` tag.
5. Disable shared/instance runners if you want to avoid GitLab SaaS Docker runners.

Successful local-runner logs should show:

```text
on ubuntu-runner
Preparing the "shell" executor
Using Shell executor...
```

If logs show `green-*.saas-linux...`, `docker+machine`, or `Pulling docker image`, the job is still running on a shared GitLab runner instead of the local shell runner.

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
6. `main/protocols/rs485_growatt/rs485_growatt_bridge.c`
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
