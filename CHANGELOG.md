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
- `RS485_GROWATT -> RS485_PYLON` bridge-mode route for JK UART protocol `006 - Growatt_BMS_RS485_Protocol_1xSxxP_ESS_Rev2.01`.
- `VOLTRONIC_MODBUS` BMS poller/decoder for JK UART protocol `007 - Voltronic_Inverter_and_BMS_485`, including all-cell voltages, individual temperatures, warning states, alarm/protection registers, and charge/discharge limit/status telemetry.
- `RS485_VOLTRONIC -> RS485_PYLON` bridge-mode route for Pylon-compatible inverter responders.
- `CHINA_TOWER_MODBUS` BMS poller/decoder for JK UART protocol `008 - China tower shared battery cabinet V2.0`, including pack voltage, `SOC`, all-cell voltages, cell extremes, and individual temperatures.
- `RS485_CHINA_TOWER -> RS485_PYLON` bridge-mode route for Pylon-compatible inverter responders.
- `WOW_MODBUS` BMS poller/decoder for JK UART protocol `009 - WOW_RS485_Modbus_V1.3`, using a PACE-compatible V1.3 register map as the initial live-test implementation.
- `RS485_WOW -> RS485_PYLON` bridge-mode route for Pylon-compatible inverter responders.
- `RS485_WOW -> RS485_GROWATT` bridge-mode route for Growatt-compatible RS485 inverter responders, fed through the shared battery model.
- PulseView `wow_modbus` decoder for JK UART protocol `009 - WOW_RS485_Modbus_V1.3`, with Modbus CRC, request/response tracking, runtime values, cell voltages, and temperatures.
- `JKBMS_CAN_250K` decoder for JK app protocol `000 - JK BMS CAN Protocol (250K) V2.0`, covering pack voltage/current/SOC, cell extremes, temperatures, and alarm severity fields.
- `JKBMS_CAN_250K -> RS485_PYLON` synthetic responder route through the universal battery model.
- CAN protocol backlog documentation under `docs/can_protocols.md`.
- `CAN_DEYE -> RS485_PYLON` route support for JK app protocol `001 - Deye Low-voltage hybrid inverter CAN`.
- Field-tested `JKBMS Pylon CAN -> EASUN Pylon CAN` direct forward path for split CAN ports (`CAN1 -> CAN2`, `500 kbit/s`).
- Runtime `Fake Inverter Data` override for Pylon CAN forward mode, including outgoing `0x355` SOC/SOH edits.
- Optional EASUN/Pylon 24V diagnostic CAN sender for isolated inverter-side tests, disabled by default.
- `DALY_RS485` BMS poller/decoder for the Daly proprietary RS485 protocol, including pack telemetry, cell data where exposed, temperatures, MOS/balance state, and failure/status bits.
- `RS485_DALY -> RS485_PYLON` bridge-mode route for Pylon-compatible inverter responders.
- `RS485_DALY -> CAN_PYLON` bridge-mode route, field-tested with Daly RS485 on `RS485_1` feeding EASUN Pylon CAN on `CAN2`.
- `RS485_PYLON -> CAN_PYLON` bridge-mode route, field-tested with JK Pylon RS485 on `RS485_1` feeding Pylon CAN output on `CAN2`.
- `RS485_PYLON -> RS485_GROWATT` bridge-mode route, using the Pylon RS485 active poller as the BMS source and the existing Growatt RS485 responder fed from the shared battery model.
- `RS485_PYLON -> CAN_GROWATT` bridge-mode route, using the Pylon RS485 active poller as the BMS source and the Growatt CAN sender fed from the shared battery model.
- `RS485_JKBMS -> CAN_GROWATT` bridge-mode route, so JK Modbus/native RS485 sources can keep full all-cell telemetry in the web/API while feeding Growatt CAN inverter frames.
- Pylon RS485 `0x42` cell-information parser and host regression coverage, so BMS variants that expose that frame can populate the web/API `cells_v[]` list.
- Experimental `DALY_CAN` BMS poller/decoder for the Daly proprietary CAN protocol.
- Experimental `DALY_CAN -> RS485_PYLON` bridge-mode route through the shared battery model and Pylon synthetic responder.
- Selectable `115200` RS485 variants for `JKBMS_MODBUS` and `RS485_PYLON`, reusing the same decoders/responders as the existing `9600` variants.
- GitLab CI pipeline for automatic ESP32-C6 builds and host-side test execution on push/merge request.
- Separate CI suites for sanity, unit, integration, and firmware-build validation, with JUnit artifacts and coverage reports.
- Host-side regression tests for `JKBMS_MODBUS` source freshness and Modbus decoder cache timestamps.
- Public `main/secrets.example.h` template for Wi-Fi credentials, with local `main/secrets.h` ignored by Git.
- Custom `8MB` single-app partition table for ESP32-C6-WROOM-1-N8 modules.
- PulseView/libsigrokdecode `Pylon RS485` protocol decoder under `tools/sigrok/decoders/pylon_rs485`, plus parser regression tests using LA2016 field-captured Pylon frames.
- PulseView/libsigrokdecode `Pylon CAN` protocol decoder under `tools/sigrok/decoders/pylon_can`, including Pylon/JK CAN IDs, raw CANH/CANL input modes for LA2016 captures, install/launcher scripts, and host regression tests.
- PulseView/libsigrokdecode `Growatt RS485` protocol decoder under `tools/sigrok/decoders/growatt_rs485`, including Modbus RTU request/response tracking, CRC validation, Growatt register names/scaling, warning/protection/status fields, and the `0x0071..0x0080` cell-voltage block.
- PulseView/libsigrokdecode `Growatt CAN` protocol decoder under `tools/sigrok/decoders/growatt_can`, including Growatt low-voltage CAN IDs `0x311..0x323`, LA2016 raw CANH/CANL input modes, endian-tolerant pack/cell decoding, and host regression tests.
- PulseView/libsigrokdecode `Deye CAN` protocol decoder under `tools/sigrok/decoders/deye_can`, including Deye/JK low-voltage CAN IDs `0x351`, `0x355`, `0x356`, `0x359`, `0x35C`, `0x35E`, `0x370`, and `0x371`, LA2016 raw CANH/CANL input modes, visible version tag, and host regression tests.
- PulseView/libsigrokdecode `GoodWe CAN` protocol decoder under `tools/sigrok/decoders/goodwe_can`, including classic GoodWe IDs `0x453`, `0x455`, `0x456`, `0x457`, `0x458`, the field-tested JK GoodWe Pylon/Deye-like dialect IDs, LA2016 raw CANH/CANL input modes, visible version tag, and host regression tests.
- PulseView/libsigrokdecode `Victron CAN` protocol decoder under `tools/sigrok/decoders/victron_can`, including Victron/Pylon-like IDs `0x351`, `0x355`, `0x356`, field-captured ASCII/raw frames, tentative `0x373` cell/temperature decoding, LA2016 raw CANH/CANL input modes, visible version tag, and host regression tests.
- `CAN_VICTRON -> RS485_PYLON` bridge-mode route using the Victron/Pylon-like CAN frames `0x351`, `0x355`, and `0x356` as a universal-model source for the Pylon RS485 synthetic responder.
- PulseView/libsigrokdecode `JKBMS Modbus` protocol decoder under `tools/sigrok/decoders/jkbms_modbus`, including JK app profile `001/013` Modbus RTU traffic, request/response tracking, CRC validation, runtime register annotations, cell-voltage blocks, pack/SOC/temperature/capacity fields, and host regression tests.
- PulseView/libsigrokdecode `JKBMS RS485 Native` protocol decoder under `tools/sigrok/decoders/jkbms_rs485_native`, including JK `4E 57` binary read-all frames, data-ID annotations, all-cell voltage list decoding, pack/SOC/temperature/status/alarm fields, and host regression tests.
- PulseView/libsigrokdecode `JKBMS CAN` protocol decoder under `tools/sigrok/decoders/jkbms_can`, including JK native CAN V2.0 frame IDs `0x02F4`, `0x04F4`, `0x05F4`, `0x07F4`, extended `0x18E*28F4` cell-voltage frames, `0x18F228F4` temperatures, raw CANH/CANL input modes, and host regression tests.
- PulseView/libsigrokdecode `PACE Modbus` protocol decoder under `tools/sigrok/decoders/pace_modbus`, including PACE RS485 Modbus V1.3 request/response tracking, CRC validation, runtime registers, all-cell voltage block, temperature registers, and host regression tests.
- PulseView/libsigrokdecode `SMA CAN` protocol decoder under `tools/sigrok/decoders/sma_can`, including SMA Sunny Island-compatible CAN IDs `0x351`, `0x355`, `0x356`, raw/ASCII annotations for `0x359`, `0x35A`, `0x35E`, `0x35F`, LA2016 raw CANH/CANL input modes, visible version tag, and host regression tests.
- PulseView/libsigrokdecode `Sofar CAN` protocol decoder under `tools/sigrok/decoders/sofar_can`, including Sofar-compatible CAN IDs `0x351`, `0x355`, `0x356`, observed support frames `0x359`, `0x35C`, `0x35E`, `0x35F`, `0x370`, `0x371`, LA2016 raw CANH/CANL input modes, visible version tag, and host regression tests.

### Changed

- `RS485_PYLON <-> RS485_PYLON` passthrough now honors the runtime `Fake Inverter Data` override by answering inverter Pylon requests from the synthetic fake model instead of forwarding live BMS frames.
- Pylon RS485 diagnostics now keep active probing enabled for known Seplos/Pylon pack addresses and can apply a configurable SOC floor to Seplos/Pylon `0x61` responses when the BMS reports an inverter-stopping low SOC.
- Seplos RS485 field notes were updated after root-cause analysis showed the tested Seplos BMS does not provide RS485 bias resistors and needs the bridge/external hardware to bias the bus.
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
- Sigrok/PulseView documentation now includes a persistent combined decoder installation flow so built-in decoders such as `CAN` stay visible alongside `Pylon CAN` and `Pylon RS485`.
- Sigrok/PulseView workflow now treats this firmware repository as the decoder workbench/source of truth; the separate decoder repository is reserved for validated/published decoders only.
- Added a PulseView decoder coverage/backlog document so register-map coverage, tentative fields, and publication rules stay explicit while decoders are refined.
- `JKBMS CAN` PulseView decoder was bumped to visible version `v2026.07.03a`, aligned with the ESP32 JK CAN map for `1..25` extended cell-voltage decoding, and now exposes vendor-reference capacity/charge-info frames without inventing meanings for still-raw frames.
- `JKBMS_MODBUS` now treats the `0x12A0..0x12A1` alarm/status word as a raw
  candidate unless validated by a real fault-state capture; this prevents live
  no-alarm values such as `0x2344_6400` from surfacing as false UI protections
  or inverter-facing alarm masks.
- Local VS Code ESP-IDF settings were removed from version control and ignored because they contain machine-specific paths.
- CAN telemetry now expires cached frames before periodic snapshot decoding, preventing stale `CAN_PYLON`/Deye/JK/Growatt cache data from being republished as fresh web telemetry after a BMS disconnect.
- `RS485_PYLON -> RS485_PYLON` bridge mode stays transparent passthrough for inverter compatibility, while stale native Pylon telemetry is cleared from the web/shared model when BMS responses stop.
- Runtime settings now boot from the NVS-backed `/api/settings` store by default again; `RUNTIME_SETTINGS_FORCE_DEFAULTS` is disabled and kept only as an explicit diagnostic switch.
- Documentation now calls out the practical workaround for JK Pylon RS485 profiles that reject `0x42`: select a JK-native BMS-side protocol when `cells_v[]` telemetry is required.
- Pylon RS485 probing now clears a stale preferred BMS address after source timeout/repeated no-response events, so swapping from one Pylon-like BMS to another resumes address scanning instead of staying pinned to the old address.
- Pylon RS485 `0x42` cell-info probing now tries the discovered response address before the generic address scan and includes `FF`, full-address, low-nibble, `01`, `00`, and empty-payload variants; the tested Seplos Pylon profile on `0x12` returns OK/no-payload for these requests.
- Growatt CAN inverter output now follows the selected runtime CAN inverter port and can publish the main low-voltage Growatt CAN frame set from the shared battery model.
- Flash size is configured for `8MB`, and the app partition was expanded from `1MB` to `7MB`.
- `bms_decoded_packet_t` now carries richer decoded telemetry for protocols that expose per-cell voltages, per-sensor temperatures, warning/protection/fault masks, status flags, and balance flags.
- Pylon synthetic status generation can now use explicit PACE MOSFET charge/discharge flags while keeping the conservative generic fallback for non-native sources.
- `RS485_GROWATT` BMS polling now follows the selected BMS RS485 port, expires stale Modbus cache data, and publishes per-cell voltage registers when available.
- `RS485_GROWATT` BMS telemetry now decodes Growatt `0x0014` error/protection bits and `0x0022` warning bits for JK UART profile `006` instead of leaving the web alert cards empty.
- `RS485_GROWATT` web telemetry now displays only the single live pack temperature exposed by register `0x0018`, instead of duplicating it as MOS/T1/T2/T4/T5.
- `VOLTRONIC_MODBUS` web telemetry follows the PACE/Growatt integration standard: expose real cells and temperatures, keep raw alert/status diagnostics visible, and feed the existing Pylon responder with the decoded battery model.
- `VOLTRONIC_MODBUS` now supports the live-tested Seplos Voltronic RS485 behavior: classic slave-`1` single-register `0x03` reads, 16-bit byte-count responses such as `01 03 00 02 ... CRC`, and exception responses for unsupported broad ranges.
- `VOLTRONIC_MODBUS` poll frames retain compatibility with standard Modbus/JK variants through the shared decoder while using the Seplos-safe single-register poll plan for the active field profile.
- `CHINA_TOWER_MODBUS` uses the bench-observed JK profile `008` register layout, with cell millivolts starting at `0x0009` and temperature registers kept at the live-tested offsets.
- `CHINA_TOWER_MODBUS` web telemetry now treats the compact summary temperatures as the only confirmed live sensors and hides non-live `Battery T4`/`Battery T5` values instead of displaying Pylon-template leftovers.
- `CHINA_TOWER_MODBUS` now captures the live-tested raw candidate warning/protection/status registers (`0x0019..0x001B`) without assigning unconfirmed names; non-zero unknown bits are preserved for future correlation with the JK app.
- `WOW_MODBUS` is integrated as its own runtime/source protocol so JK profile `009` can be validated without reusing or mutating another protocol's cache interpretation.
- Web/API telemetry and route-selection tests now cover the `WOW_MODBUS -> RS485_PYLON` path.
- `WOW_MODBUS` has been live-validated with the BMS set to JK UART profile `009`, with the inverter accepting the translated Pylon responder data.
- RS485 UART initialization now derives baud rate per physical RS485 port from the selected runtime protocol, allowing mixed routes such as `JKBMS_MODBUS_115200 -> RS485_PYLON`.
- CAN initialization now derives bitrate per physical CAN port from the selected runtime protocol, with `JKBMS_CAN_250K` and `DALY_CAN` using `250 kbit/s` and existing CAN profiles remaining at `500 kbit/s`.
- `CAN_DEYE` now updates the universal battery model directly from live CAN frames instead of waiting for the periodic diagnostic snapshot.
- Direct Pylon CAN forward mode can now apply runtime fake inverter data before transmitting frames to the inverter.
- The optional Pylon CAN `16S->8S` scaler remains available behind `CAN_FORWARD_PYLON_16S_TO_8S_ENABLE`, but is disabled by default after the EASUN field test accepted direct JKBMS values.
- `pylon_inverter_task` now acts as a real Pylon CAN sender when the inverter side is `CAN_PYLON`, using the shared battery model produced by sources such as `DALY_RS485`.
- RS485 Pylon summary frames now publish the shared battery model so native Pylon RS485 sources can feed inverter-facing Pylon CAN frames.
- Pylon RS485 active probing now caches the discovered response address, avoiding long stale gaps after a JK BMS answers on address `0x02`.
- JK-like Pylon RS485 pack-voltage scaling is normalized from the raw millivolt-like `0x61` word to the real low-voltage pack value before telemetry and CAN transmission.
- Pylon RS485 active probing now tries common `0x42` cell-information payload variants and backs off after a full unsupported scan to avoid noisy retries on BMS profiles that reject the command.
- The current compile-time fallback route is `MODE_BRIDGE`, `RS485_PYLON` on `RS485_1` to `CAN_PYLON` on `CAN2`; normal boots load saved NVS runtime settings unless the diagnostic `RUNTIME_SETTINGS_FORCE_DEFAULTS` switch is explicitly enabled.
- Daly RS485 source freshness is kept at `10s`, matching the desired inverter fail-safe behavior when the BMS disappears.
- Pylon RS485 source freshness is kept at `10s`, so the Pylon CAN sender stops after the active-probe freshness window if the JK/Pylon RS485 source disappears.
- Daly CAN documentation now records the 2026-05-30 no-ACK field result so future debugging starts at wiring/BMS CAN enablement instead of repeating protocol-level tests.

### Fixed

- Web settings route hints now follow the currently selected form values and failed saves surface an explicit timeout/error instead of leaving the UI stuck on `Saving...`.
- Fixed inverter fault on `JKBMS_MODBUS -> RS485_PYLON`.
- Fixed the inactive/stale source path for `CAN_DEYE -> RS485_PYLON`, where the Deye decoder existed but the Pylon responder route was not being armed as a supported synthetic source.

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
- Fixed `CAN_GOODWE -> RS485_PYLON` route support by decoding the live GoodWe/Pylon-like CAN frames into the universal battery model and arming the Pylon RS485 synthetic responder for GoodWe CAN sources; the alternate GoodWe `0x456..0x458` map remains supported.
- Fixed `CAN_VICTRON -> RS485_PYLON` route support by treating Victron `0x351/0x355/0x356` as Pylon-like CAN source frames and deriving charge/discharge state for the Pylon `0x63` responder payload.
- Fixed Pylon CAN inverter sender stale behavior: after a Daly RS485 disconnect, it stops transmitting after the `10s` Daly freshness window instead of replaying the last valid packet.
- Fixed near-full application partition warnings on ESP32-C6-WROOM-1-N8 by using the available `8MB` flash.
- Removed real Wi-Fi credentials from tracked source files.
- Fixed early PACE web telemetry showing identical temperature values by mapping PACE temperature registers to their real source labels (`MOS`, `Battery T1`, `Battery T2`, `Battery T4`, `Battery T5`) instead of copying a generic average temperature.

### Operational Notes

- Seplos RS485 requires a fail-safe bias network on the Seplos-facing bus. Without bias, the idle A/B state can float and the bridge may capture noise-like bytes even though the same BMS communicates with an inverter that provides proper bias.
- EASUN Pylon CAN is validated on split CAN ports after repairing the inverter CAN transceiver: JKBMS on `CAN1`, EASUN on `CAN2`, direct Pylon CAN forward at `500 kbit/s`.
- EASUN emits repeated `0x305 [00 00 00 00 00 00 00 00]` frames when its CAN hardware is healthy.
- EASUN also accepts Daly-derived Pylon CAN from the bridge: Daly RS485 decoded around `28.3 V`, `SOC=99%`, 8 cells, charge/discharge enabled, then sent on `CAN2` as Pylon CAN with status `0xC0`.
- JK Pylon RS485 also feeds Pylon CAN through the bridge: JK on `RS485_1` at `9600 bps`, `SOC=100%`, pack around `57.14 V`, status `0xC0`, then sent on `CAN2` as Pylon CAN.
- The tested JK `PYLON RS485` profile does not expose full per-cell voltages through Pylon `0x42`: `FF`, `00`, `01`, and empty-payload requests were rejected on address `0x02` with response code `0x04`, while other candidate pack addresses did not answer. Only `0x61` cell min/max are available in that profile.
- A `58 V` Pylon CAN snapshot on `CAN1` during this investigation was from the old CAN forward route, not the 24V Daly pack.
- Daly CAN is implemented but not field-validated: the tested BMS did not ACK `250 kbit/s` extended requests on CAN1 or CAN2, including both the datasheet-style DLC-8 reserved request and the public Linux CAN DLC-0 request form.
- `Fake Inverter Data` now works as an inverter-facing override for Pylon RS485 passthrough routes as well as synthetic translator routes; remember it is runtime-only and must be reapplied after reset or flash.
- `Fake Inverter Data` also works for Pylon CAN forward mode; fake `SOC=0%` was field-confirmed to influence EASUN behavior, and resulting inverter-side CAN no-ACK/TX failures can be expected if the inverter stops or sleeps.
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

### Daly CAN no-ACK on field hardware

The `DALY_CAN` task is implemented from the Daly CAN datasheet and public
Linux CAN examples, but the 2026-05-30 bench BMS did not ACK requests.

Tested combinations:

- `CAN1` and `CAN2` at `250 kbit/s`
- extended SOC request ID `0x18900140` for `BMS_ID=1`
- datasheet-style request with 8 reserved data bytes
- public-thread request style with DLC 0

Observed result:

- no valid Daly CAN frames were received
- TWAI counters showed the no-ACK pattern: `txErr=128`, increasing `txFail`,
  `rxErr=0`, and `rxMiss=0`

Current recommendation:

- treat Daly CAN as experimental until a USB-CAN/direct inverter capture proves
  the BMS ACKs/responds on the same CAN pair
- check CAN-H/CAN-L, common ground, termination, BMS CAN enable/profile, and
  whether the connected Daly port is the direct CAN port or a separate Daly
  interface-board path

### Some protocol implementations are still scaffold/partial

- `pylon_bms_task.c` and `pylon_inverter_task.c` remain placeholders
- several CAN protocol folders are present mainly for maps/constants and are not yet full end-to-end pipelines

## Maintenance Rule

When a hardware bug is solved in a way that depends on protocol semantics, add a short note here and, when helpful, a matching note in `README.md`.
