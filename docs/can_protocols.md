# CAN Protocol Backlog

Source: JK BMS mobile app screenshots, captured 2026-05-05.

This file tracks the CAN-side protocol list before we start implementing and live-testing them. The app labels below are copied as shown where visible; truncated labels need confirmation from a datasheet or a full-width app capture before implementation.

## JK App CAN Protocol List

| App # | App label | Nominal bitrate | Existing repo mapping | Current status | First implementation note |
| --- | --- | --- | --- | --- | --- |
| `000` | `JK BMS CAN Protocol (250K) V2.0` | `250 kbit/s` | `PROTOCOL_CAN_JKBMS_250K` | In progress | Native JK CAN decoder for `0x02F4`, `0x04F4`, `0x05F4`, `0x07F4`; feeds the universal model and `RS485_PYLON` synthetic responder. |
| `001` | `Deye Low-voltage hybrid inverter CAN commu...` | commonly `500 kbit/s` | `PROTOCOL_CAN_DEYE` | In progress | Deye/Pylon-like frame set is decoded and now feeds `RS485_PYLON`; verify exact JK app profile against live frames. |
| `002` | `PYLON-Low-voltage-V1.2` | commonly `500 kbit/s` | `PROTOCOL_CAN_PYLON` | Active | Current reference CAN profile; already decodes Pylon frames and supports `CAN_PYLON -> RS485_PYLON`. |
| `003` | `Growatt BMS CAN-Bus-protocol-low-voltage_R...` | unknown | `PROTOCOL_CAN_GROWATT` | Partial/active route | Growatt-like CAN cache and `CAN -> RS485_GROWATT` route exist; verify app profile fields. |
| `004` | `Victron_CANbus_BMS_protocol_20170717` | commonly `500 kbit/s` | `PROTOCOL_CAN_VICTRON` | Partial/active route | Pylon-like `0x351`, `0x355`, and `0x356` frames feed the universal model and `RS485_PYLON` synthetic responder; optional alarm/vendor frames still need live validation. |
| `005` | `MEGAREVO_Hybird_BMSCAN_Protocol_V1.0` | unknown | none | Not implemented | New CAN profile; find public PDF or capture frames. |
| `006` | `JK BMS CAN Protocol (500K) V2.0` | `500 kbit/s` | none | Not implemented | Likely same semantic family as app `000`, but different CAN bitrate. |
| `007` | `INVT BMS CAN Bus protocol V1.02` | unknown | none | Not implemented | New CAN profile; find public PDF or capture frames. |
| `008` | `GoodWe LV BMS Protocol (EX/EM/S-BP/BP)` | unknown | `PROTOCOL_CAN_GOODWE` | Partial/active route | The tested JK profile emits Pylon/Deye-like frames `0x351`, `0x355`, `0x356`, `0x359`, `0x35C`, `0x35E`, `0x370`, and `0x371`; the bridge also keeps the older GoodWe `0x456..0x458` map as an alternate dialect. |
| `009` | `FSS-ConnectingBat-TI-en-10 | Version 1.0` | unknown | none | Not implemented | Likely TI/reference BMS CAN profile; verify if it aliases another implemented map. |
| `010` | `MUST PV1800F-CAN communication Protocol...` | unknown | none | Not implemented | New inverter-side CAN profile; label truncated in screenshot. |
| `011` | `LuxpowerTek Battery CAN protocol V01` | unknown | none | Not implemented | New CAN profile; find public PDF or capture frames. |

## Current Repo CAN Support

Implemented or partially usable today:

- `CAN_PYLON`: active Pylon CAN cache, decoded telemetry, and `CAN_PYLON -> RS485_PYLON` synthetic responder path.
- `JKBMS_CAN_250K`: native JK CAN V2.0 profile at 250 kbit/s, decoded into the universal model and usable as a synthetic `RS485_PYLON` source.
- `CAN_DEYE`: active CAN decode path for Deye-like Pylon frame set; supports `CAN_DEYE -> RS485_PYLON` via the universal model.
- `CAN_GROWATT`: Growatt-like CAN cache and bridge route toward `RS485_GROWATT`; needs live validation against JK app protocol `003`.
- `CAN_GOODWE`: active basic GoodWe LV decode path for the live Pylon/Deye-like JK profile and the alternate `0x456..0x458` GoodWe map; supports `CAN_GOODWE -> RS485_PYLON` via the universal model.
- `CAN_VICTRON`: active base Victron/Pylon-like decode path for `0x351`, `0x355`, and `0x356`; supports `CAN_VICTRON -> RS485_PYLON` via the universal model.
- `CAN_SOFAR`, `CAN_SMA`: supported by existing generic route IDs, but not visible in the current JK app screenshots.

## Suggested Implementation Order

1. `000 - JK BMS CAN Protocol (250K) V2.0`: implement and live-test first because it is the currently selected JK app protocol.
2. `002 - PYLON-Low-voltage-V1.2`: keep as the known-good CAN reference and regression target.
3. `003 - Growatt BMS CAN-Bus`: verify whether the current `CAN_GROWATT` route matches the JK app profile.
4. `001 - Deye Low-voltage`: live-test the corrected `CAN_DEYE -> RS485_PYLON` route with JK app protocol `001`.
5. `008 - GoodWe LV`: live-test the basic `CAN_GOODWE -> RS485_PYLON` route, then add optional frames as captures/documentation allow.
6. `004 - Victron CANbus`: live-test the base `CAN_VICTRON -> RS485_PYLON` route, then add optional alarm/vendor frames as captures/documentation allow.
7. `006 - JK BMS CAN Protocol (500K) V2.0`: reuse the native JK CAN frame map if the payload is confirmed identical.
8. `005/007/009/010/011`: add only when we have a datasheet or live frame capture.

## Integration Standard

For every CAN profile, match the same quality bar used for the RS485 protocol work:

- decode real source fields into the universal battery model
- expose web telemetry for pack, cells, temperatures, warnings, alarms, and protections when the protocol provides them
- keep raw diagnostics visible when field names are not yet known
- add host tests with representative frames
- build firmware and validate live `/api/telemetry`, `/api/logs`, and inverter behavior
