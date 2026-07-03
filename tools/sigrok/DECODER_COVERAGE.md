# PulseView Decoder Coverage

This repository is the decoder workbench. Keep in-progress decoders here until
they are validated on captures or live hardware. The separate
`sigrok-pylon-bms-decoders` repository is only for the small, confirmed public
set.

## Working Rules

- Decode every field that has a reliable map or a field-tested meaning.
- Keep unknown but relevant bytes/words visible as raw hex instead of hiding
  them.
- Mark uncertain values as tentative until they are confirmed against a live
  capture, firmware telemetry, or vendor app data.
- Do not publish a decoder to the separate decoder repository until it has
  tests, README usage notes, and a real capture/hardware validation note.
- Keep PulseView helpers synchronized with the firmware register maps under
  `main/protocols/*/*_registers_map.h`.

## Active Workbench Decoders

| Decoder | Source map/reference | Current coverage | Known gaps / validation notes |
| --- | --- | --- | --- |
| `jkbms_modbus` | `main/protocols/jkbms_modbus/jkbms_modbus_registers_map.h`, `datasheets/Ji_Kong_BMS_RS485_Modbus_Universal_Protocol_V1.1.pdf` | Modbus RTU request/response framing, CRC, poll-block tracking, runtime cells `0x1200..0x123E`, cell summary `0x1244..0x1248`, MOS/T1/T2 temps, pack voltage/current/power candidates, SOC/SOH, capacity, cycles, `0x12A0` alarm/status candidate | Config/device/factory areas are not decoded yet. CRC-bad frames are decoded only as tentative. Cell indexes above `C32` are not promoted as valid cells. Live `0x12A0` non-zero values are kept as candidates until correlated with a real JK alarm state. |
| `jkbms_rs485_native` | `main/protocols/jkbms_rs485/jkbms_rs485_native.c`, vendor JK native references | `4E 57` binary framing, data-ID annotations, all-cell list `0x79`, temperatures, voltage/current/SOC, cycles, capacity, alarm/status IDs | Needs more live captures for settings/config frames and less common IDs. |
| `jkbms_can` | `main/protocols/jkbms_can/jkbms_can_protocol.*`, JK CAN V2.0 references | Native JK CAN at `250 kbit/s`, visible decoder version `v2026.07.03a`, standard pack/status frames, extended cell-voltage frames capped to cells `1..25`, temperature frames, capacity/charge-info frames, raw CANH/CANL input modes | Frames with no reliable field map, such as `0x18F328F*`, `0x18F428F*`, and `0x18F528F*`, stay raw. Needs more live validation for every extended frame variant and BMS-ID suffix behavior. |
| `growatt_rs485` | `main/protocols/rs485_growatt/rs485_growatt_registers_map.h`, `datasheets/Growatt_BMS_RS485_Protocol_1xSxxP_ESS_Rev2.01.pdf` | Modbus RTU request/response tracking, CRC, main status block, warning/error/protection fields, pack values, limits, cell min/max, per-cell block `0x0071..0x0080` | Device info strings and uncommon/variant registers should be decoded when captures show them. |
| `growatt_can` | `main/protocols/growatt/growatt_registers_map.h`, field captures | Classic CAN IDs `0x311..0x323`, pack/status/limit/alarm/cell/temp/metadata frames, raw CANH/CANL input modes | Optional IDs `0x324..0x325` need capture-backed decode before being promoted beyond raw annotations. |
| `pylon_rs485` | `datasheets/RS485-protocol-pylon-low-voltage-V3.3-20180821.pdf`, firmware Pylon bridge | ASCII frame parsing, length/checksum, command/response tracking, common `0x61`, `0x62`, `0x63`, and `0x42` cell-info payloads where present | Some Pylon-compatible BMS profiles answer `0x42` with OK/no-payload or reject it; keep those as protocol behavior, not decoder failure. |
| `pylon_can` | `main/protocols/pylon/pylon_can_protocol.*`, field captures | Classic CAN Pylon IDs used by JK/EASUN flows, including pack, SOC/SOH, limits, status, manufacturer, and optional cell/temperature frame | Vendor-specific optional frames should stay raw until capture-backed. |

## Promotion Checklist

Before copying a decoder to the public decoder-only repository:

1. Add or update parser tests under `tests/tools`.
2. Verify the decoder loads in PulseView together with built-in `uart`/`can`.
3. Validate on at least one real capture or live hardware session.
4. Update `tools/sigrok/README.md` with usage settings.
5. Only then copy the decoder into the external repository and update that
   README/screenshots.
