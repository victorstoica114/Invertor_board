# Sigrok / PulseView Tools

This directory contains optional analysis helpers for external logic-analyzer
workflows.

## Development Rule

This firmware repository is the workbench/source of truth for decoder
development. New or still-changing decoders stay here under
`tools/sigrok/decoders` until they are tested on captures and validated
together on real hardware.

The separate `sigrok-pylon-bms-decoders` repository is only for publishing the
small, validated decoder set. Do not move a decoder there until it has been
accepted as stable. For live debugging, always launch PulseView from this
repository or run this repository's installer, so in-test decoders such as
JKBMS remain available.

Coverage and publication rules are tracked in
[`DECODER_COVERAGE.md`](DECODER_COVERAGE.md).

Version rule: every custom decoder that is under active work should include a
visible version/build tag in its PulseView `name`/`longname`. Bump that tag
whenever decoder behavior changes, so stale copies installed under
`C:\ProgramData\libsigrokdecode\decoders` or cached by an already running
PulseView instance are obvious in the decoder selector and stack labels.

Available PulseView/libsigrokdecode protocol decoders:

- `pylon_rs485`: Pylon-compatible RS485 ASCII frames, stacked above `uart`
- `pylon_can`: Pylon-compatible Classic CAN frames, standalone CAN decoder
- `growatt_rs485`: Growatt RS485 Modbus RTU frames, stacked above `uart`
- `growatt_can`: Growatt-compatible Classic CAN frames, standalone CAN decoder
- `deye_can`: Deye-compatible Classic CAN frames, standalone CAN decoder;
  current visible name is `Deye CAN v2026.07.03a`
- `goodwe_can`: GoodWe-compatible Classic CAN frames, standalone CAN decoder;
  current visible name is `GoodWe CAN v2026.07.03a`
- `victron_can`: Victron-compatible Classic CAN frames, standalone CAN decoder;
  current visible name is `Victron CAN v2026.07.03a`
- `jkbms_modbus`: JK BMS RS485 Modbus poller frames, stacked above `uart`;
  current visible name is `JKBMS Modbus v2026.07.02b`
- `jkbms_rs485_native`: JK native binary RS485 frames, stacked above `uart`
- `jkbms_can`: JK native CAN V2.0 frames, standalone CAN decoder;
  current visible name is `JKBMS CAN v2026.07.03a`

## Pylon RS485 Decoder

The `decoders/pylon_rs485` directory is a PulseView/libsigrokdecode protocol
decoder for Pylon-compatible RS485 ASCII frames.

It stacks on top of the built-in `uart` decoder:

```text
logic -> uart -> pylon_rs485
```

Typical Pylon RS485 settings:

- baud: `9600`
- data bits: `8`
- parity: `none`
- stop bits: `1`
- bit order: `lsb-first`
- line inversion: depends on the probe point/transceiver output

On Windows, copy the whole `pylon_rs485` directory to one of PulseView's decoder
directories, for example:

```powershell
Copy-Item -Recurse tools\sigrok\decoders\pylon_rs485 "$env:ProgramData\libsigrokdecode\decoders\pylon_rs485"
[Environment]::SetEnvironmentVariable(
    "SIGROKDECODE_DIR",
    "$env:ProgramData\libsigrokdecode\decoders",
    "User"
)
```

For local development, point PulseView straight at this repository's decoder
root instead:

```powershell
[Environment]::SetEnvironmentVariable(
    "SIGROKDECODE_DIR",
    "C:\Users\Admin\Documents\test-CAN+RS485\tools\sigrok\decoders",
    "User"
)
```

Restart PulseView after copying. Add a `UART` decoder first, then add
`Pylon RS485` above it in the decoder stack.

If PulseView is started with only the repository decoder root in
`SIGROKDECODE_DIR`, built-in decoders such as `CAN` may disappear from the
selector. Use the local launcher to build a single combined decoder directory
containing both PulseView's built-in decoders and this repository's custom BMS
decoders:

```powershell
.\tools\sigrok\start-pulseview.ps1
```

For the normal Start Menu/Desktop PulseView shortcut, install a persistent
combined decoder directory under `C:\ProgramData` and point the user
`SIGROKDECODE_DIR` there:

```powershell
.\tools\sigrok\install-pulseview-decoders.ps1
```

This repository's development machine also has Desktop shortcuts named
`PulseView Workbench Decoders`, `PulseView BMS Decoders`, and
`PulseView Pylon` which run `start-pulseview.ps1`. Use
`PulseView Workbench Decoders` for normal analyzer/debug work; it rebuilds a
combined decoder bundle from PulseView's built-ins plus every custom decoder
directory under `tools/sigrok/decoders`.

## Offline sigrok-cli Analysis

`sigrok-cli` is useful for repeatable checks on captures saved from PulseView.
The official Windows `sigrok-cli 0.7.2` package does not include the Kingst
LA2016 hardware driver used by this machine's PulseView build, so direct CLI
capture from the LA2016 is not currently available. Use PulseView for LA2016
captures and screenshots, save the session as `.sr`, then run CLI analysis on
that file.

The helper below rebuilds a combined decoder bundle from PulseView's built-in
decoders plus this repository's custom decoders, then runs a stacked UART
decoder and protocol decoder:

```powershell
.\tools\sigrok\decode-sr.ps1 `
    -InputFile C:\Users\Admin\Desktop\capture.sr `
    -Protocol jkbms_modbus `
    -Rx CH1 `
    -Baud 115200
```

Useful variants:

```powershell
.\tools\sigrok\decode-sr.ps1 -InputFile C:\Users\Admin\Desktop\pylon_RS485.sr -Protocol pylon_rs485 -Rx CH0 -Baud 9600
.\tools\sigrok\decode-sr.ps1 -InputFile C:\Users\Admin\Desktop\growatt.sr -Protocol growatt_rs485 -Rx CH1 -Baud 9600
```

For JKBMS Modbus captures from JK app profile `001 - JK BMS RS485 Modbus V1.0`,
the logic-level transceiver RXD setting is `115200 8N1`, LSB-first, RX invert
off, sample point `50%`.

For direct digital probing of the RS485 A/B wires with the LA2016, validate
both polarities instead of trusting the label. On the `new.sr` field capture
from July 2, 2026, the clean decode was `CH0`, RX invert `yes`, sample point
`58%`: 20 complete Modbus frames, 20 CRC OK, 0 CRC BAD. `CH1` non-inverted at
50% showed 16 CRC OK, 3 CRC BAD, and 1 incomplete frame on the same capture.

## Pylon CAN Decoder

The `decoders/pylon_can` directory is a PulseView/libsigrokdecode protocol
decoder for Pylon-compatible Classic CAN frames. It is standalone on the CAN
RX logic signal and internally reuses the built-in CAN decoder, so add it
directly from the decoder selector.

Typical Pylon CAN settings:

- CAN RX/H: the analyzer channel connected to the CAN transceiver `RXD` logic
  signal, to digitized `CANL`, or to digitized `CANH` depending on `Input mode`
- CANL: optional second analyzer channel, only used by `CANH/CANL digital diff`
- nominal bitrate: `500000`
- fast bitrate: unused for Classic CAN; leave the default or set it to
  `500000`
- sample point: start with `70%`; if annotations look unstable, try `75%` or
  `80%`
- input mode:
  - `rx/canl-direct`: default. Use with transceiver `RXD`, or with `CANL` if
    the logic-analyzer threshold turns recessive into `1` and dominant into `0`.
  - `canh-inverted`: use with `CANH` when the analyzer shows recessive as `0`
    and dominant as `1`.
  - `canh-canl-diff`: use CH0 as `CANH` and the optional `CANL` channel as
    CH1. This works with digitized bus wires, not with analog differential
    voltages; the analyzer thresholds still matter.

The decoder keeps the normal CAN annotation rows and adds Pylon-specific rows
for the field-tested JK/Pylon CAN IDs `0x351`, `0x355`,
`0x356`, `0x359`, `0x35C`, `0x35E`, `0x370`, `0x371`, and the Pylon `0x373`
cell/temperature frame when present.

On Windows, copy the whole `pylon_can` directory alongside `pylon_rs485`, or
point `SIGROKDECODE_DIR` at this repository's decoder root:

```powershell
Copy-Item -Recurse tools\sigrok\decoders\pylon_can "$env:ProgramData\libsigrokdecode\decoders\pylon_can"
```

If PulseView was installed under `C:\Program Files\sigrok\PulseView`, add that
directory to the user `PATH` so `pulseview` can be launched from a fresh shell.

## Growatt RS485 Decoder

The `decoders/growatt_rs485` directory is a PulseView/libsigrokdecode protocol
decoder for Growatt-compatible RS485 Modbus RTU frames.

It stacks on top of the built-in `uart` decoder:

```text
logic -> uart -> growatt_rs485
```

Typical Growatt RS485 settings:

- baud: `9600`
- data bits: `8`
- parity: `none`
- stop bits: `1`
- bit order: `lsb-first`
- line inversion: depends on the probe point/transceiver output

The decoder recognizes Modbus RTU function `0x03`/`0x04` requests and
responses, validates CRC-16, tracks the request range so response words can be
annotated with register addresses, and decodes the Growatt BMS registers used
by this firmware:

- status/protection/warning fields around `0x0013`, `0x0014`, and `0x0022`
- pack `SOC`, voltage, current, temperature, capacity, cycle, and limit fields
- cell min/max registers and indexes
- per-cell voltage block `0x0071..0x0080`

Add a `UART` decoder first, then add `Growatt RS485` above it in the decoder
stack. On Windows, copy the whole `growatt_rs485` directory alongside the other
custom decoders, or run the installer script:

```powershell
Copy-Item -Recurse tools\sigrok\decoders\growatt_rs485 "$env:ProgramData\libsigrokdecode\decoders\growatt_rs485"
.\tools\sigrok\install-pulseview-decoders.ps1
```

## Growatt CAN Decoder

The `decoders/growatt_can` directory is a PulseView/libsigrokdecode protocol
decoder for Growatt low-voltage Classic CAN BMS/inverter frames. It is
standalone on the CAN RX logic signal and internally reuses the built-in CAN
decoder, so add it directly from the decoder selector.

Typical Growatt CAN settings:

- CAN RX/H: the analyzer channel connected to the CAN transceiver `RXD` logic
  signal, to digitized `CANL`, or to digitized `CANH` depending on `Input mode`
- CANL: optional second analyzer channel, only used by `CANH/CANL digital diff`
- nominal bitrate: `500000`
- fast bitrate: unused for Classic CAN; leave the default or set it to
  `500000`
- sample point: start with `70%`; if annotations look unstable, try `75%` or
  `80%`
- input mode:
  - `rx/canl-direct`: default. Use with transceiver `RXD`, or with `CANL` if
    the logic-analyzer threshold turns recessive into `1` and dominant into `0`.
  - `canh-inverted`: use with `CANH` when the analyzer shows recessive as `0`
    and dominant as `1`.
  - `canh-canl-diff`: use CH0 as `CANH` and the optional `CANL` channel as
    CH1. This works with digitized bus wires, not with analog differential
    voltages; the analyzer thresholds still matter.

The decoder covers the Growatt CAN frame IDs used by this firmware and by the
field protocol notes: `0x311`, `0x312`, `0x313`, `0x314`, optional cell groups
`0x315..0x318`, `0x319`, `0x320`, `0x321`, `0x322`, and `0x323`.

On Windows, copy the whole `growatt_can` directory alongside the other custom
decoders, or run the installer script:

```powershell
Copy-Item -Recurse tools\sigrok\decoders\growatt_can "$env:ProgramData\libsigrokdecode\decoders\growatt_can"
.\tools\sigrok\install-pulseview-decoders.ps1
```

## Deye CAN Decoder

The `decoders/deye_can` directory is a PulseView/libsigrokdecode protocol
decoder for Deye-compatible low-voltage Classic CAN frames. Its current visible
PulseView name is `Deye CAN v2026.07.03a`. It is standalone on the CAN RX
logic signal and internally reuses the built-in CAN decoder, so add it directly
from the decoder selector.

Typical Deye CAN settings for the JK app profile `001 - Deye Low-voltage
hybrid inverter CAN`:

- CAN RX/H: the analyzer channel connected to the CAN transceiver `RXD` logic
  signal, to digitized `CANL`, or to digitized `CANH` depending on `Input mode`
- CANL: optional second analyzer channel, only used by `CANH/CANL digital diff`
- nominal bitrate: `500000`
- fast bitrate: unused for Classic CAN; leave the default or set it to
  `500000`
- sample point: start with `70%`; if annotations look unstable, try `75%` or
  `80%`
- input mode:
  - `rx/canl-direct`: default. Use with transceiver `RXD`, or with `CANL` if
    the logic-analyzer threshold turns recessive into `1` and dominant into `0`.
  - `canh-inverted`: use with `CANH` when the analyzer shows recessive as `0`
    and dominant as `1`.
  - `canh-canl-diff`: use CH0 as `CANH` and the optional `CANL` channel as
    CH1. This works with digitized bus wires, not with analog differential
    voltages; the analyzer thresholds still matter.

The decoder follows the ESP32 map in
`main/protocols/deye/deye_registers_map.h` and covers IDs `0x351`, `0x355`,
`0x356`, `0x359`, `0x35C`, `0x35E`, `0x370`, and `0x371`.

On Windows, copy the whole decoder directory alongside the other custom
decoders, or run the installer script:

```powershell
Copy-Item -Recurse tools\sigrok\decoders\deye_can "$env:ProgramData\libsigrokdecode\decoders\deye_can"
.\tools\sigrok\install-pulseview-decoders.ps1
```

## GoodWe CAN Decoder

The `decoders/goodwe_can` directory is a PulseView/libsigrokdecode protocol
decoder for GoodWe-compatible low-voltage Classic CAN frames. Its current
visible PulseView name is `GoodWe CAN v2026.07.03a`. It is standalone on the
CAN RX logic signal and internally reuses the built-in CAN decoder, so add it
directly from the decoder selector.

Typical GoodWe CAN settings for the tested JK app profile
`008 - GoodWe LV BMS Protocol`:

- CAN RX/H: the analyzer channel connected to the CAN transceiver `RXD` logic
  signal, to digitized `CANL`, or to digitized `CANH` depending on `Input mode`
- CANL: optional second analyzer channel, only used by `CANH/CANL digital diff`
- nominal bitrate: `500000`
- fast bitrate: unused for Classic CAN; leave the default or set it to
  `500000`
- sample point: start with `70%`; if annotations look unstable, try `75%` or
  `80%`
- input mode:
  - `rx/canl-direct`: default. Use with transceiver `RXD`, or with `CANL` if
    the logic-analyzer threshold turns recessive into `1` and dominant into `0`.
  - `canh-inverted`: use with `CANH` when the analyzer shows recessive as `0`
    and dominant as `1`.
  - `canh-canl-diff`: use CH0 as `CANH` and the optional `CANL` channel as
    CH1. This works with digitized bus wires, not with analog differential
    voltages; the analyzer thresholds still matter.

The decoder follows the ESP32 GoodWe map in
`main/protocols/goodwe/goodwe_registers_map.h` for classic IDs `0x453`,
`0x455`, `0x456`, `0x457`, and `0x458`. It also decodes the field-tested JK
GoodWe profile that emits a Pylon/Deye-like dialect on IDs `0x351`, `0x355`,
`0x356`, `0x359`, `0x35C`, `0x35E`, `0x370`, and `0x371`.

On Windows, copy the whole decoder directory alongside the other custom
decoders, or run the installer script:

```powershell
Copy-Item -Recurse tools\sigrok\decoders\goodwe_can "$env:ProgramData\libsigrokdecode\decoders\goodwe_can"
.\tools\sigrok\install-pulseview-decoders.ps1
```

## Victron CAN Decoder

The `decoders/victron_can` directory is a PulseView/libsigrokdecode protocol
decoder for Victron-compatible low-voltage Classic CAN frames. Its current
visible PulseView name is `Victron CAN v2026.07.03a`. It is standalone on the
CAN RX logic signal and internally reuses the built-in CAN decoder, so add it
directly from the decoder selector.

Typical Victron CAN settings for the tested JK app profile
`004 - Victron_CANbus_BMS_protocol_20170717`:

- CAN RX/H: the analyzer channel connected to the CAN transceiver `RXD` logic
  signal, to digitized `CANL`, or to digitized `CANH` depending on `Input mode`
- CANL: optional second analyzer channel, only used by `CANH/CANL digital diff`
- nominal bitrate: `500000`
- fast bitrate: unused for Classic CAN; leave the default or set it to
  `500000`
- sample point: start with `70%`; if annotations look unstable, try `75%` or
  `80%`
- input mode:
  - `rx/canl-direct`: default. Use with transceiver `RXD`, or with `CANL` if
    the logic-analyzer threshold turns recessive into `1` and dominant into `0`.
  - `canh-inverted`: use with `CANH` when the analyzer shows recessive as `0`
    and dominant as `1`.
  - `canh-canl-diff`: use CH0 as `CANH` and the optional `CANL` channel as
    CH1. This works with digitized bus wires, not with analog differential
    voltages; the analyzer thresholds still matter.

The decoder follows the ESP32 Victron map in
`main/protocols/victron/victron_registers_map.h` for IDs `0x351`, `0x355`,
and `0x356`. It also annotates the field-captured JK Victron profile frames
`0x35A`, `0x35E`, `0x35F`, `0x360`, `0x370..0x381`, keeping unknown fields as
raw hex and marking the Pylon-style `0x373` cell/temperature decode as
tentative.

On Windows, copy the whole decoder directory alongside the other custom
decoders, or run the installer script:

```powershell
Copy-Item -Recurse tools\sigrok\decoders\victron_can "$env:ProgramData\libsigrokdecode\decoders\victron_can"
.\tools\sigrok\install-pulseview-decoders.ps1
```

## JKBMS Modbus RS485 Decoder

The `decoders/jkbms_modbus` directory is a PulseView/libsigrokdecode protocol
decoder for JK BMS RS485 Modbus poller traffic. It matches the firmware route
used by the web setting `JKBMS_MODBUS (RS485 Poller, 115200)` and is separate
from the JK native binary `4E 57` decoder.

It stacks on top of the built-in `uart` decoder:

```text
logic -> uart -> jkbms_modbus
```

Typical JK Modbus RS485 settings:

- baud: `115200` for JK app profile `001 - JK BMS RS485 Modbus V1.0`
- baud: `9600` for JK app profile `013 - (9600) JK BMS RS485 Modbus V1.0`
- data bits: `8`
- parity: `none`
- stop bits: `1`
- bit order: `lsb-first`
- line inversion: depends on the probe point/transceiver output

The decoder recognizes Modbus RTU function `0x03`/`0x04` requests and
responses, validates CRC-16, tracks the active request range, and annotates the
JK runtime register map used by this firmware:

- cell voltage blocks starting at `0x1200`
- cell average/delta/index registers around `0x1244`
- MOS and battery temperatures around `0x128A`
- pack voltage/current/power, `SOC`, capacity, cycles, `SOH`, and alarm fields
  around `0x1290..0x12B8`

For the currently tested bridge route, decode the BMS side with
`JKBMS Modbus` at `115200 8N1` and the inverter side with `Growatt RS485` at
the Growatt UART baud configured for that bus.

On Windows, copy the whole decoder directory alongside the other custom
decoders, or run the installer script:

```powershell
Copy-Item -Recurse tools\sigrok\decoders\jkbms_modbus "$env:ProgramData\libsigrokdecode\decoders\jkbms_modbus"
.\tools\sigrok\install-pulseview-decoders.ps1
```

## JKBMS Native RS485 Decoder

The `decoders/jkbms_rs485_native` directory is a PulseView/libsigrokdecode
protocol decoder for JK native binary RS485 frames (`4E 57 ...`). It stacks on
top of the built-in `uart` decoder:

```text
logic -> uart -> jkbms_rs485_native
```

Typical JK native RS485 settings:

- baud: usually `9600` for the tested native profile
- data bits: `8`
- parity: `none`
- stop bits: `1`
- bit order: `lsb-first`
- line inversion: depends on the probe point/transceiver output

The decoder follows the local JK native field map from
`main/protocols/jkbms_rs485/jkbms_rs485_native.c` and the vendor
`bms-jk-rs485` reference. It is not a Modbus register decoder. The useful
native data IDs include:

- `0x79`: all-cell voltage list, packed as `cell_number + mV`
- `0x80`, `0x81`, `0x82`: MOS/box/battery temperatures
- `0x83`, `0x84`, `0x85`: pack voltage, pack current, and `SOC`
- `0x87`, `0x8A`, `0xAA`: cycles, string count, rated capacity
- `0x8B`, `0x8C`: alarm bits and charge/discharge/balance status
- `0x8E`, `0x8F`, `0x90`, `0x93`, `0x97`, `0x99`: pack/cell/current limits

For JK Modbus captures, use the register map under
`main/protocols/jkbms_modbus/jkbms_modbus_registers_map.h` instead. That map
uses runtime registers around `0x1200`, with cell voltages at
`0x1200..0x123E`, summary fields at `0x1244..`, and pack/temperature/capacity
fields around `0x128A..0x12B8`.

On Windows, copy the whole decoder directory alongside the other custom
decoders, or run the installer script:

```powershell
Copy-Item -Recurse tools\sigrok\decoders\jkbms_rs485_native "$env:ProgramData\libsigrokdecode\decoders\jkbms_rs485_native"
.\tools\sigrok\install-pulseview-decoders.ps1
```

## JKBMS CAN Decoder

The `decoders/jkbms_can` directory is a PulseView/libsigrokdecode protocol
decoder for JK app profile `000 - JK BMS CAN Protocol (250K) V2.0`. Its
current visible PulseView name is `JKBMS CAN v2026.07.03a`. It is
standalone on the CAN RX logic signal and internally reuses the built-in CAN
decoder, so add it directly from the decoder selector.

Typical JK native CAN settings:

- CAN RX/H: the analyzer channel connected to the CAN transceiver `RXD` logic
  signal, to digitized `CANL`, or to digitized `CANH` depending on `Input mode`
- CANL: optional second analyzer channel, only used by `CANH/CANL digital diff`
- nominal bitrate: `250000`
- fast bitrate: unused for Classic CAN; leave the default or set it to
  `250000`
- sample point: start with `70%`; if annotations look unstable, try `75%` or
  `80%`
- input mode:
  - `rx/canl-direct`: default. Use with transceiver `RXD`, or with `CANL` if
    the logic-analyzer threshold turns recessive into `1` and dominant into `0`.
  - `canh-inverted`: use with `CANH` when the analyzer shows recessive as `0`
    and dominant as `1`.
  - `canh-canl-diff`: use CH0 as `CANH` and the optional `CANL` channel as
    CH1. This works with digitized bus wires, not with analog differential
    voltages; the analyzer thresholds still matter.

The decoder covers the JK native CAN frames used by the firmware and vendor
reference: `0x02F4`, `0x04F4`, `0x05F4`, `0x07F4`, extended cell-voltage frames
`0x18E028F4..0x18E628F4`, extended temperatures `0x18F228F4`, and related
status/capacity frames. It also accepts the same command family with an `...F0`
suffix, because some references mask the low nibble as the JK node/BMS suffix.
Per-cell voltage decoding is capped at cells `1..25`, matching the ESP32
firmware cache behavior.

On Windows, copy the whole decoder directory alongside the other custom
decoders, or run the installer script:

```powershell
Copy-Item -Recurse tools\sigrok\decoders\jkbms_can "$env:ProgramData\libsigrokdecode\decoders\jkbms_can"
.\tools\sigrok\install-pulseview-decoders.ps1
```
