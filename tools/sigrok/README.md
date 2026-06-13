# Sigrok / PulseView Tools

This directory contains optional analysis helpers for external logic-analyzer
workflows.

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
containing both PulseView's built-in decoders and this repository's Pylon
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

This repository's development machine also has a Desktop shortcut named
`PulseView Pylon` which runs `start-pulseview.ps1` and is useful when testing
decoder changes without reinstalling them globally.

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
