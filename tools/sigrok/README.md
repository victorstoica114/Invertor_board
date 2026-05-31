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

If PulseView was installed under `C:\Program Files\sigrok\PulseView`, add that
directory to the user `PATH` so `pulseview` can be launched from a fresh shell.
