# Local BMS Bluetooth access

The Raspberry Pi Bluetooth controller can connect directly to all three BMS
devices currently used on the bench. `tools/bms_ble.py` provides telemetry and
protocol inspection. `tools/bms_config.py` provides guarded, one-parameter
configuration writes with an identity check and immediate full read-back.

## Detected devices

| Alias | BLE name / identity | MAC address | Verified access |
| --- | --- | --- | --- |
| `daly` | `DL-Dali Cristi`, FW `T00K_106042_21` | `D0:18:05:01:4B:F9` | 42 operating settings and switches |
| `seplos` | `SG16S200A-SP144B-C`, FW `15` | `C0:D6:3C:55:21:C6` | 164 SPA/SFA settings and inverter profile |
| `jk` | `JK_B1A8S20P`, HW `19H`, SW `19.13` | `C8:47:80:45:18:0E` | 65 protection, switch, trigger and communication settings |

## Installation

Keep the host dependencies separate from ESP-IDF. The path below matches the
systemd service template:

```sh
python3 -m venv /home/pi/.venvs/inverter-bms-dashboard
/home/pi/.venvs/inverter-bms-dashboard/bin/pip install -r tools/bms_ble_requirements.txt
mkdir -p /home/pi/.config
cp tools/bms_dashboard.env.example /home/pi/.config/inverter-bms-dashboard.env
chmod 600 /home/pi/.config/inverter-bms-dashboard.env
```

The dashboard trusts the local LAN. Telemetry, live configuration reads, and
guarded write actions do not require an additional dashboard password or token.
Do not expose port `8765` outside the trusted LAN/VPN.

## Read-only commands

```sh
/home/pi/.venvs/inverter-bms-dashboard/bin/python tools/bms_ble.py scan
/home/pi/.venvs/inverter-bms-dashboard/bin/python tools/bms_ble.py read --device all
/home/pi/.venvs/inverter-bms-dashboard/bin/python tools/bms_ble.py protocols --device all
```

An absent or busy device is reported as an error for that alias; the other
devices are still queried.

## JK protocol changes

First use `protocols --device jk`. The output contains the active selector and
only the protocol indices enabled by this particular firmware. A write is
rejected if the selected index is not enabled, the target MAC is not confirmed,
or the value cannot be read back after the command.

Example that keeps UART1 on JK Modbus (index `1`):

```sh
/home/pi/.venvs/inverter-bms-dashboard/bin/python tools/bms_ble.py set-jk-protocol \
  --interface uart1 --protocol 1 --confirm C8:47:80:45:18:0E
```

Protocol names may be supplied instead of indices, but the name must match the
list exactly. A protocol switch can immediately stop communication on the wired
port if the bridge firmware expects a different map or baud rate. Update the
board route only after the BLE read-back confirms the new selector.

## Seplos protocol changes

The live Seplos V3 selector map and two-phase PCT write were verified by BLE
read-back. A change requires the exact BMS serial and is rolled back if the
requested selector or baud rate cannot be read back:

```sh
/home/pi/.venvs/inverter-bms-dashboard/bin/python tools/bms_ble.py set-seplos-protocol \
  --profile growatt_485 --confirm SP144B-C2506260009
```

Available profiles on this firmware are:

| Tool profile | Selector | Seplos name | Board input | Live result |
| --- | ---: | --- | --- | --- |
| `srne_485` | 9 | PACE BMS Modbus / SRNE | `WOW_MODBUS` | pack summary; cell registers did not respond |
| `growatt_485` | 10 | Growatt SPF Modbus RS485 RTU | `RS485_GROWATT` | pack data and all 16 cell voltages |
| `pylon_485` | 11 | Pylon low-voltage RS485 | `RS485_PYLON` | compatible summary; no complete all-cell block on the tested firmware |

The tested setup uses `growatt_485` on Seplos and `RS485_GROWATT` on the bridge
input. Its bridge output stays `CAN_PYLON`, so Anenji continues receiving Pylon
CAN while the web telemetry gains the individual cells. The Growatt current
register is a signed centiamp value.

## Guarded BMS configuration

The **BMS Control** tab covers the verified live maps for the exact three devices:

- Daly D2 Modbus registers `0x0080`–`0x00A8` and balancer switch `0x00CF`;
- Seplos V3 SPA registers `0x1301`–`0x1367`, meaningful SFA coils in
  `0x1400`–`0x144F`, and the verified inverter-protocol selector;
- JK v19 numeric protection/capacity settings, MOSFET and feature switches,
  dry-contact triggers, UART/CAN selectors, and inverter-request settings.

The interface reads the selected BMS automatically every 30 seconds while the
tab is visible. Every Apply action shows the live value, asks for confirmation,
writes one mapped parameter, then reads the complete map again. A mismatched
identity, an out-of-range value, an unsupported selector, or a failed read-back
rejects the operation. A JSON backup can be downloaded before making changes.

Raw writes, unknown/reserved registers, factory calibration commands without a
verified read-back field, and one-shot service/reset commands are deliberately
not exposed. The Daly firmware has no verified inverter-protocol selector, but
its operating thresholds and switches are writable through the mapped D2
registers.

## Web dashboard

`tools/bms_dashboard.py` exposes the three Bluetooth BMS devices and the Wi-Fi
inverters through a responsive LAN web interface. A single background loop owns
the Bluetooth adapter; browsers only read the cached state and therefore do not
create competing BLE connections. Inverter telemetry is read from the local
SQLite database populated by `inverter-telemetry.service`, so the dashboard
does not open a second connection during normal inverter monitoring. The
separate **BMS Control** and **Inverter Control** tabs open guarded, short-lived
configuration connections. ESP32 wired acquisition remains the operational BMS
telemetry path; configuration writes do not alter that collector.

```sh
/home/pi/.venvs/inverter-bms-dashboard/bin/python tools/bms_dashboard.py --host 0.0.0.0 --port 8765
```

Monitoring, live inverter-configuration reads, and writes do not require an
additional password. The browser supplies the already-read device identity,
shows an explicit confirmation dialog, restricts values to the verified map,
and requires a successful read-back from the device.

The inverter control tab reads the selected inverter immediately when opened
and refreshes it every 30 seconds while the tab remains visible. Polling pauses
when the tab/page is hidden and while a setting is being edited. The current
configuration can be downloaded as a JSON backup. It writes only one known setting at a
time, validates allowed values and dependent battery limits, requires the exact
inverter serial, and re-reads the entire configuration afterward. Requests are
serialized with the 30-second collector through a shared file lock. Registers
whose live values do not match the applicable protocol map are shown read-only;
arbitrary register or raw-command writes are not exposed.

The inverter-control endpoints are:

- `GET /api/inverters/{inverter_id}/configuration`
- `POST /api/inverters/{inverter_id}/setting`

The POST body contains `setting`, `value`, and the identity automatically read
from the selected device. Write attempts and their before/after read-back values
are recorded in the system journal.

The equivalent BMS-control endpoints are:

- `GET /api/bms/{daly|seplos|jk}/configuration`
- `POST /api/bms/{daly|seplos|jk}/setting`

The BMS POST body also contains `setting`, `value`, and the identity confirmation
automatically obtained by the preceding configuration read. BMS BLE operations
share one lock with the existing Bluetooth loop so two clients cannot write or
poll the adapter concurrently.

The inverter database and freshness threshold can be overridden with
`BMS_DASHBOARD_TELEMETRY_DATABASE` and
`BMS_DASHBOARD_INVERTER_STALE_SECONDS`. The inverter-only JSON endpoint is
`GET /api/inverters`; the same snapshot is also included in `GET /api/status`.

The service template is `tools/systemd/inverter-bms-dashboard.service`. It uses
the dedicated virtual environment `/home/pi/.venvs/inverter-bms-dashboard` and
reads its configuration from `/home/pi/.config/inverter-bms-dashboard.env`.
It is installed as a system service running with the unprivileged `pi` account
and is ordered after the Bluetooth and network services at boot.
The service grants write access only to `/run/user/1000`, which is required for
the shared inverter-network lock while `ProtectHome=read-only` remains enabled.

Install or update the service after completing the steps above:

```sh
sudo cp tools/systemd/inverter-bms-dashboard.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now inverter-bms-dashboard.service
```
