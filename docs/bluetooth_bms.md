# Local BMS Bluetooth access

The Raspberry Pi Bluetooth controller can connect directly to all three BMS
devices currently used on the bench. `tools/bms_ble.py` provides telemetry and
protocol inspection, plus guarded protocol writes for the JK and Seplos.

## Detected devices

| Alias | BLE name / identity | MAC address | Verified access |
| --- | --- | --- | --- |
| `daly` | `DL-Dali Cristi`, HW `FX03_R301_1.2H` | `D0:18:05:01:4B:F9` | identity and full telemetry |
| `seplos` | `SG16S200A-SP144B-C`, FW `15` | `C0:D6:3C:55:21:C6` | telemetry, identity and verified inverter-protocol read/write |
| `jk` | `JK_B1A8S20P`, HW `19H`, SW `19.13` | `C8:47:80:45:18:0E` | telemetry, identity and UART/CAN protocol read/write |

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

The example configuration leaves protocol controls disabled. To enable JK and
Seplos writes, generate a private token, place it after
`BMS_DASHBOARD_CONTROL_TOKEN=` in the local environment file, and do not commit
that populated file.

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

## Daly write status

The Daly Bluetooth module uses its Modbus telemetry protocol and responds to all
read requests. No verified inverter-protocol selector is exposed for this Daly
firmware, so protocol-changing writes are not offered.

## Web dashboard

`tools/bms_dashboard.py` exposes the three devices through a responsive LAN web
interface. A single background loop owns the Bluetooth adapter; browsers only
read the cached state and therefore do not create competing BLE connections.

```sh
/home/pi/.venvs/inverter-bms-dashboard/bin/python tools/bms_dashboard.py --host 0.0.0.0 --port 8765
```

Monitoring endpoints are read-only. JK and Seplos protocol writes require the
value of `BMS_DASHBOARD_CONTROL_TOKEN` in the `X-Control-Token` request header,
an exact device confirmation typed in the browser, a permitted protocol, and a
successful read-back from the BMS.

The service template is `tools/systemd/inverter-bms-dashboard.service`. It uses
the dedicated virtual environment `/home/pi/.venvs/inverter-bms-dashboard` and
reads its configuration from `/home/pi/.config/inverter-bms-dashboard.env`.
It is installed as a system service running with the unprivileged `pi` account
and is ordered after the Bluetooth and network services at boot.

Install or update the service after completing the steps above:

```sh
sudo cp tools/systemd/inverter-bms-dashboard.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now inverter-bms-dashboard.service
```
