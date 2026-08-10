# Local IoT dashboard

The dashboard polls Xiaomi `cuco.plug.v2eur` smart plugs and one Tuya air
conditioner on a separate asynchronous loop. A Wi-Fi device timeout therefore
does not block Bluetooth BMS polling. Power writes are serialized and verified
by an immediate read-back.

## Dependencies

Install the pinned dashboard dependencies in its virtual environment:

```sh
/home/pi/.venvs/inverter-bms-dashboard/bin/pip install \
  -r tools/bms_ble_requirements.txt
```

## Xiaomi inventory

Create `/home/pi/.config/iot-keys/xiaomi_plugs.json`:

```json
{
  "devices": [
    {
      "id": "boiler",
      "name": "Boiler",
      "ip": "192.168.1.207",
      "token": "<32 hexadecimal characters>",
      "model": "cuco.plug.v2eur",
      "reference_voltage_v": 230,
      "voltage_source_inverter_id": "inverter-anenji"
    }
  ]
}
```

This plug model reports power and energy, but it does not expose current or
voltage through its MIoT schema. Set `voltage_source_inverter_id` independently
for every plug; the deployed boiler uses `inverter-anenji` and the AC plug uses
`inverter-easun`. The dashboard uses that inverter's fresh `output_voltage_v`
and calculates plug current as `power / inverter voltage`. It never substitutes
grid-input voltage or another inverter for an explicitly mapped plug. A sample
older than five minutes is rejected, and `reference_voltage_v` is then used as
an explicitly marked fallback.

## Tuya air conditioner

Create `/home/pi/.config/iot-keys/tuya_ac.json` after obtaining the local key
from the Tuya IoT project linked to the Smart Life account:

```json
{
  "id": "air-conditioner",
  "name": "Air conditioner",
  "ip": "192.168.1.200",
  "device_id": "021002208c4f0049a4ce",
  "local_key": "<16-byte local key>",
  "product_id": "hw50w7qvxluhslkk",
  "version": 3.3,
  "temperature_scale": 10,
  "current_temperature_scale": 1,
  "target_temperature_scale": 10,
  "dps": {
    "power": "1",
    "target_temperature": "2",
    "current_temperature": "3",
    "mode": "4",
    "fan": "5",
    "advanced_flags": "123"
  },
  "controls": {
    "modes": ["auto", "cold", "hot", "wet", "wind"],
    "fan_speeds": ["strong", "high", "mid_high", "mid", "mid_low", "low", "mute", "auto"],
    "temperature": {
      "minimum_c": 16,
      "maximum_c": 30,
      "step_c": 0.5
    }
  }
}
```

The `dps` object overrides the defaults when a firmware variant exposes a
different datapoint map. `temperature_scale` remains the fallback for both
temperature fields; the per-field scales handle devices such as this TCL model,
which reports the target in tenths of a degree and the ambient value in whole
degrees. The first authenticated status response includes the non-secret raw
datapoints in `/api/iot`, which can be used to validate that map.

For TCL product `hw50w7qvxluhslkk`, the dashboard also maps the verified local
controls below. DP123 is always changed with a read-modify-write operation so
one switch cannot clear another switch sharing that hexadecimal bitfield.

| Function | Datapoint |
| --- | --- |
| Sleep profile | 105 |
| Capability flags and temperature unit | 110 |
| Vertical / horizontal sweep | 113 / 114 |
| Electricity management / GEN mode | 119 / 120 |
| ECO, self-cleaning, display, buzzer, health, anti-mildew, 8 C heat, soft airflow | 123 bitfield |
| Air quality | 125 |
| Vertical / horizontal fixed position | 126 / 127 |
| ECO temperature and filter warning | 130 / 131 |
| Hot/cold airflow | 132 |
| Faults, PM2.5, model and service diagnostics | 20, 101, 122, 128, 129, 133-136 |

The device does not expose an outdoor-unit temperature in its local schema.
DP136 remains explicitly labelled as unmapped service data instead of being
presented as a temperature. Unknown service datapoints are read-only.

The TCL app's timer is cloud scheduling rather than a local device datapoint.
The dashboard therefore provides its own persistent one-shot timers and weekly
power reservations. They are stored in private JSON at
`/var/lib/inverter-bms-dashboard/tuya_ac_schedules.json`; systemd creates that
state directory with mode `0700`. Schedules continue after a dashboard or
Raspberry Pi restart without storing TCL account credentials.

Both files are rejected unless they are private:

```sh
chmod 600 /home/pi/.config/iot-keys/xiaomi_plugs.json
chmod 600 /home/pi/.config/iot-keys/tuya_ac.json
```

Paths and the default 10-second polling interval can be changed with the
variables in `tools/bms_dashboard.env.example`. Smart-plug chart history is held
in memory and starts again when the dashboard service restarts.

## API

- `GET /api/iot` returns device state and smart-plug history without credentials.
- `POST /api/iot/plugs/{plug_id}/power` accepts `{"on": true}` or `{"on": false}`.
- `POST /api/iot/air-conditioner/power` accepts the same body.
- `POST /api/iot/air-conditioner/setting` accepts a validated `setting` and
  `value` for the mapped climate, airflow, comfort, energy, or advanced control.
- `POST /api/iot/air-conditioner/timers` accepts `minutes` and an `on` boolean.
- `POST /api/iot/air-conditioner/reservations` accepts local `time` (`HH:MM`),
  weekday numbers (`0` is Monday), and an `on` boolean.
- `DELETE /api/iot/air-conditioner/schedules/{id}` removes a timer or reservation.

The API trusts the LAN/VPN like the rest of the dashboard. Restrict port `8765`
at the network boundary.
