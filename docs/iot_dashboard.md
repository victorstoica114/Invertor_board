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
      "reference_voltage_v": 230
    }
  ]
}
```

This plug model reports power and energy, but it does not expose current or
voltage through its MIoT schema. The dashboard takes the freshest valid grid
AC-output voltage reported by either inverter and calculates plug current as
`power / inverter voltage`. Grid-input voltage is used only when output voltage
is unavailable. An inverter sample older than five minutes is rejected;
`reference_voltage_v` is then used as an explicitly marked fallback.

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
    "mode": "4",
    "fan": "5",
    "temperature_unit": "19",
    "current_temperature": "3",
    "target_temperature": "2"
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
  `value` for mode, fan speed, or target temperature.

The API trusts the LAN/VPN like the rest of the dashboard. Restrict port `8765`
at the network boundary.
