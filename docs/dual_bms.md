# Dual-BMS mode

Dual-BMS mode runs a second RS485 BMS poller alongside the normal bridge route.
The primary BMS remains the only source for the shared battery model and for all
inverter-facing messages. BMS 2 is telemetry-only, so data from two battery packs
cannot be mixed accidentally.

## Configuration

In the web interface, open **Settings**, enable **Dual BMS**, then select the BMS 2
protocol and RS485 port. After applying the settings, the interface shows
**Telemetry BMS 1** and **Telemetry BMS 2** tabs.

The first implementation supports these secondary protocols:

- `DALY_RS485` at 9600 baud
- `JKBMS_MODBUS` at 9600 baud
- `JKBMS_MODBUS_115200` at 115200 baud

BMS 2 must use a free RS485 port. It cannot share a port with an RS485 primary BMS
or RS485 inverter. The two BMS slots also cannot use the same task family (for
example, two JKBMS Modbus pollers) in this version.

## API

- `GET /api/telemetry` returns BMS 1 telemetry.
- `GET /api/telemetry2` returns BMS 2 telemetry.
- `GET /api/settings` includes `dual_bms`, `bms2_protocol_id`, and `bms2_port`.
- `POST /api/settings` accepts `dual_bms`, `bms2_protocol`, and `bms2_port`.

When dual-BMS mode is disabled, `/api/telemetry2` returns an invalid/empty
telemetry snapshot and its UI tab is hidden.
