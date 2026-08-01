# Local telemetry database

`tools/telemetry_collector.py` reads `/api/telemetry` from both inverter
boards every 30 seconds and stores successful responses in SQLite.

## Board identities

| Board | Hostname | Wi-Fi MAC |
| --- | --- | --- |
| 1 | `inverter-board-1` | `58:8c:81:3a:d6:90` |
| 2 | `inverter-board-2` | `58:8c:81:5d:1b:94` |

The collector reads the hotspot's dnsmasq lease file and resolves the current
IP address by MAC. IP addresses may change without requiring configuration
changes.

## Files

- configuration: `tools/telemetry_boards.json`
- SQLite database: `data/telemetry.sqlite3`
- systemd user service: `inverter-telemetry.service`
- collector log: `data/telemetry_collector.log`

The `data/` directory is ignored by Git.

## Useful commands

Run one collection cycle:

```bash
python3 tools/telemetry_collector.py --once
```

Show sample counts and the most recent result for each board:

```bash
python3 tools/telemetry_collector.py --status
```

Follow the background collector:

```bash
tail -f data/telemetry_collector.log
```

## Schema

`telemetry_samples` contains one row for every successful HTTP response. The
main telemetry fields are stored in typed columns. `cells_v_json` preserves
the variable-length cell array, while `payload_json` preserves the complete
response so new firmware fields are not lost.

`boards` contains device identity, last IP, last successful sample time, and
the latest connection error. A missing or unreachable board updates only this
status metadata; it does not create a telemetry row.

`latest_telemetry` is a view containing the newest sample from each board.
