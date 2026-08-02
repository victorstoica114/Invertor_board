# Local telemetry database

`tools/telemetry_collector.py` reads every configured BMS and inverter every 30
seconds and stores successful snapshots in SQLite. Multiple BMS sources may
share the same physical ESP32 board.

## Board identities

| Board | Hostname | Wi-Fi MAC |
| --- | --- | --- |
| 1 | `inverter-board-1` | `58:8c:81:3a:d6:90` |
| 2 | `inverter-board-2` | `58:8c:81:5d:1b:94` |

Current sources:

| Source | Board | Endpoint | BMS |
| --- | --- | --- | --- |
| `inverter-board-1-bms1` | 1 | `/api/telemetry` | JK BMS |
| `inverter-board-1-bms2` | 1 | `/api/telemetry2` | Daly BMS |
| `inverter-board-2-bms1` | 2 | `/api/telemetry` | Seplos BMS |

The collector first uses an explicit board IP when configured. Without one, it
resolves the current IP by MAC/hostname from the hotspot dnsmasq lease file and
finally falls back to the last successful address cached in SQLite.

## Inverters

| Source | IP | Wi-Fi MAC | Protocol | Linked board |
| --- | --- | --- | --- | --- |
| `inverter-anenji` | `192.168.1.18` | `34:5f:45:48:cf:15` | Eybond-wrapped Modbus registers 201-234 | 2 |
| `inverter-easun` | `192.168.1.185` | `c4:d8:d5:1c:6a:06` | Eybond-wrapped Voltronic ASCII | 1 |

Both installed Eybond dongles require the standard reverse TCP callback port
`8899`. The collector therefore polls the inverters sequentially on that port.
Telemetry collection is read-only: it temporarily directs each dongle to the
Raspberry Pi, receives one reverse TCP connection, runs the queries, and closes
the connection. The dashboard's explicit configuration read/write operations
use the same port and a shared inter-process lock, so they cannot overlap a
collector cycle.

Anenji snapshots contain all raw live registers 201-234 plus normalized pack,
grid, inverter-internal, output, PV, load, temperature, SOC, operating-mode,
charging-average, and power-flow values. EASUN snapshots require `QPIGS` and
also collect the live `QMOD`, `QPIWS`, and `QPIGS2` queries when supported.
`QPIRI` is deliberately not queried by the collector because it contains
ratings and configuration. Complete decoded live payloads, live raw responses,
raw frames, and optional-command errors are retained in `payload_json`.

The installed EASUN reports protocol ID `PI30`. Its `QPIGS` response includes
measured AC output active/apparent power, but no measured AC-input current or
power. The collector records grid power as exactly zero while input voltage and
frequency are both zero. If AC input becomes available, grid power remains
unknown instead of being replaced with an unlabelled energy-balance estimate;
an external meter is required for an accurate non-zero grid-power measurement.

The local wire implementation follows the documented Eybond reverse-tunnel
behavior used by
[`smartess-local`](https://github.com/oleksandr-kuzmenko/smartess-local) and
the EASUN/Voltronic command handling demonstrated by
[`easunpy`](https://github.com/vgsolar2/easunpy). The collector itself has no
additional Python package dependency.

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

Show sample counts and the most recent result for every BMS and inverter:

```bash
python3 tools/telemetry_collector.py --status
```

Follow the background collector:

```bash
tail -f data/telemetry_collector.log
```

## Schema

Schema version 5 includes the original BMS source migration plus inverter
inventory and samples. Older databases are migrated in place and no BMS or
inverter sample rows are deleted. Version 4 allowed the two installed dongles
to share their required local callback port because polling is sequential.
Version 5 adds explicit columns for every currently decoded live inverter
field and removes historical EASUN `QPIRI`/rating content from `payload_json`.

`telemetry_samples` contains one row for every valid, non-stale HTTP response.
The main telemetry fields are stored in typed columns. `cells_v_json` preserves
the variable-length cell array, while `payload_json` preserves the complete
response so new firmware fields are not lost. Invalid, stale, absent, or
unreachable sources update status metadata but never create a sample.

`boards` contains physical ESP32 identity and connectivity status.
`bms_sources` contains the independent endpoint, display name, last successful
sample time, and latest error for each BMS.

`latest_telemetry` is a view containing the newest sample from each BMS source.

`inverters` contains identity, configured network/protocol information, linked
ESP32 board, last successful sample, and latest error. `inverter_samples`
contains 44 typed live fields and the filtered live `payload_json` for each
successful read. Configuration responses and inverter settings are not stored.
`latest_inverter_telemetry` contains the newest sample per inverter.

An absent or unreachable inverter updates `inverters.last_error` but never
creates a false/empty sample.

## Periodic Git snapshots

`tools/telemetry_snapshot_push.py` uses the SQLite Backup API to create a
consistent copy even while the collector is writing. The `telemetry-data`
branch starts from `main`, then keeps periodic database commits separate from
the firmware branch. It publishes only these generated files under the
`telemetry-data/` directory:

- `telemetry.sqlite3`
- `metadata.json`
- a short branch README and Git attributes

The metadata contains independent BMS and inverter sample counts. Either type
of new sample changes the logical snapshot key and causes the next scheduled
publication to include the updated database.

The dedicated worktree is kept outside the project at
`~/.local/share/inverter-telemetry-data`, so the automation never switches the
firmware checkout away from `main`. The publisher skips a new commit when the
logical database contents have not changed and refuses files above the safe
GitHub per-file limit. WAL files, logs, and local backups are never staged.

Run one publication manually:

```bash
python3 tools/telemetry_snapshot_push.py
```

If the data branch does not exist yet, the publisher creates it from `main`.

The user timer templates are:

- `tools/systemd/inverter-telemetry-push.service`
- `tools/systemd/inverter-telemetry-push.timer`

The timer runs 15 minutes after boot and then every 12 hours. Check it with:

```bash
systemctl --user status inverter-telemetry-push.timer
systemctl --user list-timers inverter-telemetry-push.timer
journalctl --user -u inverter-telemetry-push.service
```

Keeping repeated binary database snapshots in Git increases the total remote
repository size over time even though the data history is isolated from
`main`. If the SQLite file approaches 95 MiB, the publisher stops and the data
should be partitioned or moved to a storage system designed for large datasets.
