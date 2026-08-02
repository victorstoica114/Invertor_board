# Local telemetry database

`tools/telemetry_collector.py` reads every configured BMS telemetry endpoint
every 30 seconds and stores valid, non-stale responses in SQLite. Multiple BMS
sources may share the same physical ESP32 board.

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

Show sample counts and the most recent result for each BMS source:

```bash
python3 tools/telemetry_collector.py --status
```

Follow the background collector:

```bash
tail -f data/telemetry_collector.log
```

## Schema

Schema version 2 adds `bms_sources` and a `source_id` on every sample. A version
1 database is migrated in place; existing primary samples are assigned to the
matching `*-bms1` source and are not deleted.

`telemetry_samples` contains one row for every valid, non-stale HTTP response.
The main telemetry fields are stored in typed columns. `cells_v_json` preserves
the variable-length cell array, while `payload_json` preserves the complete
response so new firmware fields are not lost. Invalid, stale, absent, or
unreachable sources update status metadata but never create a sample.

`boards` contains physical ESP32 identity and connectivity status.
`bms_sources` contains the independent endpoint, display name, last successful
sample time, and latest error for each BMS.

`latest_telemetry` is a view containing the newest sample from each BMS source.

## Periodic Git snapshots

`tools/telemetry_snapshot_push.py` uses the SQLite Backup API to create a
consistent copy even while the collector is writing. The `telemetry-data`
branch starts from `main`, then keeps periodic database commits separate from
the firmware branch. It publishes only these generated files under the
`telemetry-data/` directory:

- `telemetry.sqlite3`
- `metadata.json`
- a short branch README and Git attributes

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
