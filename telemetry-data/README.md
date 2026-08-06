# Inverter telemetry data

This directory contains the consistent SQLite snapshot generated on the
Raspberry Pi. The `telemetry-data` branch starts from `main`, while subsequent
database commits remain isolated on that branch.

- `telemetry.sqlite3`: complete SQLite snapshot created with the SQLite Backup API
- `metadata.json`: schema version, sample counts, latest source data, size and SHA-256

The snapshot publisher commits only when the logical database content changes.
Do not edit these generated files manually.
