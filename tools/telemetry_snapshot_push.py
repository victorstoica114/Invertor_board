#!/usr/bin/env python3
"""Publish a consistent SQLite telemetry snapshot to a dedicated Git branch."""

from __future__ import annotations

import argparse
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
from typing import Any, Final


PROJECT_ROOT: Final[Path] = Path(__file__).resolve().parents[1]
DEFAULT_DATABASE: Final[Path] = PROJECT_ROOT / "data" / "telemetry.sqlite3"
DEFAULT_WORKTREE: Final[Path] = (
    Path.home() / ".local" / "share" / "inverter-telemetry-data"
)
DEFAULT_LOCK: Final[Path] = (
    Path.home() / ".local" / "state" / "inverter-telemetry-push.lock"
)
DEFAULT_BRANCH: Final[str] = "telemetry-data"
DEFAULT_BASE_BRANCH: Final[str] = "main"
DEFAULT_REMOTE: Final[str] = "origin"
DATA_DIRECTORY: Final[str] = "telemetry-data"
SNAPSHOT_NAME: Final[str] = "telemetry.sqlite3"
METADATA_NAME: Final[str] = "metadata.json"
MAX_GITHUB_FILE_BYTES: Final[int] = 95 * 1024 * 1024

README_CONTENT: Final[str] = """# Inverter telemetry data

This directory contains the consistent SQLite snapshot generated on the
Raspberry Pi. The `telemetry-data` branch starts from `main`, while subsequent
database commits remain isolated on that branch.

- `telemetry.sqlite3`: complete SQLite snapshot created with the SQLite Backup API
- `metadata.json`: schema version, sample counts, latest source data, size and SHA-256

The snapshot publisher commits only when the logical database content changes.
Do not edit these generated files manually.
"""

GITATTRIBUTES_CONTENT: Final[str] = "*.sqlite3 binary\n"
GITIGNORE_CONTENT: Final[str] = "*.next\n*.tmp\n*-wal\n*-shm\n"


class PublishError(RuntimeError):
    """Raised when a safe snapshot or Git publication cannot be completed."""


def utc_now() -> str:
    return (
        dt.datetime.now(dt.timezone.utc)
        .isoformat(timespec="seconds")
        .replace("+00:00", "Z")
    )


def run_command(
    arguments: list[str], cwd: Path, *, check: bool = True
) -> subprocess.CompletedProcess[str]:
    environment = os.environ.copy()
    environment["GIT_TERMINAL_PROMPT"] = "0"
    result = subprocess.run(
        arguments,
        cwd=cwd,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if check and result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip() or "unknown error"
        raise PublishError(f"{' '.join(arguments)} failed: {detail}")
    return result


def git(repository: Path, *arguments: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    return run_command(["git", *arguments], repository, check=check)


def ensure_worktree(
    repository: Path,
    worktree: Path,
    branch: str,
    base_branch: str,
    remote: str,
) -> None:
    if (worktree / ".git").exists():
        current_branch = git(worktree, "symbolic-ref", "--short", "HEAD").stdout.strip()
        if current_branch != branch:
            raise PublishError(
                f"snapshot worktree uses branch {current_branch!r}, expected {branch!r}"
            )
        return
    if worktree.exists() and any(worktree.iterdir()):
        raise PublishError(f"snapshot worktree path is not empty: {worktree}")

    worktree.parent.mkdir(parents=True, exist_ok=True)
    remote_branch = git(
        repository,
        "ls-remote",
        "--exit-code",
        "--heads",
        remote,
        f"refs/heads/{branch}",
        check=False,
    )
    if remote_branch.returncode not in (0, 2):
        detail = remote_branch.stderr.strip() or remote_branch.stdout.strip()
        raise PublishError(f"cannot inspect remote branch {remote}/{branch}: {detail}")
    local_branch = git(
        repository,
        "show-ref",
        "--verify",
        "--quiet",
        f"refs/heads/{branch}",
        check=False,
    )
    if local_branch.returncode == 0:
        git(
            repository,
            "worktree",
            "add",
            "--lock",
            "--reason",
            "automated telemetry snapshots",
            str(worktree),
            branch,
        )
    elif remote_branch.returncode == 0:
        git(repository, "fetch", remote, f"{branch}:refs/remotes/{remote}/{branch}")
        git(
            repository,
            "worktree",
            "add",
            "--lock",
            "--reason",
            "automated telemetry snapshots",
            "-b",
            branch,
            "--track",
            str(worktree),
            f"{remote}/{branch}",
        )
    else:
        git(
            repository,
            "worktree",
            "add",
            "-b",
            branch,
            "--lock",
            "--reason",
            "automated telemetry snapshots",
            str(worktree),
            base_branch,
        )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def database_summary(connection: sqlite3.Connection) -> dict[str, Any]:
    connection.row_factory = sqlite3.Row
    schema_version = int(connection.execute("PRAGMA user_version").fetchone()[0])
    sample_count = int(
        connection.execute("SELECT COUNT(*) FROM telemetry_samples").fetchone()[0]
    )
    latest_sample = connection.execute(
        "SELECT MAX(sampled_at_utc) FROM telemetry_samples"
    ).fetchone()[0]
    sources = [
        dict(row)
        for row in connection.execute(
            """
            SELECT
                bms_sources.source_id,
                bms_sources.name,
                bms_sources.board_id,
                bms_sources.endpoint,
                COUNT(telemetry_samples.id) AS sample_count,
                MAX(telemetry_samples.sampled_at_utc) AS latest_sample_utc,
                latest_telemetry.protocol AS latest_protocol,
                latest_telemetry.cell_count AS latest_cell_count
            FROM bms_sources
            LEFT JOIN telemetry_samples
                ON telemetry_samples.source_id = bms_sources.source_id
            LEFT JOIN latest_telemetry
                ON latest_telemetry.source_id = bms_sources.source_id
            GROUP BY bms_sources.source_id
            ORDER BY bms_sources.source_id
            """
        )
    ]
    return {
        "schema_version": schema_version,
        "sample_count": sample_count,
        "latest_sample_utc": latest_sample,
        "sources": sources,
    }


def create_snapshot(source: Path, destination: Path) -> dict[str, Any]:
    if not source.is_file():
        raise PublishError(f"telemetry database does not exist: {source}")
    if destination.exists():
        destination.unlink()
    source_uri = f"{source.resolve().as_uri()}?mode=ro"
    try:
        with sqlite3.connect(source_uri, uri=True, timeout=10) as source_connection:
            with sqlite3.connect(destination, timeout=10) as snapshot_connection:
                source_connection.backup(snapshot_connection)
                snapshot_connection.commit()
                integrity = snapshot_connection.execute(
                    "PRAGMA integrity_check"
                ).fetchone()[0]
                if integrity != "ok":
                    raise PublishError(f"SQLite snapshot integrity check failed: {integrity}")
                summary = database_summary(snapshot_connection)
    except sqlite3.Error as exc:
        raise PublishError(f"SQLite snapshot failed: {exc}") from exc

    size_bytes = destination.stat().st_size
    if size_bytes > MAX_GITHUB_FILE_BYTES:
        destination.unlink(missing_ok=True)
        raise PublishError(
            f"snapshot is {size_bytes} bytes, above the safe GitHub file limit; "
            "archive or partition the telemetry data before publishing"
        )
    summary.update(
        {
            "format_version": 1,
            "generated_at_utc": utc_now(),
            "database_file": f"{DATA_DIRECTORY}/{SNAPSHOT_NAME}",
            "size_bytes": size_bytes,
            "sha256": sha256_file(destination),
        }
    )
    return summary


def logical_snapshot_key(metadata: dict[str, Any]) -> tuple[Any, ...]:
    sources = tuple(
        (
            source.get("source_id"),
            source.get("sample_count"),
            source.get("latest_sample_utc"),
            source.get("latest_protocol"),
            source.get("latest_cell_count"),
        )
        for source in metadata.get("sources", [])
    )
    return (
        metadata.get("schema_version"),
        metadata.get("sample_count"),
        metadata.get("latest_sample_utc"),
        sources,
    )


def read_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def write_text_if_changed(path: Path, content: str) -> None:
    if path.is_file() and path.read_text(encoding="utf-8") == content:
        return
    temporary = path.with_name(f"{path.name}.tmp")
    temporary.write_text(content, encoding="utf-8")
    os.replace(temporary, path)


def synchronize_remote(worktree: Path, branch: str, remote: str) -> None:
    remote_branch = git(
        worktree,
        "ls-remote",
        "--exit-code",
        "--heads",
        remote,
        f"refs/heads/{branch}",
        check=False,
    )
    if remote_branch.returncode not in (0, 2):
        detail = remote_branch.stderr.strip() or remote_branch.stdout.strip()
        raise PublishError(f"cannot inspect remote branch {remote}/{branch}: {detail}")
    if remote_branch.returncode != 0:
        return
    git(worktree, "fetch", remote, branch)
    local_head = git(worktree, "rev-parse", "--verify", "HEAD", check=False)
    if local_head.returncode == 0:
        git(worktree, "merge", "--ff-only", "FETCH_HEAD")


def publish_snapshot(
    repository: Path,
    source_database: Path,
    worktree: Path,
    branch: str,
    base_branch: str,
    remote: str,
) -> dict[str, Any]:
    ensure_worktree(repository, worktree, branch, base_branch, remote)
    synchronize_remote(worktree, branch, remote)

    data_directory = worktree / DATA_DIRECTORY
    for temporary_name in (f"{SNAPSHOT_NAME}.next", f"{METADATA_NAME}.tmp"):
        (data_directory / temporary_name).unlink(missing_ok=True)
    dirty = git(worktree, "status", "--porcelain").stdout.strip()
    if dirty:
        raise PublishError(f"snapshot worktree has unexpected changes:\n{dirty}")

    data_directory.mkdir(parents=True, exist_ok=True)
    write_text_if_changed(data_directory / "README.md", README_CONTENT)
    write_text_if_changed(data_directory / ".gitattributes", GITATTRIBUTES_CONTENT)
    write_text_if_changed(data_directory / ".gitignore", GITIGNORE_CONTENT)

    next_snapshot = data_directory / f"{SNAPSHOT_NAME}.next"
    metadata = create_snapshot(source_database, next_snapshot)
    previous_metadata = read_json(data_directory / METADATA_NAME)
    database_changed = (
        previous_metadata is None
        or logical_snapshot_key(previous_metadata) != logical_snapshot_key(metadata)
        or not (data_directory / SNAPSHOT_NAME).is_file()
    )
    if database_changed:
        os.replace(next_snapshot, data_directory / SNAPSHOT_NAME)
        write_text_if_changed(
            data_directory / METADATA_NAME,
            json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        )
    else:
        next_snapshot.unlink(missing_ok=True)

    git(
        worktree,
        "add",
        "--",
        DATA_DIRECTORY,
    )
    staged = git(worktree, "diff", "--cached", "--quiet", check=False)
    if staged.returncode not in (0, 1):
        detail = staged.stderr.strip() or staged.stdout.strip()
        raise PublishError(f"cannot inspect staged snapshot changes: {detail}")
    committed = staged.returncode != 0
    if committed:
        timestamp = metadata["generated_at_utc"].replace("T", " ").replace("Z", " UTC")
        git(
            worktree,
            "-c",
            "user.name=Inverter Telemetry Bot",
            "-c",
            "user.email=telemetry-bot@users.noreply.github.com",
            "commit",
            "-m",
            f"Telemetry snapshot {timestamp}",
        )

    local_head = git(worktree, "rev-parse", "--verify", "HEAD", check=False)
    if local_head.returncode != 0:
        raise PublishError("snapshot branch has no commit to push")
    git(worktree, "push", "--set-upstream", remote, f"HEAD:{branch}")
    commit = git(worktree, "rev-parse", "HEAD").stdout.strip()
    return {
        "branch": branch,
        "commit": commit,
        "committed": committed,
        "database_changed": database_changed,
        "sample_count": metadata["sample_count"],
        "latest_sample_utc": metadata["latest_sample_utc"],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository", type=Path, default=PROJECT_ROOT)
    parser.add_argument("--database", type=Path, default=DEFAULT_DATABASE)
    parser.add_argument("--worktree", type=Path, default=DEFAULT_WORKTREE)
    parser.add_argument("--lock-file", type=Path, default=DEFAULT_LOCK)
    parser.add_argument("--branch", default=DEFAULT_BRANCH)
    parser.add_argument("--base-branch", default=DEFAULT_BASE_BRANCH)
    parser.add_argument("--remote", default=DEFAULT_REMOTE)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repository = args.repository.expanduser().resolve()
    database = args.database.expanduser().resolve()
    worktree = args.worktree.expanduser().resolve()
    lock_file = args.lock_file.expanduser().resolve()
    if not (repository / ".git").exists():
        print(f"Repository error: not a Git checkout: {repository}", file=sys.stderr)
        return 2

    lock_file.parent.mkdir(parents=True, exist_ok=True)
    try:
        with lock_file.open("w", encoding="utf-8") as lock_handle:
            try:
                fcntl.flock(lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                print("Another telemetry snapshot publication is already running")
                return 0
            result = publish_snapshot(
                repository,
                database,
                worktree,
                args.branch,
                args.base_branch,
                args.remote,
            )
    except (OSError, PublishError) as exc:
        print(f"Telemetry snapshot error: {exc}", file=sys.stderr)
        return 1

    action = "committed and pushed" if result["committed"] else "already current; push verified"
    print(
        f"Telemetry snapshot {action}: branch={result['branch']} "
        f"commit={result['commit']} samples={result['sample_count']} "
        f"latest={result['latest_sample_utc'] or '-'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
