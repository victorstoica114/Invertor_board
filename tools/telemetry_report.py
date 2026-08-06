#!/usr/bin/env python3
"""Generate a reusable PDF report from the local telemetry SQLite database."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import sqlite3
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence
from zoneinfo import ZoneInfo

import matplotlib

matplotlib.use("Agg")

import matplotlib.dates as mdates  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATABASE = PROJECT_ROOT / "data" / "telemetry.sqlite3"
DEFAULT_REPORT_DIR = PROJECT_ROOT / "data" / "reports"
DEFAULT_TIMEZONE = "Europe/Bucharest"
PAGE_SIZE = (11.69, 8.27)  # A4 landscape, inches
PLOT_BACKGROUND = "#ffffff"
PAGE_BACKGROUND = "#f4f7f9"
TEXT_COLOR = "#1b2a35"
MUTED_COLOR = "#637783"
GRID_COLOR = "#dbe5e8"
ACCENT = "#18866f"
SERIES_COLORS = (
    "#18866f",
    "#386cb0",
    "#e07a24",
    "#8e5ab5",
    "#c94c4c",
    "#6a8f3d",
)

BMS_COLUMNS = (
    "sampled_at_unix_ms",
    "sampled_at_utc",
    "pack_voltage_v",
    "current_a",
    "pack_power_w",
    "soc_pct",
    "soh_pct",
    "temp_mos_c",
    "temp_t1_c",
    "temp_t2_c",
    "cell_min_v",
    "cell_max_v",
    "cell_diff_v",
    "cell_count",
    "cells_v_json",
)

INVERTER_COLUMNS = (
    "sampled_at_unix_ms",
    "sampled_at_utc",
    "working_mode",
    "grid_voltage_v",
    "grid_frequency_hz",
    "grid_power_w",
    "inverter_power_w",
    "output_voltage_v",
    "output_current_a",
    "output_frequency_hz",
    "output_power_w",
    "output_apparent_power_va",
    "load_pct",
    "battery_voltage_v",
    "battery_current_a",
    "battery_charge_current_a",
    "battery_discharge_current_a",
    "battery_power_w",
    "battery_soc_pct",
    "pv_voltage_v",
    "pv_current_a",
    "pv_power_w",
    "pv_charging_power_w",
    "pv2_power_w",
    "inverter_temperature_c",
    "dcdc_temperature_c",
    "pv_temperature_c",
)


@dataclass(frozen=True)
class TelemetrySeries:
    source_id: str
    name: str
    rows: tuple[sqlite3.Row, ...]

    @property
    def sample_count(self) -> int:
        return len(self.rows)

    @property
    def first_ms(self) -> int:
        return int(self.rows[0]["sampled_at_unix_ms"])

    @property
    def last_ms(self) -> int:
        return int(self.rows[-1]["sampled_at_unix_ms"])

    def values(self, column: str) -> np.ndarray:
        return np.asarray(
            [math.nan if row[column] is None else float(row[column]) for row in self.rows],
            dtype=float,
        )


def parse_timestamp(value: str) -> dt.datetime:
    """Parse an ISO-8601 timestamp and normalize it to UTC."""
    normalized = value.strip()
    if normalized.endswith("Z"):
        normalized = normalized[:-1] + "+00:00"
    parsed = dt.datetime.fromisoformat(normalized)
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=dt.timezone.utc)
    return parsed.astimezone(dt.timezone.utc)


def timestamp_ms(value: dt.datetime) -> int:
    return int(value.timestamp() * 1000)


def minmax_indices(values: Sequence[float], max_points: int) -> np.ndarray:
    """Return ordered indices that retain bucket minima and maxima."""
    size = len(values)
    if size <= max_points or max_points < 4:
        return np.arange(size, dtype=int)

    array = np.asarray(values, dtype=float)
    bucket_count = max(1, (max_points - 2) // 2)
    edges = np.linspace(1, size - 1, bucket_count + 1, dtype=int)
    selected = {0, size - 1}
    for start, stop in zip(edges[:-1], edges[1:]):
        if stop <= start:
            continue
        bucket = array[start:stop]
        finite = np.flatnonzero(np.isfinite(bucket))
        if finite.size == 0:
            selected.add(start)
            continue
        finite_values = bucket[finite]
        selected.add(start + int(finite[np.argmin(finite_values)]))
        selected.add(start + int(finite[np.argmax(finite_values)]))
    return np.asarray(sorted(selected), dtype=int)


def _time_values(
    series: TelemetrySeries,
    column: str,
    timezone: ZoneInfo,
    max_points: int,
) -> tuple[list[dt.datetime], np.ndarray]:
    values = series.values(column)
    indices = minmax_indices(values, max_points)
    times_ms = np.asarray(
        [int(series.rows[index]["sampled_at_unix_ms"]) for index in indices],
        dtype=np.int64,
    )
    selected_values = values[indices].copy()

    if times_ms.size > 2:
        positive_steps = np.diff(times_ms)
        positive_steps = positive_steps[positive_steps > 0]
        typical_step = float(np.median(positive_steps)) if positive_steps.size else 30_000.0
        gap_limit = max(5 * typical_step, 5 * 60_000.0)
        selected_values[1:][np.diff(times_ms) > gap_limit] = math.nan

    times = [dt.datetime.fromtimestamp(value / 1000, timezone) for value in times_ms]
    return times, selected_values


def _configure_axis(axis: plt.Axes, ylabel: str | None = None) -> None:
    axis.set_facecolor(PLOT_BACKGROUND)
    axis.grid(True, color=GRID_COLOR, linewidth=0.6, alpha=0.8)
    axis.tick_params(colors=MUTED_COLOR, labelsize=8)
    for spine in axis.spines.values():
        spine.set_color(GRID_COLOR)
    if ylabel:
        axis.set_ylabel(ylabel, color=MUTED_COLOR, fontsize=8)
    locator = mdates.AutoDateLocator(minticks=3, maxticks=7)
    axis.xaxis.set_major_locator(locator)
    axis.xaxis.set_major_formatter(mdates.ConciseDateFormatter(locator))


def _new_figure(title: str, subtitle: str | None = None) -> tuple[plt.Figure, float]:
    figure = plt.figure(figsize=PAGE_SIZE, facecolor=PAGE_BACKGROUND)
    figure.text(0.04, 0.955, title, fontsize=19, fontweight="bold", color=TEXT_COLOR, va="top")
    top = 0.91
    if subtitle:
        figure.text(0.04, 0.915, subtitle, fontsize=9, color=MUTED_COLOR, va="top")
        top = 0.88
    return figure, top


def _finalize_figure(figure: plt.Figure, pdf: PdfPages) -> None:
    figure.text(
        0.965,
        0.022,
        "Inverter Board telemetry",
        ha="right",
        fontsize=7,
        color=MUTED_COLOR,
    )
    pdf.savefig(figure, facecolor=figure.get_facecolor())
    plt.close(figure)


def _plot_column(
    axis: plt.Axes,
    series: TelemetrySeries,
    column: str,
    timezone: ZoneInfo,
    max_points: int,
    *,
    color: str = ACCENT,
    label: str | None = None,
    zero_line: bool = False,
) -> None:
    times, values = _time_values(series, column, timezone, max_points)
    if np.isfinite(values).any():
        axis.plot(times, values, color=color, linewidth=1.15, label=label)
    else:
        axis.text(0.5, 0.5, "No data", transform=axis.transAxes, ha="center", color=MUTED_COLOR)
    if zero_line:
        axis.axhline(0, color=MUTED_COLOR, linewidth=0.75, alpha=0.75)


def _plot_multiple(
    axis: plt.Axes,
    series: TelemetrySeries,
    metrics: Iterable[tuple[str, str, str]],
    timezone: ZoneInfo,
    max_points: int,
    *,
    zero_line: bool = False,
    zero_is_missing: bool = False,
) -> None:
    plotted = False
    for column, label, color in metrics:
        times, values = _time_values(series, column, timezone, max_points)
        if zero_is_missing:
            values[np.isclose(values, 0.0, equal_nan=False)] = math.nan
        if np.isfinite(values).any():
            axis.plot(times, values, linewidth=1.05, label=label, color=color)
            plotted = True
    if plotted:
        axis.legend(loc="best", fontsize=7, frameon=False, ncol=2)
    else:
        axis.text(0.5, 0.5, "No data", transform=axis.transAxes, ha="center", color=MUTED_COLOR)
    if zero_line:
        axis.axhline(0, color=MUTED_COLOR, linewidth=0.75, alpha=0.75)


def _format_local_time(timestamp: int, timezone: ZoneInfo) -> str:
    return dt.datetime.fromtimestamp(timestamp / 1000, timezone).strftime("%Y-%m-%d %H:%M:%S %Z")


def _latest_value(series: TelemetrySeries, column: str, suffix: str, decimals: int = 1) -> str:
    value = series.rows[-1][column]
    if value is None:
        return "—"
    return f"{float(value):.{decimals}f}{suffix}"


def _cover_page(
    pdf: PdfPages,
    bms_series: Sequence[TelemetrySeries],
    inverter_series: Sequence[TelemetrySeries],
    timezone: ZoneInfo,
    database: Path,
    start_ms: int,
    end_ms: int,
) -> None:
    figure = plt.figure(figsize=PAGE_SIZE, facecolor=PAGE_BACKGROUND)
    figure.text(0.055, 0.90, "Energy system telemetry", fontsize=30, fontweight="bold", color=TEXT_COLOR)
    figure.text(0.057, 0.845, "BMS and inverter report", fontsize=16, color=ACCENT)
    generated = dt.datetime.now(timezone).strftime("%Y-%m-%d %H:%M:%S %Z")
    interval = f"{_format_local_time(start_ms, timezone)}  —  {_format_local_time(end_ms, timezone)}"
    figure.text(0.057, 0.785, f"Data interval: {interval}", fontsize=10, color=MUTED_COLOR)
    figure.text(0.057, 0.755, f"Generated: {generated}", fontsize=10, color=MUTED_COLOR)
    figure.text(0.057, 0.725, f"Database: {database}", fontsize=8, color=MUTED_COLOR)

    table_rows: list[list[str]] = []
    for series in bms_series:
        table_rows.append([
            "BMS",
            series.name,
            f"{series.sample_count:,}",
            _format_local_time(series.first_ms, timezone),
            _format_local_time(series.last_ms, timezone),
            ", ".join((
                _latest_value(series, "pack_voltage_v", " V", 2),
                _latest_value(series, "current_a", " A", 2),
                _latest_value(series, "soc_pct", "%", 0),
            )),
        ])
    for series in inverter_series:
        table_rows.append([
            "Inverter",
            series.name,
            f"{series.sample_count:,}",
            _format_local_time(series.first_ms, timezone),
            _format_local_time(series.last_ms, timezone),
            ", ".join((
                _latest_value(series, "pv_power_w", " W", 0),
                _latest_value(series, "output_power_w", " W", 0),
                _latest_value(series, "battery_soc_pct", "%", 0),
            )),
        ])

    axis = figure.add_axes((0.055, 0.20, 0.89, 0.45))
    axis.axis("off")
    table = axis.table(
        cellText=table_rows,
        colLabels=("Type", "Device", "Samples", "First sample", "Last sample", "Latest values"),
        cellLoc="left",
        colLoc="left",
        loc="upper left",
        bbox=(0, 0, 1, 1),
        colWidths=(0.09, 0.14, 0.09, 0.22, 0.22, 0.24),
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    for (row, _column), cell in table.get_celld().items():
        cell.set_edgecolor(GRID_COLOR)
        cell.set_linewidth(0.6)
        cell.set_text_props(color=TEXT_COLOR)
        cell.set_facecolor("#e5f1ee" if row == 0 else PLOT_BACKGROUND)
        if row == 0:
            cell.set_text_props(weight="bold", color=TEXT_COLOR)

    figure.text(
        0.057,
        0.125,
        "Charts include valid BMS samples only. Missing intervals are shown as line breaks; "
        "stored source rows are never changed by this report.",
        fontsize=9,
        color=MUTED_COLOR,
    )
    _finalize_figure(figure, pdf)


def _bms_overview_page(
    pdf: PdfPages,
    all_series: Sequence[TelemetrySeries],
    timezone: ZoneInfo,
    max_points: int,
) -> None:
    figure, top = _new_figure("BMS overview", "Pack voltage, current and state of charge")
    axes = figure.subplots(
        len(all_series),
        3,
        squeeze=False,
        gridspec_kw={"left": 0.06, "right": 0.97, "bottom": 0.08, "top": top, "hspace": 0.42, "wspace": 0.23},
    )
    for row, series in enumerate(all_series):
        specifications = (
            ("pack_voltage_v", "Pack voltage", "V", False),
            ("current_a", "Battery current", "A", True),
            ("soc_pct", "State of charge", "%", False),
        )
        for column_index, (column, title, ylabel, zero_line) in enumerate(specifications):
            axis = axes[row][column_index]
            _configure_axis(axis, ylabel)
            _plot_column(axis, series, column, timezone, max_points, zero_line=zero_line)
            axis.set_title(f"{series.name} — {title}", fontsize=9, color=TEXT_COLOR, loc="left")
            if column == "soc_pct":
                axis.set_ylim(-3, 103)
    _finalize_figure(figure, pdf)


def _bms_detail_page(
    pdf: PdfPages,
    series: TelemetrySeries,
    timezone: ZoneInfo,
    max_points: int,
) -> None:
    subtitle = (
        f"{series.sample_count:,} valid samples · "
        f"{_format_local_time(series.first_ms, timezone)} — {_format_local_time(series.last_ms, timezone)}"
    )
    figure, top_text = _new_figure(f"BMS detail — {series.name}", subtitle)
    axes = figure.subplots(
        2,
        3,
        gridspec_kw={"left": 0.06, "right": 0.97, "bottom": 0.08, "top": top_text, "hspace": 0.33, "wspace": 0.24},
    )

    plots = (
        (axes[0][0], "Pack voltage", "V", (("pack_voltage_v", "Pack", ACCENT),), False),
        (axes[0][1], "Battery current", "A", (("current_a", "Current", SERIES_COLORS[1]),), True),
        (axes[0][2], "State of charge", "%", (("soc_pct", "SOC", SERIES_COLORS[2]),), False),
        (
            axes[1][0],
            "Temperatures",
            "°C",
            (
                ("temp_mos_c", "MOS", SERIES_COLORS[0]),
                ("temp_t1_c", "T1", SERIES_COLORS[1]),
                ("temp_t2_c", "T2", SERIES_COLORS[2]),
            ),
            False,
        ),
        (
            axes[1][1],
            "Cell voltage envelope",
            "V",
            (
                ("cell_min_v", "Minimum", SERIES_COLORS[1]),
                ("cell_max_v", "Maximum", SERIES_COLORS[2]),
            ),
            False,
        ),
    )
    for axis, title, ylabel, metrics, zero_line in plots:
        _configure_axis(axis, ylabel)
        _plot_multiple(
            axis,
            series,
            metrics,
            timezone,
            max_points,
            zero_line=zero_line,
            zero_is_missing=title == "Temperatures",
        )
        axis.set_title(title, fontsize=10, color=TEXT_COLOR, loc="left")
        if title == "State of charge":
            axis.set_ylim(-3, 103)

    axis = axes[1][2]
    _configure_axis(axis, "V")
    latest_cells: list[float] = []
    for row in reversed(series.rows):
        try:
            latest_cells = [float(value) for value in json.loads(row["cells_v_json"])]
        except (TypeError, ValueError, json.JSONDecodeError):
            latest_cells = []
        if latest_cells:
            break
    axis.set_title("Latest individual cell voltages", fontsize=10, color=TEXT_COLOR, loc="left")
    if latest_cells:
        indexes = np.arange(1, len(latest_cells) + 1)
        cell_array = np.asarray(latest_cells)
        spread_mv = (float(np.max(cell_array)) - float(np.min(cell_array))) * 1000
        colors = [ACCENT] * len(latest_cells)
        colors[int(np.argmin(cell_array))] = SERIES_COLORS[2]
        colors[int(np.argmax(cell_array))] = SERIES_COLORS[1]
        axis.bar(indexes, cell_array, color=colors, width=0.72)
        axis.set_xticks(indexes)
        axis.set_xticklabels([f"C{index:02d}" for index in indexes], rotation=45, ha="right", fontsize=7)
        padding = max(0.003, (float(np.max(cell_array)) - float(np.min(cell_array))) * 1.5)
        axis.set_ylim(float(np.min(cell_array)) - padding, float(np.max(cell_array)) + padding)
        axis.text(
            0.98,
            0.94,
            f"spread {spread_mv:.0f} mV",
            transform=axis.transAxes,
            ha="right",
            va="top",
            fontsize=8,
            color=MUTED_COLOR,
        )
    else:
        axis.text(0.5, 0.5, "No cell data", transform=axis.transAxes, ha="center", color=MUTED_COLOR)
    _finalize_figure(figure, pdf)


def _inverter_overview_page(
    pdf: PdfPages,
    all_series: Sequence[TelemetrySeries],
    timezone: ZoneInfo,
    max_points: int,
) -> None:
    figure, top_text = _new_figure("Inverter overview", "Power production, load and battery behavior")
    axes = figure.subplots(
        len(all_series),
        3,
        squeeze=False,
        gridspec_kw={"left": 0.06, "right": 0.97, "bottom": 0.08, "top": top_text, "hspace": 0.42, "wspace": 0.23},
    )
    for row, series in enumerate(all_series):
        specifications = (
            (
                axes[row][0],
                "Power",
                "W",
                (
                    ("pv_power_w", "PV", SERIES_COLORS[2]),
                    ("output_power_w", "Output", SERIES_COLORS[1]),
                    ("grid_power_w", "Grid", SERIES_COLORS[4]),
                ),
                False,
            ),
            (
                axes[row][1],
                "Battery current",
                "A",
                (("battery_current_a", "Battery", ACCENT),),
                True,
            ),
            (
                axes[row][2],
                "SOC and load",
                "%",
                (
                    ("battery_soc_pct", "Battery SOC", SERIES_COLORS[0]),
                    ("load_pct", "Load", SERIES_COLORS[3]),
                ),
                False,
            ),
        )
        for axis, title, ylabel, metrics, zero_line in specifications:
            _configure_axis(axis, ylabel)
            _plot_multiple(axis, series, metrics, timezone, max_points, zero_line=zero_line)
            axis.set_title(f"{series.name} — {title}", fontsize=9, color=TEXT_COLOR, loc="left")
            if title == "SOC and load":
                axis.set_ylim(-3, 103)
    _finalize_figure(figure, pdf)


def _inverter_detail_page(
    pdf: PdfPages,
    series: TelemetrySeries,
    timezone: ZoneInfo,
    max_points: int,
) -> None:
    subtitle = (
        f"{series.sample_count:,} samples · "
        f"{_format_local_time(series.first_ms, timezone)} — {_format_local_time(series.last_ms, timezone)}"
    )
    figure, top_text = _new_figure(f"Inverter detail — {series.name}", subtitle)
    axes = figure.subplots(
        2,
        3,
        gridspec_kw={"left": 0.06, "right": 0.97, "bottom": 0.08, "top": top_text, "hspace": 0.33, "wspace": 0.24},
    )
    plots = (
        (
            axes[0][0],
            "Power flow",
            "W",
            (
                ("pv_power_w", "PV", SERIES_COLORS[2]),
                ("output_power_w", "Output", SERIES_COLORS[1]),
                ("grid_power_w", "Grid", SERIES_COLORS[4]),
                ("battery_power_w", "Battery", SERIES_COLORS[0]),
            ),
            True,
        ),
        (axes[0][1], "Battery voltage", "V", (("battery_voltage_v", "Battery", ACCENT),), False),
        (axes[0][2], "Battery current", "A", (("battery_current_a", "Battery", SERIES_COLORS[1]),), True),
        (
            axes[1][0],
            "State of charge and load",
            "%",
            (
                ("battery_soc_pct", "Battery SOC", SERIES_COLORS[0]),
                ("load_pct", "Load", SERIES_COLORS[3]),
            ),
            False,
        ),
        (
            axes[1][1],
            "AC voltages",
            "V",
            (
                ("grid_voltage_v", "Grid", SERIES_COLORS[4]),
                ("output_voltage_v", "Output", SERIES_COLORS[1]),
            ),
            False,
        ),
        (
            axes[1][2],
            "Temperatures",
            "°C",
            (
                ("inverter_temperature_c", "Inverter", SERIES_COLORS[0]),
                ("dcdc_temperature_c", "DC/DC", SERIES_COLORS[2]),
                ("pv_temperature_c", "PV", SERIES_COLORS[3]),
            ),
            False,
        ),
    )
    for axis, title, ylabel, metrics, zero_line in plots:
        _configure_axis(axis, ylabel)
        _plot_multiple(
            axis,
            series,
            metrics,
            timezone,
            max_points,
            zero_line=zero_line,
            zero_is_missing=title == "Temperatures",
        )
        axis.set_title(title, fontsize=10, color=TEXT_COLOR, loc="left")
        if title == "State of charge and load":
            axis.set_ylim(-3, 103)
    _finalize_figure(figure, pdf)


def _database_range(connection: sqlite3.Connection) -> tuple[int, int]:
    row = connection.execute(
        """
        SELECT MIN(sampled_at_unix_ms), MAX(sampled_at_unix_ms)
        FROM (
            SELECT sampled_at_unix_ms FROM telemetry_samples
            UNION ALL
            SELECT sampled_at_unix_ms FROM inverter_samples
        )
        """
    ).fetchone()
    if row is None or row[0] is None or row[1] is None:
        raise RuntimeError("The telemetry database does not contain any samples")
    return int(row[0]), int(row[1])


def _load_series(
    connection: sqlite3.Connection,
    table: str,
    inventory_table: str,
    id_column: str,
    columns: Sequence[str],
    start_ms: int,
    end_ms: int,
    *,
    valid_only: bool = False,
) -> list[TelemetrySeries]:
    inventory_rows = connection.execute(
        f"SELECT {id_column}, name FROM {inventory_table} ORDER BY {id_column}"
    ).fetchall()
    result: list[TelemetrySeries] = []
    valid_clause = " AND valid = 1" if valid_only else ""
    selected_columns = ", ".join(columns)
    for inventory in inventory_rows:
        rows = connection.execute(
            f"""
            SELECT {selected_columns}
            FROM {table}
            WHERE {id_column} = ?
              AND sampled_at_unix_ms BETWEEN ? AND ?
              {valid_clause}
            ORDER BY sampled_at_unix_ms, id
            """,
            (inventory[id_column], start_ms, end_ms),
        ).fetchall()
        if rows:
            result.append(TelemetrySeries(inventory[id_column], inventory["name"], tuple(rows)))
    return result


def generate_report(
    database: Path,
    output: Path,
    *,
    since: dt.datetime | None = None,
    until: dt.datetime | None = None,
    hours: float | None = None,
    timezone_name: str = DEFAULT_TIMEZONE,
    max_points: int = 4000,
) -> tuple[int, int, int]:
    database = database.expanduser().resolve()
    output = output.expanduser().resolve()
    if not database.is_file():
        raise FileNotFoundError(f"Telemetry database not found: {database}")
    if hours is not None and hours <= 0:
        raise ValueError("--hours must be greater than zero")
    if max_points < 100:
        raise ValueError("--max-points must be at least 100")
    timezone = ZoneInfo(timezone_name)

    uri = f"file:{database}?mode=ro"
    with sqlite3.connect(uri, uri=True) as connection:
        connection.row_factory = sqlite3.Row
        connection.execute("BEGIN")
        database_start_ms, database_end_ms = _database_range(connection)
        end_ms = timestamp_ms(until) if until else database_end_ms
        if hours is not None:
            start_ms = end_ms - int(hours * 3_600_000)
        else:
            start_ms = timestamp_ms(since) if since else database_start_ms
        if start_ms > end_ms:
            raise ValueError("The selected start time is after the end time")

        bms_series = _load_series(
            connection,
            "telemetry_samples",
            "bms_sources",
            "source_id",
            BMS_COLUMNS,
            start_ms,
            end_ms,
            valid_only=True,
        )
        inverter_series = _load_series(
            connection,
            "inverter_samples",
            "inverters",
            "inverter_id",
            INVERTER_COLUMNS,
            start_ms,
            end_ms,
        )
        if not bms_series and not inverter_series:
            raise RuntimeError("No telemetry samples exist in the selected interval")

        actual_start_ms = min(series.first_ms for series in (*bms_series, *inverter_series))
        actual_end_ms = max(series.last_ms for series in (*bms_series, *inverter_series))
        output.parent.mkdir(parents=True, exist_ok=True)
        with PdfPages(output) as pdf:
            metadata = pdf.infodict()
            metadata["Title"] = "Energy system telemetry — BMS and inverters"
            metadata["Author"] = "Inverter Board telemetry report generator"
            metadata["Subject"] = "Local BMS and inverter telemetry"
            metadata["CreationDate"] = dt.datetime.now(dt.timezone.utc)
            _cover_page(
                pdf,
                bms_series,
                inverter_series,
                timezone,
                database,
                actual_start_ms,
                actual_end_ms,
            )
            if bms_series:
                _bms_overview_page(pdf, bms_series, timezone, max_points)
                for series in bms_series:
                    _bms_detail_page(pdf, series, timezone, max_points)
            if inverter_series:
                _inverter_overview_page(pdf, inverter_series, timezone, max_points)
                for series in inverter_series:
                    _inverter_detail_page(pdf, series, timezone, max_points)

    page_count = 1 + (1 + len(bms_series) if bms_series else 0) + (
        1 + len(inverter_series) if inverter_series else 0
    )
    sample_count = sum(series.sample_count for series in (*bms_series, *inverter_series))
    return page_count, sample_count, output.stat().st_size


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a PDF report with BMS and inverter telemetry charts.",
    )
    parser.add_argument(
        "--database",
        type=Path,
        default=DEFAULT_DATABASE,
        help=f"SQLite database (default: {DEFAULT_DATABASE})",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Output PDF; defaults to data/reports/telemetry-report-<timestamp>.pdf",
    )
    interval = parser.add_mutually_exclusive_group()
    interval.add_argument(
        "--hours",
        type=float,
        help="Include the latest N hours relative to the newest selected sample",
    )
    interval.add_argument(
        "--since",
        type=parse_timestamp,
        help="Include data at or after this ISO-8601 timestamp (UTC when omitted)",
    )
    parser.add_argument(
        "--until",
        type=parse_timestamp,
        help="Include data at or before this ISO-8601 timestamp (UTC when omitted)",
    )
    parser.add_argument(
        "--timezone",
        default=DEFAULT_TIMEZONE,
        help=f"Timezone used in the charts (default: {DEFAULT_TIMEZONE})",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=4000,
        help="Maximum plotted points per line after peak-preserving reduction (default: 4000)",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    output = args.output
    if output is None:
        timezone = ZoneInfo(args.timezone)
        stamp = dt.datetime.now(timezone).strftime("%Y%m%d-%H%M%S")
        output = DEFAULT_REPORT_DIR / f"telemetry-report-{stamp}.pdf"
    try:
        pages, samples, size = generate_report(
            args.database,
            output,
            since=args.since,
            until=args.until,
            hours=args.hours,
            timezone_name=args.timezone,
            max_points=args.max_points,
        )
    except (FileNotFoundError, RuntimeError, ValueError, sqlite3.Error) as exc:
        parser.error(str(exc))
    print(f"Created {output.resolve()}")
    print(f"Pages: {pages}; charted samples: {samples:,}; size: {size / 1024:.1f} KiB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
