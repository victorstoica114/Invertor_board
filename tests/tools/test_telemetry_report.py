import datetime as dt

import numpy as np

from tools.telemetry_report import minmax_indices, parse_timestamp


def test_parse_timestamp_normalizes_zulu_and_naive_values_to_utc():
    assert parse_timestamp("2026-08-03T12:30:00Z") == dt.datetime(
        2026, 8, 3, 12, 30, tzinfo=dt.timezone.utc
    )
    assert parse_timestamp("2026-08-03T12:30:00") == dt.datetime(
        2026, 8, 3, 12, 30, tzinfo=dt.timezone.utc
    )


def test_minmax_indices_preserves_endpoints_and_extremes():
    values = np.zeros(10_000)
    values[1234] = -50
    values[8765] = 75

    indices = minmax_indices(values, 200)

    assert indices[0] == 0
    assert indices[-1] == len(values) - 1
    assert 1234 in indices
    assert 8765 in indices
    assert len(indices) <= 200


def test_minmax_indices_keeps_small_series_unchanged():
    assert np.array_equal(minmax_indices([1.0, 2.0, 3.0], 100), np.arange(3))
