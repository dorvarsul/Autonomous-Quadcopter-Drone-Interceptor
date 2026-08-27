"""Unit tests for the KPI measurement module.

KPIs are verified against **hand-computed** synthetic traces so the arithmetic — not just
the plumbing — is checked. No MuJoCo: these operate on in-memory / tiny-CSV
logs, so they run in the fast, dependency-free suite.
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np
import pytest

from interceptor.analysis.kpis import (
    RunTrace,
    compute_kpis,
    kpis_from_log,
    load_run_trace,
)
from interceptor.config import constants


def _trace(times, interceptor, target, saturated) -> RunTrace:
    return RunTrace(
        time_s=np.asarray(times, dtype=np.float64),
        interceptor_pos_m=np.asarray(interceptor, dtype=np.float64),
        target_pos_m=np.asarray(target, dtype=np.float64),
        saturated=np.asarray(saturated, dtype=bool),
    )


def test_kpis_hand_computed_hit_with_overshoot_and_saturation():
    """A synthetic approach with a known min range, overshoot, and saturation fraction."""
    # Interceptor closes on a static target at [10, 0, 5]; ranges 10,7,4,1.0,0.5.
    times = [0.0, 1.0, 2.0, 3.0, 4.0]
    target = [[10.0, 0.0, 5.0]] * 5
    interceptor = [
        [0.0, 0.0, 5.0],   # range 10
        [3.0, 0.0, 5.0],   # range 7
        [6.0, 0.0, 6.0],   # range 4, Z overshoot +1.0 above target
        [9.0, 0.0, 5.2],   # range 1.0  <= R_miss -> first intercept frame
        [9.5, 0.0, 5.0],   # range 0.5  -> closest approach
    ]
    saturated = [False, False, True, True, False]  # 2/5 = 0.4

    k = compute_kpis(_trace(times, interceptor, target, saturated),
                     time_to_intercept_max_s=constants.T_INT_STATIC_MAX_S)

    assert k.miss_distance_m == pytest.approx(0.5)
    assert k.time_to_intercept_s == pytest.approx(3.0)  # first frame within 1.05 m
    assert k.z_overshoot_m == pytest.approx(1.0)        # max(interceptor_z - target_z, 0)
    assert k.command_saturation_frac == pytest.approx(0.4)
    assert k.max_target_speed_kmh == pytest.approx(0.0)  # static target

    assert k.miss_ok and k.time_ok
    assert not k.z_overshoot_ok            # 1.0 > 0.5 m
    assert not k.saturation_ok             # 0.4 > 0.05
    assert not k.success
    assert k.intercepted


def test_no_intercept_reports_infinite_time_and_fails_time_kpi():
    """A run that never reaches the miss threshold has t_int = inf and is not intercepted."""
    times = [0.0, 1.0, 2.0]
    target = [[10.0, 0.0, 5.0]] * 3
    interceptor = [[0.0, 0.0, 5.0], [2.0, 0.0, 5.0], [4.0, 0.0, 5.0]]  # closest 6 m
    k = compute_kpis(_trace(times, interceptor, target, [False] * 3),
                     time_to_intercept_max_s=constants.T_INT_STATIC_MAX_S)
    assert not np.isfinite(k.time_to_intercept_s)
    assert not k.intercepted
    assert not k.time_ok
    assert not k.miss_ok  # 6 m > 1.05 m


def test_max_target_speed_from_finite_difference():
    """Max target speed is the peak per-frame finite-difference speed in km/h."""
    times = [0.0, 1.0, 2.0]
    # Target jumps 3 m in X over 1 s (3 m/s = 10.8 km/h), then holds.
    target = [[5.0, 0.0, 5.0], [8.0, 0.0, 5.0], [8.0, 0.0, 5.0]]
    interceptor = [[0.0, 0.0, 5.0]] * 3
    k = compute_kpis(_trace(times, interceptor, target, [False] * 3),
                     time_to_intercept_max_s=constants.T_INT_MOVING_MAX_S)
    assert k.max_target_speed_kmh == pytest.approx(10.8)


def test_overshoot_only_counts_up_to_closest_approach():
    """Altitude excess after the closest-approach frame must not count as overshoot."""
    times = [0.0, 1.0, 2.0]
    target = [[5.0, 0.0, 5.0]] * 3
    # Closest approach at index 1 (range 0); a big Z excess appears only afterwards.
    interceptor = [[0.0, 0.0, 5.0], [5.0, 0.0, 5.0], [6.0, 0.0, 9.0]]
    k = compute_kpis(_trace(times, interceptor, target, [False] * 3),
                     time_to_intercept_max_s=constants.T_INT_STATIC_MAX_S)
    assert k.z_overshoot_m == pytest.approx(0.0)  # the +4 m excess is post-intercept


def test_descending_intercept_does_not_count_initial_altitude_gap():
    """Starting above the target and descending onto it is not overshoot.

    A monotone descent from +3 m above to the target altitude must report ~0 overshoot
    (the initial gap is closed, not overshot); only sinking *below* the target counts.
    """
    times = [0.0, 1.0, 2.0, 3.0]
    target = [[6.0, 0.0, 1.0]] * 4
    interceptor = [
        [0.0, 0.0, 4.0],   # +3 above target
        [2.0, 0.0, 3.0],   # +2
        [4.0, 0.0, 1.6],   # +0.6
        [5.8, 0.0, 0.8],   # -0.2 below target -> overshoot 0.2, closest approach
    ]
    k = compute_kpis(_trace(times, interceptor, target, [False] * 4),
                     time_to_intercept_max_s=constants.T_INT_STATIC_MAX_S)
    assert k.z_overshoot_m == pytest.approx(0.2)  # only the dip below the target counts


def _write_log(path: Path, rows: list[dict]) -> None:
    cols = ["sim_time_s", "interceptor_x_m", "interceptor_y_m", "interceptor_z_m",
            "target_x_m", "target_y_m", "target_z_m", "saturated"]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=cols, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def test_load_run_trace_parses_columns_and_boolean(tmp_path: Path):
    """load_run_trace reads the pose + saturated columns and the "1"/"0" boolean encoding."""
    path = tmp_path / "run_log.csv"
    _write_log(path, [
        {"sim_time_s": 0.0, "interceptor_x_m": 0.0, "interceptor_y_m": 0.0,
         "interceptor_z_m": 5.0, "target_x_m": 3.0, "target_y_m": 0.0,
         "target_z_m": 5.0, "saturated": "0"},
        {"sim_time_s": 1.0, "interceptor_x_m": 3.0, "interceptor_y_m": 0.0,
         "interceptor_z_m": 5.0, "target_x_m": 3.0, "target_y_m": 0.0,
         "target_z_m": 5.0, "saturated": "1"},
    ])
    k = kpis_from_log(path, time_to_intercept_max_s=constants.T_INT_STATIC_MAX_S)
    assert k.miss_distance_m == pytest.approx(0.0)       # frame 1 is a direct hit
    assert k.command_saturation_frac == pytest.approx(0.5)  # one of two frames saturated


def test_load_run_trace_fails_loud_on_empty_log(tmp_path: Path):
    """An empty run log is a defect, not a zero-KPI success (fail loud)."""
    path = tmp_path / "run_log.csv"
    _write_log(path, [])
    with pytest.raises(ValueError, match="no data rows"):
        load_run_trace(path)
