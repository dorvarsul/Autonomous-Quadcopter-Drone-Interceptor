"""Unit tests for the multi-rate scheduler (tick counts, rate ratios, drift guards)."""

from __future__ import annotations

import pytest

from interceptor.config import constants
from interceptor.pipeline.scheduler import MultiRateScheduler


def _default_scheduler() -> MultiRateScheduler:
    return MultiRateScheduler(
        sim_hz=constants.SIM_HZ,
        inner_loop_hz=constants.INNER_LOOP_HZ,
        outer_loop_hz=constants.OUTER_LOOP_HZ,
        estimation_hz=constants.ESTIMATION_HZ,
        guidance_hz=constants.GUIDANCE_HZ,
    )


def test_inner_loop_fires_every_step_at_sim_rate():
    sched = _default_scheduler()  # inner == sim == 400 Hz
    ticks = list(sched.ticks(400))
    assert all(t.run_inner_loop for t in ticks)


def test_outer_loop_fires_at_its_rate():
    sched = _default_scheduler()
    ticks = list(sched.ticks(constants.SIM_HZ))  # one second
    fired = sum(t.run_outer_loop for t in ticks)
    assert fired == constants.OUTER_LOOP_HZ  # 50 Hz over 1 s


def test_estimation_and_guidance_counts_match_rates():
    sched = _default_scheduler()
    n = constants.SIM_HZ
    ticks = list(sched.ticks(n))
    assert sum(t.run_estimation for t in ticks) == constants.ESTIMATION_HZ
    assert sum(t.run_guidance for t in ticks) == constants.GUIDANCE_HZ


def test_all_loops_fire_on_step_zero():
    sched = _default_scheduler()
    first = next(sched.ticks(1))
    assert first.run_inner_loop and first.run_outer_loop
    assert first.run_estimation and first.run_guidance


def test_sim_time_advances_by_dt():
    sched = _default_scheduler()
    ticks = list(sched.ticks(3))
    dt = 1.0 / constants.SIM_HZ
    assert ticks[0].sim_time_s == 0.0
    assert ticks[1].sim_time_s == pytest.approx(dt)
    assert ticks[2].sim_time_s == pytest.approx(2 * dt)


def test_expected_tick_count_helper_matches_actual():
    sched = _default_scheduler()
    n = 1000
    ticks = list(sched.ticks(n))
    assert sched.expected_tick_count(n, constants.OUTER_LOOP_HZ) == sum(
        t.run_outer_loop for t in ticks
    )


def test_non_divisor_rate_fails_loud():
    with pytest.raises(ValueError):
        MultiRateScheduler(sim_hz=400, inner_loop_hz=400, outer_loop_hz=30,
                           estimation_hz=100, guidance_hz=50)


def test_rate_above_sim_rate_fails_loud():
    with pytest.raises(ValueError):
        MultiRateScheduler(sim_hz=400, inner_loop_hz=800, outer_loop_hz=50,
                           estimation_hz=100, guidance_hz=50)
