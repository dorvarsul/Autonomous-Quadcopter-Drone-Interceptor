"""Trajectory-generator correctness & determinism (Phase 1 — T1.3 / T1.9)."""

from __future__ import annotations

import numpy as np
import pytest

from interceptor.common.rng import RngFactory
from interceptor.simulation.trajectories import (
    LinearTrajectory,
    SinusoidalTrajectory,
    StaticTrajectory,
    VaryingSpeedTrajectory,
    WindAffectedTrajectory,
)
from interceptor.simulation.wind import WindField, calm, gusty

VARYING_SPEED_KMH_TARGET = 90.0
MS_PER_KMH = 1.0 / 3.6


def test_static_is_fixed_and_zero_velocity() -> None:
    traj = StaticTrajectory(np.array([1.0, 2.0, 3.0]))
    for t in (0.0, 5.0, 50.0):
        np.testing.assert_array_equal(traj.position_at(t), [1.0, 2.0, 3.0])
        np.testing.assert_array_equal(traj.velocity_at(t), [0.0, 0.0, 0.0])


def test_linear_position_and_velocity() -> None:
    traj = LinearTrajectory(np.array([0.0, 0.0, 10.0]), np.array([2.0, -1.0, 0.0]))
    np.testing.assert_allclose(traj.position_at(3.0), [6.0, -3.0, 10.0])
    np.testing.assert_allclose(traj.velocity_at(123.0), [2.0, -1.0, 0.0])


def test_sinusoidal_analytic_velocity_matches_finite_difference() -> None:
    traj = SinusoidalTrajectory(
        start_position_m=np.array([0.0, 0.0, 5.0]),
        drift_velocity_m_s=np.array([1.0, 0.0, 0.0]),
        amplitude_m=np.array([0.0, 2.0, 0.5]),
        frequency_hz=0.5,
    )
    t = 1.3
    h = 1e-6
    fd = (traj.position_at(t + h) - traj.position_at(t - h)) / (2 * h)
    np.testing.assert_allclose(traj.velocity_at(t), fd, atol=1e-6)


def test_sinusoidal_weaves_in_z() -> None:
    traj = SinusoidalTrajectory(
        start_position_m=np.zeros(3),
        drift_velocity_m_s=np.zeros(3),
        amplitude_m=np.array([0.0, 0.0, 1.0]),
        frequency_hz=1.0,
    )
    # Quarter period -> peak +Z amplitude.
    np.testing.assert_allclose(traj.position_at(0.25)[2], 1.0, atol=1e-9)


def test_varying_speed_reaches_target_speed_and_is_monotonic() -> None:
    peak = 25.0  # m/s == 90 km/h
    traj = VaryingSpeedTrajectory(
        start_position_m=np.zeros(3),
        heading=np.array([1.0, 0.0, 0.0]),
        initial_speed_m_s=5.0,
        peak_speed_m_s=peak,
        ramp_duration_s=4.0,
    )
    # Exceeds the KPI minimum after the ramp.
    assert np.linalg.norm(traj.velocity_at(10.0)) >= VARYING_SPEED_KMH_TARGET * MS_PER_KMH
    # Distance is strictly increasing.
    xs = [traj.position_at(t)[0] for t in np.linspace(0, 10, 50)]
    assert all(b > a for a, b in zip(xs, xs[1:], strict=False))


def test_varying_speed_distance_integral_matches_velocity() -> None:
    traj = VaryingSpeedTrajectory(
        start_position_m=np.zeros(3),
        heading=np.array([0.0, 1.0, 0.0]),
        initial_speed_m_s=2.0,
        peak_speed_m_s=20.0,
        ramp_duration_s=3.0,
    )
    # Numerically integrate speed and compare to the analytic position.
    ts = np.linspace(0, 8, 8001)
    speeds = [np.linalg.norm(traj.velocity_at(t)) for t in ts]
    integrated = np.trapezoid(speeds, ts)
    np.testing.assert_allclose(traj.position_at(8.0)[1], integrated, rtol=1e-4)


def test_wind_affected_reduces_to_base_in_calm() -> None:
    base = LinearTrajectory(np.array([0.0, 0.0, 5.0]), np.array([1.0, 0.0, 0.0]))
    wind = WindField(calm(), rng=None, horizon_s=10.0)
    traj = WindAffectedTrajectory(base, wind, horizon_s=10.0)
    for t in (0.0, 2.5, 9.0):
        np.testing.assert_allclose(traj.position_at(t), base.position_at(t), atol=1e-12)


def test_wind_affected_is_reproducible_and_perturbs() -> None:
    base = LinearTrajectory(np.zeros(3), np.array([1.0, 0.0, 0.0]))

    def build(seed: int) -> WindAffectedTrajectory:
        rng = RngFactory(seed).stream("wind")
        wind = WindField(gusty(), rng=rng, horizon_s=10.0)
        return WindAffectedTrajectory(base, wind, horizon_s=10.0)

    a1 = build(7).position_at(5.0)
    a2 = build(7).position_at(5.0)
    b = build(99).position_at(5.0)
    np.testing.assert_array_equal(a1, a2)  # same seed -> identical path
    assert not np.allclose(a1, base.position_at(5.0))  # wind actually perturbs
    assert not np.allclose(a1, b)  # different seed -> different path


def test_invalid_configs_fail_loud() -> None:
    with pytest.raises(ValueError):
        VaryingSpeedTrajectory(np.zeros(3), np.zeros(3), 1.0, 2.0, 1.0)  # zero heading
    with pytest.raises(ValueError):
        VaryingSpeedTrajectory(np.zeros(3), np.array([1.0, 0, 0]), 1.0, 2.0, 0.0)  # zero ramp
