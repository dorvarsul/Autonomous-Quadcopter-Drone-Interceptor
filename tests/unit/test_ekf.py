"""Unit tests for the Extended Kalman Filter (Role 2).

Covers convergence + bounded error on synthetic noisy tracks, latency compensation,
clean LOS-rate output, quality reporting, and fail-loud divergence. The EKF is exercised
only through ``RawSensorMeasurement`` — it never sees ground truth.
"""

from __future__ import annotations

import numpy as np
import pytest

from interceptor.common import frames, guards
from interceptor.common.types import RawSensorMeasurement
from interceptor.config.params import EkfParams
from interceptor.estimation.ekf import ExtendedKalmanFilter


def _measure(
    rel_pos: np.ndarray,
    timestamp_s: float,
    latency_s: float = 0.0,
    rng: np.random.Generator | None = None,
    noise_std=(0.0, 0.0, 0.0),
) -> RawSensorMeasurement:
    """Synthesize a raw measurement of a relative position (optionally noisy)."""
    rng_m = float(np.linalg.norm(rel_pos))
    az, el = frames.los_angles(rel_pos)
    if rng is not None:
        rng_m += rng.normal(0.0, noise_std[0])
        az += rng.normal(0.0, noise_std[1])
        el += rng.normal(0.0, noise_std[2])
    return RawSensorMeasurement(
        range_m=max(0.0, rng_m), los_azimuth_rad=az, los_elevation_rad=el,
        timestamp_s=timestamp_s, latency_s=latency_s,
    )


def test_ekf_converges_on_noisy_static_target():
    """Interceptor moving toward a static target: position error stays small under noise."""
    ekf = ExtendedKalmanFilter(EkfParams())
    rng = np.random.default_rng(0)
    target = np.array([8.0, 3.0, 6.0])
    dt = 1.0 / 100.0
    errs = []
    est = None
    for k in range(400):
        t = k * dt
        interceptor = np.array([0.0, 0.0, 2.0]) + np.array([1.2, 0.5, 0.8]) * t
        rel = target - interceptor
        m = _measure(rel, t, rng=rng, noise_std=(0.3, 0.0035, 0.0035))
        est = ekf.update(m, dt)
        if k > 100:  # after the filter has settled
            errs.append(float(np.linalg.norm(est.relative_position_m - rel)))
    assert np.mean(errs) < 0.5  # bounded, well below the ~0.3 m range noise scale
    assert est is not None and 0.0 < est.quality <= 1.0


def test_ekf_estimates_relative_velocity():
    """On a constant-relative-velocity track the EKF recovers the velocity sign/scale."""
    ekf = ExtendedKalmanFilter(EkfParams())
    rel0 = np.array([10.0, -4.0, 5.0])
    vel = np.array([-1.5, 0.6, -0.4])
    dt = 1.0 / 100.0
    est = None
    for k in range(500):
        t = k * dt
        est = ekf.update(_measure(rel0 + vel * t, t), dt)
    np.testing.assert_allclose(est.relative_velocity_m_s, vel, atol=0.15)


def test_ekf_latency_compensation_predicts_to_current_time():
    """A delayed measurement is predicted forward so the estimate is valid *now*."""
    latency = 0.05
    dt = 1.0 / 100.0
    rel0 = np.array([12.0, 0.0, 4.0])
    vel = np.array([-3.0, 0.0, 0.0])  # closing fast, so latency matters

    def run(latency_s: float) -> np.ndarray:
        ekf = ExtendedKalmanFilter(EkfParams())
        est = None
        for k in range(300):
            t = k * dt
            # Measurement generated at t (stamped), delivered "now" = t + latency.
            m = _measure(rel0 + vel * t, timestamp_s=t, latency_s=latency_s)
            est = ekf.update(m, dt)
        return np.asarray(est.relative_position_m)

    t_final = 299 * dt
    truth_now = rel0 + vel * (t_final + latency)  # true geometry at delivery time
    est_now = run(latency)
    # The latency-compensated estimate matches truth-at-delivery, not the stale sample.
    assert np.linalg.norm(est_now - truth_now) < 0.1


def test_ekf_los_rate_tracks_truth():
    """The clean LOS rate handed to guidance matches the analytic truth on a crossing track."""
    ekf = ExtendedKalmanFilter(EkfParams())
    rel0 = np.array([10.0, 0.0, 3.0])
    vel = np.array([-1.0, 2.0, 0.0])  # lateral crossing -> non-zero azimuth rate
    dt = 1.0 / 100.0
    est = None
    for k in range(600):
        t = k * dt
        est = ekf.update(_measure(rel0 + vel * t, t), dt)
    t_final = 599 * dt
    truth_rate = frames.los_rate_from_relative(rel0 + vel * t_final, vel)
    np.testing.assert_allclose(est.los_rate_rad_s, truth_rate, atol=0.02)


def test_ekf_fails_loud_on_divergence():
    """A tiny divergence bound makes the (legitimately large) covariance trip the guard."""
    ekf = ExtendedKalmanFilter(EkfParams(divergence_covariance_trace_max=1.0))
    with pytest.raises(guards.NumericalInstabilityError):
        ekf.update(_measure(np.array([5.0, 0.0, 2.0]), 0.0), 0.01)


def test_ekf_exposes_acceleration_field_for_maneuvering():
    """The estimate carries a (finite) relative-acceleration field for later use."""
    ekf = ExtendedKalmanFilter(EkfParams())
    est = ekf.update(_measure(np.array([5.0, 1.0, 2.0]), 0.0), 0.01)
    assert est.relative_acceleration_m_s2.shape == (3,)
    assert np.all(np.isfinite(est.relative_acceleration_m_s2))
