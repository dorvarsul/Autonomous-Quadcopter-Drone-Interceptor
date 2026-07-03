"""Sensor noise/latency correctness (Phase 1 — T1.5 / T1.9)."""

from __future__ import annotations

import numpy as np
import pytest

from interceptor.common.rng import RngFactory
from interceptor.config.params import SensorParams
from interceptor.simulation.sensors import NoisyDelayedSensorModel


def test_missing_profile_fails_loud() -> None:
    with pytest.raises(ValueError):
        NoisyDelayedSensorModel(None, RngFactory(0).stream("s"))


def test_noisy_sensor_without_seed_fails_loud() -> None:
    params = SensorParams(range_noise_std_m=0.5, latency_s=0.0)
    with pytest.raises(ValueError):
        NoisyDelayedSensorModel(params, None)


def test_residual_statistics_match_configuration() -> None:
    # High update rate, zero latency: every call is a fresh, immediately-emitted sample.
    params = SensorParams(
        range_noise_std_m=0.5,
        azimuth_noise_std_rad=0.01,
        elevation_noise_std_rad=0.01,
        range_bias_m=0.2,
        update_rate_hz=1000.0,
        latency_s=0.0,
    )
    sensor = NoisyDelayedSensorModel(params, RngFactory(2024).stream("sensor"))

    interceptor = np.zeros(3)
    target = np.array([20.0, 0.0, 0.0])  # true range 20, az 0, el 0
    n = 20000
    dt = 1.0 / 1000.0
    range_residuals = np.empty(n)
    az_residuals = np.empty(n)
    for k in range(n):
        m = sensor.measure(interceptor, target, k * dt)
        range_residuals[k] = m.range_m - 20.0
        az_residuals[k] = m.los_azimuth_rad - 0.0

    # Range residual mean ~ bias, std ~ configured; azimuth zero-mean, configured std.
    assert range_residuals.mean() == pytest.approx(0.2, abs=0.02)
    assert range_residuals.std() == pytest.approx(0.5, rel=0.05)
    assert az_residuals.mean() == pytest.approx(0.0, abs=0.001)
    assert az_residuals.std() == pytest.approx(0.01, rel=0.05)


def test_latency_delays_measurement_by_configured_amount() -> None:
    # Noise-free so we can read the delay directly from the geometry.
    params = SensorParams(
        range_noise_std_m=0.0,
        azimuth_noise_std_rad=0.0,
        elevation_noise_std_rad=0.0,
        update_rate_hz=100.0,
        latency_s=0.1,
    )
    sensor = NoisyDelayedSensorModel(params, None)

    interceptor = np.zeros(3)
    dt = 1.0 / 100.0
    latest = None
    # Target recedes along +X at 1 m/s, so range == time numerically (range0 = 10).
    for k in range(200):
        t = k * dt
        target = np.array([10.0 + t, 0.0, 0.0])
        latest = sensor.measure(interceptor, target, t)

    # At the final step the emitted sample is ~0.1 s old.
    assert latest is not None
    assert latest.latency_s == pytest.approx(0.1, abs=dt)
    # Its range corresponds to the geometry ~0.1 s in the past.
    final_t = 199 * dt
    assert latest.range_m == pytest.approx(10.0 + (final_t - 0.1), abs=0.02)


def test_finite_update_rate_holds_value_between_samples() -> None:
    params = SensorParams(
        range_noise_std_m=0.0,
        azimuth_noise_std_rad=0.0,
        elevation_noise_std_rad=0.0,
        update_rate_hz=10.0,  # new sample every 0.1 s
        latency_s=0.0,
    )
    sensor = NoisyDelayedSensorModel(params, None)
    interceptor = np.zeros(3)

    # Sim at 100 Hz: 10 sim steps per sensor sample -> the reading should be piecewise
    # constant across each 0.1 s window.
    readings = []
    for k in range(30):
        t = k * 0.01
        target = np.array([10.0 + t, 0.0, 0.0])
        readings.append(sensor.measure(interceptor, target, t).range_m)

    # Within the first window (steps 1..9) the value equals the step-0 sample.
    assert readings[1] == pytest.approx(readings[0])
    assert readings[9] == pytest.approx(readings[0])
    # A new window produces a fresh (larger) value.
    assert readings[10] > readings[0]


def test_zero_noise_sensor_is_deterministic_without_rng() -> None:
    params = SensorParams(
        range_noise_std_m=0.0,
        azimuth_noise_std_rad=0.0,
        elevation_noise_std_rad=0.0,
        update_rate_hz=100.0,
        latency_s=0.0,
    )
    sensor = NoisyDelayedSensorModel(params, None)
    m = sensor.measure(np.zeros(3), np.array([3.0, 4.0, 0.0]), 0.0)
    assert m.range_m == pytest.approx(5.0)
