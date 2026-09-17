"""Noisy, delayed sensor model (Role 1, Phase 1 — T1.5).

Turns the true interceptor->target geometry into a corrupted
:class:`RawSensorMeasurement` carrying the imperfections the EKF is built to fight:

* **Gaussian noise** (per-channel std) and optional **bias**,
* optional **quantization** (finite sensor resolution),
* a finite **update rate** (the sensor samples slower than the sim steps), and
* a configurable **latency** via a delay buffer that stamps each emitted sample with
  its true age.

Determinism & honesty (AGENTS.md → Role 1): the noise is intentional and must not be
sanitized for downstream convenience; it is drawn from a single **seeded** RNG stream
so a fixed seed reproduces the measurement series exactly. Constructing a sensor with
no profile, or with noise but no seed, **fails loud** — a silent "clean" sensor would
quietly invalidate the whole estimation story.
"""

from __future__ import annotations

from collections import deque

import numpy as np
from numpy.random import Generator

from interceptor.common import frames
from interceptor.common.types import RawSensorMeasurement
from interceptor.config.params import SensorParams
from interceptor.simulation.interfaces import SensorModel

# Tiny epsilon so float time comparisons against the sample grid are robust.
_TIME_EPS_S = 1.0e-9


def _quantize(value: float, step: float) -> float:
    """Round ``value`` to the nearest multiple of ``step`` (no-op if step <= 0)."""
    if step <= 0.0:
        return value
    return float(np.round(value / step) * step)


class NoisyDelayedSensorModel(SensorModel):
    """A range + LOS-bearing sensor with configurable noise, resolution, and delay.

    Internally stateful: it owns its sample clock and a delay buffer, so calling
    ``measure`` every sim step still yields measurements at the configured update rate,
    delayed by the configured latency.
    """

    def __init__(self, params: SensorParams | None, rng: Generator | None) -> None:
        if params is None:
            raise ValueError(
                "NoisyDelayedSensorModel requires an explicit SensorParams profile; "
                "a sensor with no noise/latency profile is forbidden (AGENTS.md → "
                "Role 1 must not sanitize signals)."
            )
        self._params = params
        if params.update_rate_hz <= 0.0:
            raise ValueError("sensor update_rate_hz must be > 0.")
        if params.latency_s < 0.0:
            raise ValueError("sensor latency_s must be >= 0.")

        self._has_noise = (
            params.range_noise_std_m > 0.0
            or params.azimuth_noise_std_rad > 0.0
            or params.elevation_noise_std_rad > 0.0
        )
        if self._has_noise and rng is None:
            raise ValueError(
                "A noisy sensor requires a seeded rng for reproducibility "
                "(AGENTS.md → seed all randomness)."
            )
        self._rng = rng

        self._update_period_s = 1.0 / float(params.update_rate_hz)
        self._next_sample_time_s: float | None = None
        # Buffer of generated samples: (generation_time_s, range, azimuth, elevation).
        self._buffer: deque[tuple[float, float, float, float]] = deque()

    def measure(
        self,
        interceptor_position_m: np.ndarray,
        target_position_m: np.ndarray,
        sim_time_s: float,
    ) -> RawSensorMeasurement:
        """Return a (possibly delayed) raw measurement of the target."""
        t = float(sim_time_s)

        # --- Generate a fresh noisy sample if the sample clock says it is time --------
        if self._next_sample_time_s is None or t + _TIME_EPS_S >= self._next_sample_time_s:
            self._buffer.append(self._sample_truth(interceptor_position_m, target_position_m, t))
            if self._next_sample_time_s is None:
                self._next_sample_time_s = t + self._update_period_s
            else:
                # Advance on the fixed grid; skip-forward if calls arrived late.
                while self._next_sample_time_s <= t + _TIME_EPS_S:
                    self._next_sample_time_s += self._update_period_s
            self._prune(t)

        # --- Emit the sample whose age best matches the configured latency ------------
        gen_time, range_m, azimuth, elevation = self._select_delayed(t)
        return RawSensorMeasurement(
            range_m=range_m,
            los_azimuth_rad=azimuth,
            los_elevation_rad=elevation,
            timestamp_s=gen_time,
            latency_s=max(0.0, t - gen_time),
        )

    def _sample_truth(
        self, interceptor_position_m: np.ndarray, target_position_m: np.ndarray, t: float
    ) -> tuple[float, float, float, float]:
        """Compute true range/LOS from geometry, then corrupt with noise/bias/quant."""
        rel = np.asarray(target_position_m, dtype=np.float64) - np.asarray(
            interceptor_position_m, dtype=np.float64
        )
        true_range = float(np.linalg.norm(rel))
        if true_range < 1e-9:
            true_az, true_el = 0.0, 0.0
        else:
            true_az, true_el = frames.los_angles(rel)

        p = self._params
        range_m = true_range + p.range_bias_m + self._noise(p.range_noise_std_m)
        azimuth = true_az + p.azimuth_bias_rad + self._noise(p.azimuth_noise_std_rad)
        elevation = true_el + p.elevation_bias_rad + self._noise(p.elevation_noise_std_rad)

        range_m = _quantize(range_m, p.range_quantization_m)
        azimuth = _quantize(azimuth, p.angle_quantization_rad)
        elevation = _quantize(elevation, p.angle_quantization_rad)

        # A measured range cannot be negative; clamp (the contract also forbids < 0).
        range_m = max(0.0, range_m)
        return (t, range_m, azimuth, elevation)

    def _noise(self, std: float) -> float:
        """Zero-mean Gaussian draw; exactly 0.0 when the channel std is 0 (no RNG use)."""
        if std <= 0.0 or self._rng is None:
            return 0.0
        return float(self._rng.normal(0.0, std))

    def _select_delayed(self, t: float) -> tuple[float, float, float, float]:
        """Pick the newest buffered sample at least ``latency_s`` old.

        During the initial latency window (no sample is old enough yet) the oldest
        available sample is emitted, stamped with its true (smaller) age, so a
        measurement always exists and the EKF is never starved.
        """
        target_time = t - self._params.latency_s
        chosen = self._buffer[0]
        for sample in self._buffer:
            if sample[0] <= target_time + _TIME_EPS_S:
                chosen = sample
            else:
                break
        return chosen

    def _prune(self, t: float) -> None:
        """Drop samples older than needed to satisfy the latency lookup (bounded memory)."""
        keep_after = t - self._params.latency_s - self._update_period_s
        while len(self._buffer) > 1 and self._buffer[1][0] <= keep_after + _TIME_EPS_S:
            self._buffer.popleft()
