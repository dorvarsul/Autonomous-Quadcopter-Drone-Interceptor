"""Extended Kalman Filter for relative target tracking (Role 2, Phase 2 — T2.1).

The EKF is the Estimation stage: it consumes ONLY the raw, noisy, delayed
:class:`RawSensorMeasurement` (range + LOS azimuth/elevation) and produces a clean,
latency-compensated :class:`TargetStateEstimate` for Guidance. It never reads
ground-truth state (AGENTS.md → Pipeline Contract).

State model
-----------
9-state constant-acceleration model of the **relative** target motion in the world
frame (``rel = target - interceptor``)::

    x = [ p(3)  v(3)  a(3) ]^T    (relative position, velocity, acceleration)

The interceptor's own maneuver is not known to this layer (the ``update`` contract
carries no control input), so its acceleration is absorbed into the process noise on
the velocity/acceleration states — a standard, honest choice for a relative-state
tracker. Position is fully observed each update, so velocity and acceleration are
inferred from the position history the filter smooths.

Process (discrete, step ``dt``)::

    p += v*dt + 0.5*a*dt^2 ;  v += a*dt ;  a += w   (a is a random walk)

Measurement (nonlinear) maps the relative position to what the sensor reports::

    range = |p| ;  azimuth = atan2(p_y, p_x) ;  elevation = atan2(p_z, hypot(p_x,p_y))

with the analytic Jacobian in :meth:`_measurement_jacobian`. Angle innovations are
wrapped to (-pi, pi].

Latency compensation
--------------------
Each measurement is stamped with its generation time and age (Phase 1 sensor latency).
The filter keeps its state synchronized to the measurement generation time and, for the
value it publishes to Guidance, predicts that post-update state **forward by the
latency** so Guidance sees an estimate valid at the current wall-clock instant, not a
stale one (AGENTS.md → Role 2 must compensate sensor latency).

Fail loud
---------
Non-finite states or a covariance-trace blow-up raise
:class:`~interceptor.common.guards.NumericalInstabilityError` rather than emitting
silent garbage (AGENTS.md → fail loud; EKF divergence must surface).
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from interceptor.common import frames, guards
from interceptor.common.types import RawSensorMeasurement, TargetStateEstimate
from interceptor.config.params import EkfParams
from interceptor.estimation.interfaces import Estimator

# State layout indices.
_POS = slice(0, 3)
_VEL = slice(3, 6)
_ACC = slice(6, 9)
_STATE_DIM = 9

# Small epsilons guarding the measurement Jacobian near the vertical singularity and a
# degenerate zero-range state.
_HORIZONTAL_EPS_M = 1.0e-6
_RANGE_EPS_M = 1.0e-9


def _wrap_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi] so an innovation never jumps by ~2*pi."""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def _position_from_measurement(m: RawSensorMeasurement) -> NDArray[np.float64]:
    """Invert (range, az, el) back into a relative position vector (filter init)."""
    horizontal = m.range_m * np.cos(m.los_elevation_rad)
    return np.array(
        [
            horizontal * np.cos(m.los_azimuth_rad),
            horizontal * np.sin(m.los_azimuth_rad),
            m.range_m * np.sin(m.los_elevation_rad),
        ],
        dtype=np.float64,
    )


class ExtendedKalmanFilter(Estimator):
    """Relative-target EKF (range + LOS bearings → clean state estimate + LOS rate)."""

    def __init__(self, params: EkfParams | None = None) -> None:
        self._p = params or EkfParams()
        self._x = np.zeros(_STATE_DIM, dtype=np.float64)
        self._cov = np.eye(_STATE_DIM, dtype=np.float64)
        self._filter_time_s: float | None = None  # measurement-time the state is valid at
        self._r = np.diag(
            [
                self._p.measurement_noise_range_m2,
                self._p.measurement_noise_angle_rad2,
                self._p.measurement_noise_angle_rad2,
            ]
        ).astype(np.float64)

    # ------------------------------------------------------------------ Estimator API
    def update(self, measurement: RawSensorMeasurement, dt_s: float) -> TargetStateEstimate:
        """Fuse one measurement and return a latency-compensated state estimate.

        ``dt_s`` is a fallback prediction interval; the filter prefers the elapsed time
        derived from consecutive measurement timestamps, which is robust to whatever
        cadence the orchestrator calls it at.
        """
        t_meas = float(measurement.timestamp_s)

        if self._filter_time_s is None:
            self._initialize(measurement)
        else:
            dt = t_meas - self._filter_time_s
            if dt <= 0.0:  # equal/out-of-order stamp: fall back to the scheduler dt
                dt = max(0.0, float(dt_s))
            if dt > 0.0:
                self._predict(dt)
            self._correct(measurement)

        self._filter_time_s = t_meas
        self._guard_divergence()
        return self._publish(measurement)

    # ------------------------------------------------------------------ filter steps
    def _initialize(self, measurement: RawSensorMeasurement) -> None:
        """Seed the state from the first measurement geometry; velocity/accel unknown."""
        self._x = np.zeros(_STATE_DIM, dtype=np.float64)
        self._x[_POS] = _position_from_measurement(measurement)
        scale = self._p.initial_covariance_scale
        # Position is directly observed (tighter prior); velocity/acceleration are not.
        diag = np.array(
            [scale, scale, scale, scale * 10.0, scale * 10.0, scale * 10.0,
             scale * 100.0, scale * 100.0, scale * 100.0],
            dtype=np.float64,
        )
        self._cov = np.diag(diag)

    def _predict(self, dt: float) -> None:
        """Constant-acceleration prediction over ``dt`` seconds."""
        f = self._transition_matrix(dt)
        self._x = f @ self._x
        self._cov = f @ self._cov @ f.T + self._process_noise(dt)

    def _correct(self, measurement: RawSensorMeasurement) -> None:
        """EKF measurement update with wrapped angle innovations."""
        z = np.array(
            [measurement.range_m, measurement.los_azimuth_rad, measurement.los_elevation_rad],
            dtype=np.float64,
        )
        h_x = self._expected_measurement(self._x[_POS])
        innovation = z - h_x
        innovation[1] = _wrap_angle(innovation[1])
        innovation[2] = _wrap_angle(innovation[2])

        jac = self._measurement_jacobian(self._x[_POS])
        s = jac @ self._cov @ jac.T + self._r
        kalman_gain = self._cov @ jac.T @ np.linalg.inv(s)
        self._x = self._x + kalman_gain @ innovation
        # Joseph form keeps the covariance symmetric positive-definite under rounding.
        identity = np.eye(_STATE_DIM, dtype=np.float64)
        factor = identity - kalman_gain @ jac
        self._cov = factor @ self._cov @ factor.T + kalman_gain @ self._r @ kalman_gain.T

    # ------------------------------------------------------------------ model matrices
    @staticmethod
    def _transition_matrix(dt: float) -> NDArray[np.float64]:
        f = np.eye(_STATE_DIM, dtype=np.float64)
        f[_POS, _VEL] = np.eye(3) * dt
        f[_POS, _ACC] = np.eye(3) * (0.5 * dt * dt)
        f[_VEL, _ACC] = np.eye(3) * dt
        return f

    def _process_noise(self, dt: float) -> NDArray[np.float64]:
        """Diagonal process-noise covariance scaled by ``dt`` (Phase 3 retunes shape)."""
        p = self._p
        diag = np.array(
            [p.process_noise_position] * 3
            + [p.process_noise_velocity] * 3
            + [p.process_noise_acceleration] * 3,
            dtype=np.float64,
        )
        return np.diag(diag) * dt

    @staticmethod
    def _expected_measurement(pos: NDArray[np.float64]) -> NDArray[np.float64]:
        """h(x): relative position → (range, azimuth, elevation)."""
        rng = float(np.linalg.norm(pos))
        if rng < _RANGE_EPS_M:
            return np.zeros(3, dtype=np.float64)
        az, el = frames.los_angles(pos)
        return np.array([rng, az, el], dtype=np.float64)

    @staticmethod
    def _measurement_jacobian(pos: NDArray[np.float64]) -> NDArray[np.float64]:
        """Analytic ∂(range, az, el)/∂position; velocity/accel columns are zero."""
        px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
        jac = np.zeros((3, _STATE_DIM), dtype=np.float64)
        rng_sq = px * px + py * py + pz * pz
        rng = float(np.sqrt(rng_sq))
        horiz_sq = px * px + py * py
        horiz = float(np.sqrt(horiz_sq))
        if rng < _RANGE_EPS_M:
            return jac
        # d range / d pos
        jac[0, 0:3] = [px / rng, py / rng, pz / rng]
        if horiz < _HORIZONTAL_EPS_M:
            # Near-vertical: azimuth ill-conditioned; leave its row zero, elevation uses
            # d el/d pz = h/r^2 -> 0 here as well. This matches the kinematics guard.
            return jac
        # d azimuth / d pos
        jac[1, 0:3] = [-py / horiz_sq, px / horiz_sq, 0.0]
        # d elevation / d pos
        jac[2, 0:3] = [
            -px * pz / (rng_sq * horiz),
            -py * pz / (rng_sq * horiz),
            horiz / rng_sq,
        ]
        return jac

    # ------------------------------------------------------------------ output / guards
    def _guard_divergence(self) -> None:
        """Fail loud on NaN/Inf or covariance blow-up (EKF divergence)."""
        guards.ensure_finite("ekf_state", self._x)
        guards.ensure_finite("ekf_covariance", self._cov)
        trace = float(np.trace(self._cov))
        if trace > self._p.divergence_covariance_trace_max:
            raise guards.NumericalInstabilityError(
                f"EKF covariance trace {trace:.3e} exceeds divergence bound "
                f"{self._p.divergence_covariance_trace_max:.3e}; the filter has diverged."
            )

    def _publish(self, measurement: RawSensorMeasurement) -> TargetStateEstimate:
        """Predict the post-update state forward by the measurement latency and emit it."""
        latency = float(measurement.latency_s)
        x_now = self._transition_matrix(latency) @ self._x if latency > 0.0 else self._x

        pos = np.array(x_now[_POS], dtype=np.float64)
        vel = np.array(x_now[_VEL], dtype=np.float64)
        acc = np.array(x_now[_ACC], dtype=np.float64)
        rng = float(np.linalg.norm(pos))
        los_rate = frames.los_rate_from_relative(pos, vel)

        # Quality in (0, 1]: tight position covariance -> high confidence.
        pos_cov_trace = float(np.trace(self._cov[_POS, _POS]))
        quality = 1.0 / (1.0 + max(0.0, pos_cov_trace))

        return TargetStateEstimate(
            relative_position_m=pos,
            relative_velocity_m_s=vel,
            range_m=max(0.0, rng),
            los_rate_rad_s=los_rate,
            angular_rates_rad_s=np.zeros(3, dtype=np.float64),  # interceptor gyro, not target
            covariance=np.array(self._cov, dtype=np.float64),
            quality=float(np.clip(quality, 0.0, 1.0)),
            relative_acceleration_m_s2=acc,
        )
