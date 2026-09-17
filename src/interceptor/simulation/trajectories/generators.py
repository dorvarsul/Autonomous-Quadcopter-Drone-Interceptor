"""Concrete target-trajectory generators (Role 1, Phase 1 — T1.3).

The target is a *threat to be tracked*, not a controlled body: its motion is
**prescribed kinematically** (position is a closed-form function of time), so the
interceptor's job is purely to chase it. Every generator therefore exposes both
``position_at`` (the :class:`TargetTrajectory` contract) and an analytic
``velocity_at`` used by the ground-truth relative kinematics (T1.4).

All families are deterministic: an identical configuration reproduces an identical
path. The only stochastic family, :class:`WindAffectedTrajectory`, draws its
randomness from a seeded :class:`~interceptor.simulation.wind.WindField`, so it too is
byte-reproducible for a fixed seed (T1.3 DoD).

Units: positions [m], velocities [m/s], times [s], angular frequency via Hz.
"""

from __future__ import annotations

from abc import abstractmethod

import numpy as np
from numpy.typing import NDArray

from interceptor.common import guards
from interceptor.simulation.interfaces import TargetTrajectory
from interceptor.simulation.wind import WindField

# Step used for the finite-difference fallback when a subclass has no closed-form
# velocity. Small enough that the central difference is accurate to ~1e-8 m/s here.
_VELOCITY_FD_STEP_S = 1.0e-4


class AnalyticTrajectory(TargetTrajectory):
    """Base for closed-form trajectories: adds a ground-truth ``velocity_at``.

    Subclasses implement ``position_at``; ``velocity_at`` defaults to a central finite
    difference but is overridden with the exact derivative wherever one is available.
    """

    @abstractmethod
    def position_at(self, sim_time_s: float) -> NDArray[np.float64]:
        """Return the target world position [m] at ``sim_time_s``."""

    def velocity_at(self, sim_time_s: float) -> NDArray[np.float64]:
        """Return the target world velocity [m/s] at ``sim_time_s`` (central difference)."""
        h = _VELOCITY_FD_STEP_S
        forward = self.position_at(sim_time_s + h)
        backward = self.position_at(sim_time_s - h)
        return (forward - backward) / (2.0 * h)


class StaticTrajectory(AnalyticTrajectory):
    """A target fixed at a point in space. Time-of-intercept target for the < 10 s KPI."""

    def __init__(self, position_m: NDArray[np.float64]) -> None:
        self._position = guards.ensure_vector(
            "position_m", np.asarray(position_m, dtype=np.float64), 3
        )

    def position_at(self, sim_time_s: float) -> NDArray[np.float64]:
        return self._position.copy()

    def velocity_at(self, sim_time_s: float) -> NDArray[np.float64]:
        return np.zeros(3, dtype=np.float64)


class LinearTrajectory(AnalyticTrajectory):
    """Constant-velocity straight line: ``p(t) = p0 + v * t``."""

    def __init__(
        self, start_position_m: NDArray[np.float64], velocity_m_s: NDArray[np.float64]
    ) -> None:
        self._p0 = guards.ensure_vector(
            "start_position_m", np.asarray(start_position_m, dtype=np.float64), 3
        )
        self._v = guards.ensure_vector(
            "velocity_m_s", np.asarray(velocity_m_s, dtype=np.float64), 3
        )

    def position_at(self, sim_time_s: float) -> NDArray[np.float64]:
        return self._p0 + self._v * float(sim_time_s)

    def velocity_at(self, sim_time_s: float) -> NDArray[np.float64]:
        return self._v.copy()


class SinusoidalTrajectory(AnalyticTrajectory):
    """Evasive weave: constant-velocity drift plus a 3D sinusoidal oscillation.

    ``p(t) = p0 + v*t + A * sin(2*pi*f*t + phase)``

    The per-axis amplitude ``A`` and the scalar frequency ``f`` make this the primary
    *evasive* target (Design Review §7). A non-zero Z amplitude exercises the altitude
    axis the b-penalty is designed to tame.
    """

    def __init__(
        self,
        start_position_m: NDArray[np.float64],
        drift_velocity_m_s: NDArray[np.float64],
        amplitude_m: NDArray[np.float64],
        frequency_hz: float,
        phase_rad: NDArray[np.float64] | None = None,
    ) -> None:
        self._p0 = guards.ensure_vector(
            "start_position_m", np.asarray(start_position_m, dtype=np.float64), 3
        )
        self._v = guards.ensure_vector(
            "drift_velocity_m_s", np.asarray(drift_velocity_m_s, dtype=np.float64), 3
        )
        self._amp = guards.ensure_vector(
            "amplitude_m", np.asarray(amplitude_m, dtype=np.float64), 3
        )
        if frequency_hz < 0.0:
            raise ValueError("frequency_hz must be >= 0.")
        self._omega = 2.0 * np.pi * float(frequency_hz)
        phase = np.zeros(3) if phase_rad is None else np.asarray(phase_rad, dtype=np.float64)
        self._phase = guards.ensure_vector("phase_rad", phase, 3)

    def position_at(self, sim_time_s: float) -> NDArray[np.float64]:
        t = float(sim_time_s)
        return self._p0 + self._v * t + self._amp * np.sin(self._omega * t + self._phase)

    def velocity_at(self, sim_time_s: float) -> NDArray[np.float64]:
        t = float(sim_time_s)
        return self._v + self._amp * self._omega * np.cos(self._omega * t + self._phase)


class VaryingSpeedTrajectory(AnalyticTrajectory):
    """Straight-line motion whose speed ramps from an initial to a peak speed.

    Speed ramps linearly over ``ramp_duration_s`` then holds at ``peak_speed_m_s``::

        s(t) = s0 + (s_peak - s0) * min(t, T_ramp) / T_ramp        (t >= 0)

    Position is the exact integral of ``s(t)`` along the (unit) heading. Parametrize
    ``peak_speed_m_s`` to >= 25 m/s (90 km/h) to exercise ``MAX_TARGET_SPEED_MIN_KMH``.
    """

    def __init__(
        self,
        start_position_m: NDArray[np.float64],
        heading: NDArray[np.float64],
        initial_speed_m_s: float,
        peak_speed_m_s: float,
        ramp_duration_s: float,
    ) -> None:
        self._p0 = guards.ensure_vector(
            "start_position_m", np.asarray(start_position_m, dtype=np.float64), 3
        )
        heading = guards.ensure_vector("heading", np.asarray(heading, dtype=np.float64), 3)
        norm = float(np.linalg.norm(heading))
        if norm < 1e-9:
            raise ValueError("heading must be a non-zero direction vector.")
        self._dir = heading / norm
        self._s0 = float(initial_speed_m_s)
        self._speak = float(peak_speed_m_s)
        if ramp_duration_s <= 0.0:
            raise ValueError("ramp_duration_s must be > 0.")
        self._t_ramp = float(ramp_duration_s)

    def _distance_along(self, t: float) -> float:
        """Arc length travelled by ``t`` — exact integral of the ramp-then-hold speed."""
        if t <= 0.0:
            return 0.0
        if t <= self._t_ramp:
            # integral of s0 + (speak-s0)*tau/T_ramp from 0..t
            return self._s0 * t + (self._speak - self._s0) * t * t / (2.0 * self._t_ramp)
        ramp_distance = self._s0 * self._t_ramp + (self._speak - self._s0) * self._t_ramp / 2.0
        return ramp_distance + self._speak * (t - self._t_ramp)

    def _speed_at(self, t: float) -> float:
        if t <= 0.0:
            return self._s0
        if t >= self._t_ramp:
            return self._speak
        return self._s0 + (self._speak - self._s0) * t / self._t_ramp

    def position_at(self, sim_time_s: float) -> NDArray[np.float64]:
        return self._p0 + self._dir * self._distance_along(float(sim_time_s))

    def velocity_at(self, sim_time_s: float) -> NDArray[np.float64]:
        return self._dir * self._speed_at(float(sim_time_s))


class WindAffectedTrajectory(AnalyticTrajectory):
    """A base trajectory displaced by the integral of a seeded wind field.

    The wind pushes the (lightweight) target off its nominal path by a displacement
    proportional to the integral of the air velocity::

        p(t) = p_base(t) + coupling * integral_0^t v_wind(tau) d(tau)

    The displacement integral is precomputed once on a fixed grid from the deterministic
    :class:`WindField`, so the perturbed path is fully reproducible for a fixed seed and
    a zero-wind field leaves the base path untouched exactly.
    """

    def __init__(
        self,
        base: AnalyticTrajectory,
        wind: WindField,
        coupling: float = 1.0,
        *,
        horizon_s: float = 60.0,
        dt_s: float = 1.0 / 400.0,
    ) -> None:
        self._base = base
        self._coupling = float(coupling)
        self._dt = float(dt_s)
        if horizon_s <= 0.0 or dt_s <= 0.0:
            raise ValueError("WindAffectedTrajectory requires positive horizon_s and dt_s.")
        # Precompute cumulative wind displacement via the trapezoidal rule on a fixed
        # grid; deterministic because WindField.velocity_at is deterministic.
        num = int(np.ceil(horizon_s / self._dt)) + 1
        times = np.arange(num) * self._dt
        velocities = np.array([wind.velocity_at(float(t)) for t in times], dtype=np.float64)
        increments = 0.5 * (velocities[1:] + velocities[:-1]) * self._dt
        self._displacement = np.zeros((num, 3), dtype=np.float64)
        self._displacement[1:] = np.cumsum(increments, axis=0)

    def _displacement_at(self, t: float) -> NDArray[np.float64]:
        t = max(0.0, t)
        pos = t / self._dt
        i = int(np.floor(pos))
        last = self._displacement.shape[0] - 1
        if i >= last:
            return self._displacement[last]
        frac = pos - i
        return (1.0 - frac) * self._displacement[i] + frac * self._displacement[i + 1]

    def position_at(self, sim_time_s: float) -> NDArray[np.float64]:
        t = float(sim_time_s)
        return self._base.position_at(t) + self._coupling * self._displacement_at(t)
