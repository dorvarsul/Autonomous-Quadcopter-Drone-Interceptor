"""Wind & gust disturbance model (Role 1).

Models the air velocity the airframe sees as a **steady wind vector plus a seeded
stochastic gust**. Gusts are a first-order Gauss-Markov (Ornstein-Uhlenbeck) process,
which gives temporally-correlated, band-limited turbulence rather than white noise —
physically closer to real gusts and numerically gentle on the controller.

Determinism (AGENTS.md): the whole gust time-series is **precomputed once** at
construction from a single seeded RNG stream, then sampled by time. This makes
``velocity_at(t)`` a pure function of ``t`` — the same seed yields a byte-identical
disturbance series, and both the plant-force path and the wind-affected
trajectory can sample it without sharing mutable state.

The **calm** preset (zero steady wind, zero gust std) reduces to exactly zero air
velocity at every time, so the dynamics are bit-for-bit the undisturbed dynamics.
"""

from __future__ import annotations

import numpy as np
from numpy.random import Generator
from numpy.typing import NDArray

from interceptor.common import guards
from interceptor.config.params import WindParams

# World axes that gusts act on (all three: horizontal + a smaller vertical component).
_AXES = 3


class WindField:
    """A reproducible world-frame air-velocity field, sampled by time.

    Parameters
    ----------
    params:
        The wind profile (steady vector + gust statistics).
    rng:
        Seeded generator used **only** to draw the gust process. Required when
        ``gust_std_m_s > 0`` (a stochastic disturbance with no seed would be
        irreproducible — fail loud). Ignored when there are no gusts.
    horizon_s:
        Length of the precomputed series [s]. Samples past the horizon clamp to the
        last value rather than raising, so a slightly-too-long run never crashes.
    dt_s:
        Sampling period of the precomputed series [s]; should match the sim step.
    """

    def __init__(
        self,
        params: WindParams,
        rng: Generator | None = None,
        *,
        horizon_s: float = 60.0,
        dt_s: float = 1.0 / 400.0,
    ) -> None:
        if horizon_s <= 0.0 or dt_s <= 0.0:
            raise ValueError("WindField requires positive horizon_s and dt_s.")
        self._dt = float(dt_s)
        self._steady = guards.ensure_vector(
            "steady_velocity_m_s", np.asarray(params.steady_velocity_m_s, dtype=np.float64), 3
        )
        self._gust_std = float(params.gust_std_m_s)
        if self._gust_std < 0.0:
            raise ValueError("gust_std_m_s must be >= 0.")
        tau = float(params.gust_correlation_time_s)
        if tau <= 0.0:
            raise ValueError("gust_correlation_time_s must be > 0.")

        num_samples = int(np.ceil(horizon_s / self._dt)) + 1
        self._gusts = self._precompute_gusts(num_samples, tau, rng)

    def _precompute_gusts(
        self, num_samples: int, tau_s: float, rng: Generator | None
    ) -> NDArray[np.float64]:
        """Precompute the OU gust series ``g[k]`` over the horizon.

        Recurrence: ``g[k] = a*g[k-1] + b*sigma*N(0,1)`` with ``a = exp(-dt/tau)`` and
        ``b = sqrt(1 - a^2)`` so the stationary variance is exactly ``sigma^2`` for any
        dt (the discrete-time exact solution of the OU SDE).
        """
        gusts = np.zeros((num_samples, _AXES), dtype=np.float64)
        if self._gust_std == 0.0:
            return gusts  # calm: identically zero, no RNG consumed
        if rng is None:
            raise ValueError(
                "WindField with gust_std_m_s > 0 requires a seeded rng for "
                "reproducibility (AGENTS.md → seed all randomness)."
            )
        a = float(np.exp(-self._dt / tau_s))
        b = float(np.sqrt(1.0 - a * a))
        noise = rng.standard_normal((num_samples, _AXES))
        for k in range(1, num_samples):
            gusts[k] = a * gusts[k - 1] + b * self._gust_std * noise[k]
        return gusts

    def velocity_at(self, sim_time_s: float) -> NDArray[np.float64]:
        """Return the world-frame air velocity [m/s] at ``sim_time_s``.

        Linear interpolation between precomputed samples keeps the field smooth; times
        beyond the horizon clamp to the final sample.
        """
        t = max(0.0, float(sim_time_s))
        pos = t / self._dt
        i = int(np.floor(pos))
        last = self._gusts.shape[0] - 1
        if i >= last:
            gust = self._gusts[last]
        else:
            frac = pos - i
            gust = (1.0 - frac) * self._gusts[i] + frac * self._gusts[i + 1]
        return self._steady + gust

    def force_on(
        self, body_velocity_world_m_s: NDArray[np.float64], sim_time_s: float, drag_coeff: float
    ) -> NDArray[np.float64]:
        """Aerodynamic disturbance force [N]: ``k * (v_wind - v_body)`` in world frame.

        A single lumped linear coefficient (``constants.WIND_DRAG_COEFF_N_PER_M_S``) —
        explicit and reproducible, no hidden aero surfaces. With calm wind and zero
        body velocity this is exactly zero.
        """
        v_body = guards.ensure_vector("body_velocity_world_m_s", body_velocity_world_m_s, 3)
        relative_air_velocity = self.velocity_at(sim_time_s) - v_body
        return float(drag_coeff) * relative_air_velocity


def calm() -> WindParams:
    """Calm preset: no steady wind, no gusts. Dynamics reduce to undisturbed exactly."""
    return WindParams(steady_velocity_m_s=(0.0, 0.0, 0.0), gust_std_m_s=0.0)


def moderate() -> WindParams:
    """Moderate preset: a light steady breeze with gentle gusts."""
    return WindParams(
        steady_velocity_m_s=(3.0, 0.0, 0.0),
        gust_std_m_s=1.0,
        gust_correlation_time_s=1.5,
    )


def gusty() -> WindParams:
    """Gusty preset: stronger steady wind with sharp, frequent gusts (stress profile)."""
    return WindParams(
        steady_velocity_m_s=(6.0, 2.0, 0.0),
        gust_std_m_s=3.0,
        gust_correlation_time_s=0.6,
    )


WIND_PRESETS = {"calm": calm, "moderate": moderate, "gusty": gusty}
"""Named presets referenced by scenario configs (used by the stress scenarios)."""
