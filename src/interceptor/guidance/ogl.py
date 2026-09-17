"""Optimal Guidance Law — the project's sole guidance law (Role 3, Phase 2 — T2.3).

OGL is modeled as the Linear-Quadratic problem minimizing miss distance and control
effort, ``J = y(t_f)^2 + ∫ u(t)^2 dt``. Its closed-form solution is a
Zero-Effort-Miss law with a **time-varying, lag-aware navigation ratio**::

    a_cmd = N'(t_go / T) / t_go^2 * ZEM

Two Design-Review features distinguish OGL from a naive proportional law:

* **Tilt-delay awareness.** A real quad cannot re-point instantly; its attitude follows
  a first-order lag ``1/(T s + 1)`` with ``T = TILT_DELAY_TIME_CONSTANT_S``. The optimal
  gain that accounts for this lag is the classic OGL schedule (Zarchan), a function of
  ``x = t_go / T`` that tends to 3 (the pure-PN limit) for large ``x`` and rises near
  intercept to pre-empt the lag. We clamp it to ``[nav_ratio_min, nav_ratio_max]`` for
  numerical safety; ``t_go`` is floored upstream so ``x`` never reaches the 0/0 corner.

* **Altitude penalty ``b``.** The Design Review notes the Z (altitude) axis is
  overshoot-prone. ``b`` (default 0.1) de-weights the vertical command, trading a little
  vertical aggressiveness for the elimination of altitude overshoot. It is applied as a
  ``1/(1 + b)`` attenuation of the Z channel here; the exact weighting is retuned in
  Phase 3 (changing ``b`` affects a KPI and needs user confirmation — AGENTS.md).

OGL requests an *ideal* acceleration only. It does not clamp to physical limits (the
Command Limiter's job) nor convert to tilt/motor commands (Flight Control's job).
"""

from __future__ import annotations

import numpy as np

from interceptor.common import frames, guards
from interceptor.common.types import AccelerationCommand, TargetStateEstimate
from interceptor.config.params import GuidanceParams
from interceptor.guidance.interfaces import GuidanceLaw
from interceptor.guidance.time_to_go import time_to_go_s
from interceptor.guidance.zem import zero_effort_miss

# Below this x = t_go/T the lag-aware gain is in its 0/0 corner (formula -> +inf); return
# the max ratio directly rather than dividing two vanishing series.
_SMALL_X = 1.0e-3


def lag_aware_nav_ratio(time_to_go: float, tilt_delay_time_constant_s: float) -> float:
    """Optimal navigation ratio ``N'(x)``, ``x = t_go / T``, for a first-order tilt lag.

    Closed-form OGL gain (Zarchan). For large ``x`` it tends to 3 (pure PN); as ``x``
    shrinks it grows to compensate the lag. Returns a large value in the ``x -> 0`` corner
    so the caller's clamp bounds it; callers clamp the result to their nav-ratio window.
    """
    t = float(tilt_delay_time_constant_s)
    if t <= 0.0:
        return 3.0  # no lag -> pure proportional-navigation limit
    x = float(time_to_go) / t
    if x < _SMALL_X:
        return float("inf")
    ex = np.exp(-x)
    e2x = np.exp(-2.0 * x)
    numerator = 6.0 * x * x * (ex - 1.0 + x)
    denominator = 2.0 * x**3 - 6.0 * x * x + 6.0 * x + 3.0 - 12.0 * x * ex - 3.0 * e2x
    if abs(denominator) < 1e-12 or not np.isfinite(numerator / denominator):
        return float("inf")
    return float(numerator / denominator)


class OptimalGuidanceLaw(GuidanceLaw):
    """Lag-aware, altitude-penalized ZEM guidance (the operational default and only law)."""

    def __init__(self, params: GuidanceParams | None = None) -> None:
        self._p = params or GuidanceParams()

    @property
    def name(self) -> str:
        return "OGL"

    def compute(self, estimate: TargetStateEstimate) -> AccelerationCommand:
        r = np.asarray(estimate.relative_position_m, dtype=np.float64)
        v = np.asarray(estimate.relative_velocity_m_s, dtype=np.float64)
        # Augmented ZEM term is opt-in (see GuidanceParams.use_target_acceleration): the
        # relative-state EKF conflates the interceptor's own maneuver into a_rel, so we
        # zero it for non-maneuvering (Phase 2) engagements to avoid positive feedback.
        if self._p.use_target_acceleration:
            a = np.asarray(estimate.relative_acceleration_m_s2, dtype=np.float64)
        else:
            a = np.zeros(3, dtype=np.float64)

        t_go = time_to_go_s(r, v, self._p)
        nav_ratio = float(
            np.clip(
                lag_aware_nav_ratio(t_go, self._p.tilt_delay_time_constant_s),
                self._p.nav_ratio_min,
                self._p.nav_ratio_max,
            )
        )
        zem = zero_effort_miss(r, v, a, t_go)
        accel = nav_ratio / (t_go * t_go) * zem

        # Altitude penalty: de-weight the Z channel to suppress overshoot (Design Review).
        accel[frames.Z] *= 1.0 / (1.0 + self._p.altitude_penalty_b)

        guards.ensure_finite("ogl_acceleration", accel)
        return AccelerationCommand(acceleration_m_s2=accel)
