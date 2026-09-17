"""Command Limiter — the single owner of saturation (Role 4).

SAFETY stage. It clamps OGL's *ideal* acceleration request to what the airframe can
physically and safely execute, so guidance never has to know the limits (Interface
Segregation) and saturation is measured in exactly one place (the ``Command Saturation``
KPI, ≤ 5% of flight time). Two bounds are enforced:

* **Tilt limit** — a quad produces horizontal acceleration only by tilting, so the
  horizontal command is capped at ``g * tan(max_tilt)``; beyond that the required tilt
  would exceed ``max_tilt_rad`` and risk loss of control.
* **Total magnitude limit** — the overall acceleration magnitude is capped at
  ``max_acceleration_m_s2`` to protect the rotors.

Each clamp is reported (``saturated`` flag + magnitude removed) and logged, never applied
silently (AGENTS.md → fail loud; saturation must stay measurable). No other layer clamps.
"""

from __future__ import annotations

import logging

import numpy as np

from interceptor.common import frames, guards
from interceptor.common.types import AccelerationCommand, LimitedAccelerationCommand
from interceptor.config import constants
from interceptor.config.params import LimiterParams

_log = logging.getLogger(__name__)

# Commands within this magnitude of the boundary are treated as unsaturated (float slack).
_SATURATION_EPS_M_S2 = 1.0e-9


class AccelerationCommandLimiter:
    """Clamp an acceleration request to the tilt and magnitude bounds, reporting saturation."""

    def __init__(self, params: LimiterParams | None = None) -> None:
        self._p = params or LimiterParams()
        self._max_horizontal = constants.GRAVITY_M_S2 * np.tan(self._p.max_tilt_rad)

    def limit(self, command: AccelerationCommand) -> LimitedAccelerationCommand:
        requested = np.asarray(command.acceleration_m_s2, dtype=np.float64)
        clamped = requested.copy()

        # 1) Tilt limit on the horizontal (X-Y) component.
        horizontal = clamped[[frames.X, frames.Y]]
        h_mag = float(np.linalg.norm(horizontal))
        if h_mag > self._max_horizontal + _SATURATION_EPS_M_S2:
            clamped[[frames.X, frames.Y]] = horizontal * (self._max_horizontal / h_mag)

        # 2) Total magnitude limit.
        total_mag = float(np.linalg.norm(clamped))
        if total_mag > self._p.max_acceleration_m_s2 + _SATURATION_EPS_M_S2:
            clamped = clamped * (self._p.max_acceleration_m_s2 / total_mag)

        removed = float(np.linalg.norm(requested) - np.linalg.norm(clamped))
        removed = max(0.0, removed)
        saturated = removed > _SATURATION_EPS_M_S2
        if saturated:
            _log.warning(
                "command saturation: |requested|=%.3f m/s^2 clamped to |%.3f|; removed %.3f",
                float(np.linalg.norm(requested)),
                float(np.linalg.norm(clamped)),
                removed,
            )

        guards.ensure_finite("limited_acceleration", clamped)
        return LimitedAccelerationCommand(
            acceleration_m_s2=clamped,
            saturated=saturated,
            saturation_magnitude_m_s2=removed,
        )
