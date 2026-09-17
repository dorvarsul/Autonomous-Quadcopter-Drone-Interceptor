"""Time-to-go estimation for the guidance layer (Role 3, Phase 2 — T2.2).

``time_to_go`` (``t_go``) is the estimated time remaining until intercept. It drives the
lag-aware navigation ratio and scales the Zero-Effort-Miss, so its conditioning matters:
a naive ``range / closing_speed`` blows up when the closing speed is ~0 (a static target,
or an interceptor still at rest), exactly the regime the from-rest engagement starts in.

We therefore:
 * use ``t_go = range / closing_speed`` when genuinely closing, and
 * fall back to ``range / reference_closing_speed`` when closing speed is ~0 or negative,
   so OGL still synthesizes a finite closing command (ZEM trajectory-shaping), then
 * clamp to ``[time_to_go_min_s, time_to_go_max_s]`` to bound the terminal ``1/t_go^2``.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from interceptor.config.params import GuidanceParams

# Below this closing speed the geometry is treated as "not closing" and the reference
# closing speed is used to synthesize t_go instead of dividing by ~0.
_CLOSING_SPEED_EPS_M_S = 1.0e-3


def closing_speed_m_s(
    relative_position_m: NDArray[np.float64], relative_velocity_m_s: NDArray[np.float64]
) -> float:
    """Closing speed ``-d(range)/dt = -(r·v)/|r|`` [m/s]; positive when range shrinks."""
    r = np.asarray(relative_position_m, dtype=np.float64)
    v = np.asarray(relative_velocity_m_s, dtype=np.float64)
    rng = float(np.linalg.norm(r))
    if rng < 1e-9:
        return 0.0
    return -float(np.dot(r, v)) / rng


def time_to_go_s(
    relative_position_m: NDArray[np.float64],
    relative_velocity_m_s: NDArray[np.float64],
    params: GuidanceParams,
) -> float:
    """Return a well-conditioned ``t_go`` [s] for the given relative geometry."""
    rng = float(np.linalg.norm(np.asarray(relative_position_m, dtype=np.float64)))
    v_c = closing_speed_m_s(relative_position_m, relative_velocity_m_s)
    if v_c > _CLOSING_SPEED_EPS_M_S:
        t_go = rng / v_c
    else:
        # Static/from-rest: synthesize a t_go from a reference closing speed so guidance
        # still produces a command that builds closing velocity.
        t_go = rng / max(params.reference_closing_speed_m_s, _CLOSING_SPEED_EPS_M_S)
    return float(np.clip(t_go, params.time_to_go_min_s, params.time_to_go_max_s))
