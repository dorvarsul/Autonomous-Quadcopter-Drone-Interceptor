"""Zero-Effort-Miss (ZEM) computation for the guidance layer (Role 3, Phase 2 — T2.2).

The **Zero-Effort-Miss** is the relative position that would remain at intercept time if
the interceptor applied no further acceleration — i.e. the predicted miss vector::

    ZEM = r + v * t_go + 1/2 * a * t_go^2

where ``r``, ``v``, ``a`` are the filtered *relative* position/velocity/acceleration
(target − interceptor) and ``t_go`` is the time-to-go. The augmented (``a``) term folds
in the estimated target acceleration so evasive/maneuvering targets are anticipated —
this is what lets OGL subsume the rejected APN baseline.

OGL commands acceleration proportional to ``ZEM / t_go^2``, driving ZEM → 0. The
component of ZEM **perpendicular to the line of sight** is the part pure line-of-sight
nulling would act on; :func:`perpendicular_component` exposes it for analysis/tests
(e.g. a constant-bearing closing geometry has ~zero perpendicular ZEM).
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


def zero_effort_miss(
    relative_position_m: NDArray[np.float64],
    relative_velocity_m_s: NDArray[np.float64],
    relative_acceleration_m_s2: NDArray[np.float64],
    time_to_go_s: float,
) -> NDArray[np.float64]:
    """Return the augmented ZEM vector [m] in the world frame."""
    r = np.asarray(relative_position_m, dtype=np.float64)
    v = np.asarray(relative_velocity_m_s, dtype=np.float64)
    a = np.asarray(relative_acceleration_m_s2, dtype=np.float64)
    t = float(time_to_go_s)
    return r + v * t + 0.5 * a * t * t


def perpendicular_component(
    vector: NDArray[np.float64], line_of_sight_m: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Component of ``vector`` perpendicular to the line-of-sight direction [same units]."""
    los = np.asarray(line_of_sight_m, dtype=np.float64)
    norm = float(np.linalg.norm(los))
    if norm < 1e-12:
        return np.asarray(vector, dtype=np.float64)
    unit = los / norm
    v = np.asarray(vector, dtype=np.float64)
    return v - np.dot(v, unit) * unit
