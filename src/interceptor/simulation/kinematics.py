"""Ground-truth relative kinematics (Role 1).

Computes the *true* engagement geometry between interceptor and target each step:
relative position/velocity, range, Line-of-Sight (LOS) angles and **LOS rate**, and
closing velocity. These follow the sign/axis conventions documented in
:mod:`interceptor.common.frames`.

**Boundary (AGENTS.md → Pipeline Contract):** this is the raw truth the *sensors*
corrupt. It lives strictly inside the Simulation/sensor layer. Estimation, Guidance,
and Control must never read it directly — doing so is the "cheating with ground truth"
defect the architecture forbids. The type below is deliberately *not* one of the
pipeline message contracts in ``common.types`` for exactly that reason.

LOS-rate derivation (analytic, from relative position ``r`` and relative velocity
``v`` in the world frame):

    azimuth  = atan2(r_y, r_x)
    -> az_rate    = (r_x * v_y - r_y * v_x) / (r_x^2 + r_y^2)

    elevation = atan2(r_z, h),  h = hypot(r_x, r_y)
    -> el_rate    = (h * v_z - r_z * dh/dt) / range^2,  dh/dt = (r_x v_x + r_y v_y) / h

    closing speed = -d(range)/dt = -(r . v) / range   (positive when range shrinks)
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from interceptor.common import frames, guards

# Below this horizontal radius the azimuth (and its rate) are ill-conditioned: the
# target is nearly straight overhead/below. We report a zero azimuth rate there rather
# than dividing by ~0, and flag it for callers that care.
_HORIZONTAL_SINGULARITY_M = 1.0e-9


@dataclass(frozen=True)
class GroundTruthRelativeState:
    """The true interceptor->target geometry at one instant (Simulation-layer only).

    Every field is ground truth; none of it may cross into Estimation/Guidance/Control.
    """

    relative_position_m: NDArray[np.float64]  # target - interceptor, world frame [m]
    relative_velocity_m_s: NDArray[np.float64]  # d/dt of the above [m/s]
    range_m: float  # |relative_position| [m]
    los_azimuth_rad: float  # LOS azimuth, frames convention [rad]
    los_elevation_rad: float  # LOS elevation, frames convention [rad]
    los_rate_rad_s: NDArray[np.float64]  # [azimuth_rate, elevation_rate] [rad/s]
    closing_speed_m_s: float  # -d(range)/dt; positive when closing [m/s]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "relative_position_m",
            guards.freeze(guards.ensure_vector("relative_position_m", self.relative_position_m, 3)),
        )
        object.__setattr__(
            self,
            "relative_velocity_m_s",
            guards.freeze(
                guards.ensure_vector("relative_velocity_m_s", self.relative_velocity_m_s, 3)
            ),
        )
        guards.ensure_in_range("range_m", self.range_m, 0.0, np.inf)
        object.__setattr__(
            self,
            "los_rate_rad_s",
            guards.freeze(guards.ensure_vector("los_rate_rad_s", self.los_rate_rad_s, 2)),
        )


def compute_relative_state(
    interceptor_position_m: NDArray[np.float64],
    interceptor_velocity_m_s: NDArray[np.float64],
    target_position_m: NDArray[np.float64],
    target_velocity_m_s: NDArray[np.float64],
) -> GroundTruthRelativeState:
    """Compute the true relative kinematics from both bodies' world pos/vel.

    Fails loud on a zero-range engagement, where LOS angles are undefined (consistent
    with :func:`frames.los_angles`).
    """
    p_i = guards.ensure_vector("interceptor_position_m", interceptor_position_m, 3)
    v_i = guards.ensure_vector("interceptor_velocity_m_s", interceptor_velocity_m_s, 3)
    p_t = guards.ensure_vector("target_position_m", target_position_m, 3)
    v_t = guards.ensure_vector("target_velocity_m_s", target_velocity_m_s, 3)

    rel = p_t - p_i
    rel_vel = v_t - v_i
    range_m = float(np.linalg.norm(rel))
    if range_m < _HORIZONTAL_SINGULARITY_M:
        raise guards.ContractViolationError(
            "Relative kinematics undefined at zero range (interceptor and target "
            "co-located); this should be treated as an intercept, not a measurement."
        )

    azimuth, elevation = frames.los_angles(rel)

    rx, ry, rz = float(rel[frames.X]), float(rel[frames.Y]), float(rel[frames.Z])
    vx, vy, vz = float(rel_vel[frames.X]), float(rel_vel[frames.Y]), float(rel_vel[frames.Z])
    horizontal_sq = rx * rx + ry * ry
    horizontal = float(np.sqrt(horizontal_sq))

    if horizontal < _HORIZONTAL_SINGULARITY_M:
        # Target nearly overhead/below: azimuth rate ill-conditioned -> report 0.
        azimuth_rate = 0.0
        elevation_rate = 0.0
    else:
        azimuth_rate = (rx * vy - ry * vx) / horizontal_sq
        dh_dt = (rx * vx + ry * vy) / horizontal
        elevation_rate = (horizontal * vz - rz * dh_dt) / (range_m * range_m)

    closing_speed = -float(np.dot(rel, rel_vel)) / range_m

    return GroundTruthRelativeState(
        relative_position_m=rel,
        relative_velocity_m_s=rel_vel,
        range_m=range_m,
        los_azimuth_rad=azimuth,
        los_elevation_rad=elevation,
        los_rate_rad_s=np.array([azimuth_rate, elevation_rate], dtype=np.float64),
        closing_speed_m_s=closing_speed,
    )
