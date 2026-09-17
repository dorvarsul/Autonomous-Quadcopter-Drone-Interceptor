"""Flight Control outer loop, ~50 Hz (Role 4, Phase 2 — T2.5).

Translates the clamped acceleration command into a target attitude + collective thrust
via quadrotor **differential flatness**: a quad accelerates only along its body +Z
(thrust) axis, so the desired specific-force direction fixes the tilt.

    f = a_cmd + g*ẑ          (world specific force the rotors must produce)
    thrust = m * |f|         (collective thrust magnitude)
    body +Z must align with f -> gives the target roll/pitch

Frames follow :mod:`common.frames` (world Z-up; body FLU; intrinsic Z-Y-X euler). Yaw is
held at 0 — the engagement never commands a heading change, and the outer-loop contract
does not carry the current yaw, so a fixed 0 reference is both consistent and honest. A
hover command (``a_cmd = 0``) yields zero tilt and weight-compensating thrust ``m*g``.
"""

from __future__ import annotations

import numpy as np

from interceptor.common import frames, guards
from interceptor.common.types import AttitudeReference, LimitedAccelerationCommand
from interceptor.config import constants

# Held yaw reference: the interceptor does not command heading changes this project.
_YAW_REFERENCE_RAD = 0.0


class DifferentialFlatnessOuterLoop:
    """Map a safe acceleration command to a target attitude + thrust (~50 Hz)."""

    def __init__(self, mass_kg: float = constants.QUAD_MASS_KG) -> None:
        self._mass = float(mass_kg)

    def compute_attitude(self, command: LimitedAccelerationCommand) -> AttitudeReference:
        a_cmd = np.asarray(command.acceleration_m_s2, dtype=np.float64)
        # Specific force the rotors must generate = commanded accel + gravity support.
        f = a_cmd + np.array([0.0, 0.0, constants.GRAVITY_M_S2], dtype=np.float64)
        f_mag = float(np.linalg.norm(f))
        thrust = self._mass * f_mag

        if f_mag < 1e-9:
            # Free-fall command (a_cmd = -g): no defined tilt; stay level, zero thrust.
            return AttitudeReference(
                roll_rad=0.0, pitch_rad=0.0, yaw_rad=_YAW_REFERENCE_RAD, thrust_n=0.0
            )

        n = f / f_mag  # desired body +Z direction in world (yaw = 0 frame)
        # Solve R(0, theta, phi) @ ẑ = n for roll/pitch (see frames Z-Y-X convention):
        #   n_x = cos(phi) sin(theta) ;  n_y = -sin(phi) ;  n_z = cos(phi) cos(theta)
        roll = float(np.arctan2(-n[frames.Y], np.hypot(n[frames.X], n[frames.Z])))
        pitch = float(np.arctan2(n[frames.X], n[frames.Z]))

        guards.ensure_finite("outer_thrust", thrust)
        return AttitudeReference(
            roll_rad=roll,
            pitch_rad=pitch,
            yaw_rad=_YAW_REFERENCE_RAD,
            thrust_n=max(0.0, thrust),
        )
