"""Rotor actuator & motor dynamics (Role 1, Phase 1 — T1.2).

Maps the four rotor speeds in a :class:`MotorCommand` to the body-frame force and
torque the physics engine applies, using the quadratic rotor model:

    thrust_i = THRUST_COEFF_KT * rpm_i^2        [N]   (always along body +Z)
    drag_i   = TORQUE_COEFF_KQ * rpm_i^2        [N*m] (yaw reaction, sign per spin)

This is the **physical actuator boundary**: incoming RPMs are clamped to
``[MOTOR_RPM_MIN, MOTOR_RPM_MAX]`` here (a real rotor cannot exceed its limits), and a
clamp is reported as a saturation event rather than applied silently (AGENTS.md → fail
loud; KPI: command saturation must stay measurable). The Phase 2 motor mixer must
respect the *same* bound so the two never disagree.

Geometry is the "+" configuration in MotorCommand order
``[front(+X), right(-Y), back(-X), left(+Y)]``. Roll/pitch torque comes from the arm
lever; yaw torque from differential rotor drag.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from interceptor.common import frames, guards
from interceptor.config import constants

# Spin directions in rotor order: opposite arms spin the same way so equal RPM yields
# zero net yaw torque (front/back = +1, right/left = -1).
_SPIN_DIRECTIONS = np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64)


@dataclass(frozen=True)
class RotorSaturationEvent:
    """Reported when one or more rotor commands hit the physical RPM bound."""

    any_saturated: bool  # True if any rotor was clamped this step
    per_rotor: NDArray[np.float64]  # clamped RPM actually applied [RPM]
    max_overshoot_rpm: float  # largest |requested - clamped| over the four rotors [RPM]


class RotorActuatorModel:
    """Converts rotor RPMs into body-frame wrench, enforcing the RPM saturation bound."""

    def __init__(
        self,
        arm_length_m: float = constants.ARM_LENGTH_M,
        thrust_coeff_kt: float = constants.THRUST_COEFF_KT,
        torque_coeff_kq: float = constants.TORQUE_COEFF_KQ,
        rpm_min: float = constants.MOTOR_RPM_MIN,
        rpm_max: float = constants.MOTOR_RPM_MAX,
    ) -> None:
        self._arm = float(arm_length_m)
        self._kt = float(thrust_coeff_kt)
        self._kq = float(torque_coeff_kq)
        self._rpm_min = float(rpm_min)
        self._rpm_max = float(rpm_max)

    def clamp_rpm(self, rotor_rpm: NDArray[np.float64]) -> RotorSaturationEvent:
        """Clamp rotor RPMs to the physical bound and report any saturation."""
        requested = guards.ensure_vector("rotor_rpm", rotor_rpm, 4)
        clamped = np.clip(requested, self._rpm_min, self._rpm_max)
        overshoot = float(np.max(np.abs(requested - clamped)))
        return RotorSaturationEvent(
            any_saturated=overshoot > 0.0,
            per_rotor=guards.freeze(clamped),
            max_overshoot_rpm=overshoot,
        )

    def thrust_per_rotor_n(self, rotor_rpm: NDArray[np.float64]) -> NDArray[np.float64]:
        """Per-rotor thrust [N] = kT * rpm^2 (after clamping)."""
        clamped = self.clamp_rpm(rotor_rpm).per_rotor
        return self._kt * clamped * clamped

    def body_wrench(
        self, rotor_rpm: NDArray[np.float64]
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], RotorSaturationEvent]:
        """Return ``(body_force[3], body_torque[3], saturation)`` for the rotor command.

        Force is purely along body +Z (collective thrust). Torque combines the arm-lever
        roll/pitch moments and the yaw reaction drag.
        """
        saturation = self.clamp_rpm(rotor_rpm)
        rpm = saturation.per_rotor
        thrust = self._kt * rpm * rpm  # [front, right, back, left]
        total_thrust = float(np.sum(thrust))

        # "+"-config arm levers: roll about +X from the left/right pair, pitch about +Y
        # from the back/front pair (a force along +Z at +X arm gives a -Y/pitch torque).
        roll = self._arm * (thrust[3] - thrust[1])  # left - right
        pitch = self._arm * (thrust[2] - thrust[0])  # back - front
        # Yaw reaction opposes rotor spin: tau_z = sum(-spin_i * kQ * rpm_i^2).
        yaw = float(np.sum(-_SPIN_DIRECTIONS * self._kq * rpm * rpm))

        body_force = np.array([0.0, 0.0, total_thrust], dtype=np.float64)
        body_torque = np.array([roll, pitch, yaw], dtype=np.float64)
        guards.ensure_finite("body_force", body_force)
        guards.ensure_finite("body_torque", body_torque)
        return body_force, body_torque, saturation

    def hover_rpm(self, mass_kg: float = constants.QUAD_MASS_KG) -> float:
        """Per-rotor RPM that exactly balances weight: sqrt(m*g / (4*kT)) [RPM]."""
        weight = mass_kg * constants.GRAVITY_M_S2
        return float(np.sqrt(weight / (4.0 * self._kt)))


# Re-exported axis indices for callers assembling wrenches by name.
ROLL, PITCH, YAW = frames.X, frames.Y, frames.Z
