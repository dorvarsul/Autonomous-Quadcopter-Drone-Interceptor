"""Motor Mixer — body torque + thrust → four rotor RPMs (Role 4).

The mixer is the exact algebraic **inverse** of the rotor model
(:class:`~interceptor.simulation.actuators.RotorActuatorModel`): given the desired body
wrench it solves for the four per-rotor thrusts, then for RPM via ``thrust = kT·rpm²``.
It reads the same shared constants as the forward model, so the two can never disagree
(DRY — one source of truth for the airframe geometry/coefficients).

"+"-config, rotor order ``[front(+X), right(-Y), back(-X), left(+Y)]``. With
``f_i = kT·rpm_i²`` the forward model is::

    thrust   T      = f0 + f1 + f2 + f3
    roll  τ_x       = arm·(f3 - f1)
    pitch τ_y       = arm·(f2 - f0)
    yaw   τ_z       = -(kQ/kT)·(f0 - f1 + f2 - f3)

Inverting for the ``f_i`` and taking ``rpm_i = sqrt(f_i / kT)`` gives the command. RPM
saturation (``[MOTOR_RPM_MIN, MOTOR_RPM_MAX]``) is the physical actuator ceiling: an
infeasible request is clamped and **logged loudly**, never silently exceeded (AGENTS.md →
respect physical limits; KPI: command saturation must stay measurable).
"""

from __future__ import annotations

import logging

import numpy as np

from interceptor.common import frames, guards
from interceptor.common.types import BodyTorqueThrustCommand, MotorCommand
from interceptor.config import constants

_log = logging.getLogger(__name__)


class QuadMotorMixer:
    """Invert the rotor model to realize a body torque + thrust within RPM limits."""

    def __init__(
        self,
        *,
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

    def mix(self, command: BodyTorqueThrustCommand) -> MotorCommand:
        torque = np.asarray(command.torque_body_n_m, dtype=np.float64)
        total_thrust = float(command.thrust_n)

        roll_term = float(torque[frames.X]) / self._arm  # f3 - f1
        pitch_term = float(torque[frames.Y]) / self._arm  # f2 - f0
        yaw_term = -float(torque[frames.Z]) * self._kt / self._kq  # f0 - f1 + f2 - f3

        # Solve the 4x4 allocation for per-rotor thrusts f = [front, right, back, left].
        a = 0.5 * (total_thrust + yaw_term)  # f0 + f2
        b = 0.5 * (total_thrust - yaw_term)  # f1 + f3
        f_front = 0.5 * (a - pitch_term)
        f_back = 0.5 * (a + pitch_term)
        f_right = 0.5 * (b - roll_term)
        f_left = 0.5 * (b + roll_term)
        thrusts = np.array([f_front, f_right, f_back, f_left], dtype=np.float64)

        # thrust = kT·rpm² -> rpm = sqrt(f/kT); a negative required thrust is infeasible.
        infeasible = thrusts < 0.0
        rpm = np.sqrt(np.clip(thrusts, 0.0, None) / self._kt)
        clamped = np.clip(rpm, self._rpm_min, self._rpm_max)

        saturated = bool(np.any(infeasible) or np.any(clamped != rpm))
        if saturated:
            _log.warning(
                "motor mixer saturation: requested per-rotor thrust=%s N, rpm=%s clamped to %s",
                np.array2string(thrusts, precision=3),
                np.array2string(rpm, precision=1),
                np.array2string(clamped, precision=1),
            )

        guards.ensure_finite("rotor_rpm", clamped)
        return MotorCommand(rotor_rpm=clamped)
