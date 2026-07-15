"""Flight Control inner loop, ~400 Hz attitude PID (Role 4).

The fast loop that makes the tilt delay *real*: it drives the actual attitude toward the
outer loop's :class:`AttitudeReference` using gyroscope feedback, emitting the body
torque the mixer realizes. It is deliberately distinct from the 50 Hz outer loop and runs
at ``INNER_LOOP_HZ`` (AGENTS.md → never collapse the two loops).

The ``track`` contract provides the desired attitude and the measured **body rates only**
(the gyro), not the current attitude — exactly what a rate gyro gives. The controller
therefore maintains its own attitude by strapdown-integrating the gyro (quaternion
kinematics, seeded level to match the airframe's initial state), then runs a PD law:

    angular_accel_desired = kp * attitude_error - kd * body_rate
    torque = inertia * angular_accel_desired

Proportional term pulls attitude toward the reference; the rate term damps using the gyro.
The resulting first-order-lag-like tilt response is what OGL's lag model anticipates — we
do not shortcut it. Gains live in ``config/params.py``.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from interceptor.common import frames, guards
from interceptor.common.types import AttitudeReference, BodyTorqueThrustCommand
from interceptor.config import constants
from interceptor.config.params import ControlParams


def _quat_multiply(a: NDArray[np.float64], b: NDArray[np.float64]) -> NDArray[np.float64]:
    """Hamilton product ``a ⊗ b`` for scalar-first quaternions ``[w, x, y, z]``."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )


def _integrate_body_rate(
    q: NDArray[np.float64], body_rate_rad_s: NDArray[np.float64], dt: float
) -> NDArray[np.float64]:
    """Advance body-to-world quaternion ``q`` by body angular velocity over ``dt``.

    Exact exponential map: ``q_{k+1} = q ⊗ [cos(θ/2), sin(θ/2)·û]`` with ``θ = |ω|·dt``.
    """
    omega = np.asarray(body_rate_rad_s, dtype=np.float64)
    angle = float(np.linalg.norm(omega)) * dt
    if angle < 1e-12:
        return frames.quat_normalize(q)
    axis = omega / np.linalg.norm(omega)
    dq = np.array(
        [np.cos(angle / 2.0), *(np.sin(angle / 2.0) * axis)], dtype=np.float64
    )
    return frames.quat_normalize(_quat_multiply(q, dq))


class AttitudePidInnerLoop:
    """Gyro-fed strapdown attitude PD producing body torque + pass-through thrust."""

    def __init__(
        self,
        params: ControlParams | None = None,
        *,
        inner_loop_hz: int = constants.INNER_LOOP_HZ,
    ) -> None:
        p = params or ControlParams()
        self._dt = 1.0 / float(inner_loop_hz)
        self._kp = np.array([p.inner_roll.kp, p.inner_pitch.kp, p.inner_yaw.kp], dtype=np.float64)
        self._kd = np.array([p.inner_roll.kd, p.inner_pitch.kd, p.inner_yaw.kd], dtype=np.float64)
        self._inertia = np.array(
            [
                constants.QUAD_INERTIA_IXX_KG_M2,
                constants.QUAD_INERTIA_IYY_KG_M2,
                constants.QUAD_INERTIA_IZZ_KG_M2,
            ],
            dtype=np.float64,
        )
        # Internal attitude estimate, seeded level (identity) to match the airframe start.
        self._q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    def track(
        self,
        reference: AttitudeReference,
        body_rates_rad_s: NDArray[np.float64],
    ) -> BodyTorqueThrustCommand:
        rates = guards.ensure_vector("body_rates_rad_s", body_rates_rad_s, 3)
        # Strapdown-integrate the gyro to update the internal attitude, then read euler.
        self._q = _integrate_body_rate(self._q, rates, self._dt)
        roll, pitch, yaw = frames.quat_to_euler(self._q)

        error = np.array(
            [
                reference.roll_rad - roll,
                reference.pitch_rad - pitch,
                _wrap_angle(reference.yaw_rad - yaw),
            ],
            dtype=np.float64,
        )
        angular_accel = self._kp * error - self._kd * rates
        torque = self._inertia * angular_accel

        guards.ensure_finite("inner_torque", torque)
        return BodyTorqueThrustCommand(torque_body_n_m=torque, thrust_n=reference.thrust_n)


def _wrap_angle(angle: float) -> float:
    """Wrap to (-pi, pi] so a yaw error never takes the long way around."""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)
