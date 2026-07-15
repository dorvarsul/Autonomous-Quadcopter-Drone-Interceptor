"""Unit tests for the Flight Control & Actuation layer (Role 4): limiter, loops, mixer.

Covers saturation ownership, the flatness attitude map, inner-loop step
tracking + rate separation, and mixer inversion / RPM saturation.
"""

from __future__ import annotations

import numpy as np

from interceptor.common import frames
from interceptor.common.types import (
    AccelerationCommand,
    AttitudeReference,
    BodyTorqueThrustCommand,
)
from interceptor.config import constants
from interceptor.config.params import ControlParams, LimiterParams
from interceptor.control.command_limiter import AccelerationCommandLimiter
from interceptor.control.inner_loop import AttitudePidInnerLoop
from interceptor.control.motor_mixer import QuadMotorMixer
from interceptor.control.outer_loop import DifferentialFlatnessOuterLoop
from interceptor.simulation.actuators import RotorActuatorModel


# ------------------------------------------------------------------ command limiter
def test_limiter_passes_through_within_bounds():
    limiter = AccelerationCommandLimiter(LimiterParams(30.0, max_tilt_rad=0.6))
    cmd = AccelerationCommand(acceleration_m_s2=np.array([1.0, 0.5, 2.0]))
    out = limiter.limit(cmd)
    assert out.saturated is False
    assert out.saturation_magnitude_m_s2 == 0.0
    np.testing.assert_allclose(out.acceleration_m_s2, cmd.acceleration_m_s2)


def test_limiter_clamps_total_magnitude():
    limiter = AccelerationCommandLimiter(LimiterParams(10.0, max_tilt_rad=1.5))
    out = limiter.limit(AccelerationCommand(acceleration_m_s2=np.array([0.0, 0.0, 50.0])))
    assert out.saturated is True
    assert out.saturation_magnitude_m_s2 > 0.0
    assert np.linalg.norm(out.acceleration_m_s2) <= 10.0 + 1e-9


def test_limiter_clamps_horizontal_to_tilt_bound():
    # max horizontal accel = g * tan(max_tilt).
    limiter = AccelerationCommandLimiter(LimiterParams(100.0, max_tilt_rad=0.5))
    out = limiter.limit(AccelerationCommand(acceleration_m_s2=np.array([50.0, 0.0, 0.0])))
    max_h = constants.GRAVITY_M_S2 * np.tan(0.5)
    assert out.saturated is True
    horizontal = np.linalg.norm(out.acceleration_m_s2[[frames.X, frames.Y]])
    np.testing.assert_allclose(horizontal, max_h, atol=1e-6)


# ------------------------------------------------------------------ outer loop
def test_outer_loop_hover_is_level_with_weight_thrust():
    outer = DifferentialFlatnessOuterLoop(mass_kg=1.0)
    att = outer.compute_attitude(_limited([0.0, 0.0, 0.0]))
    assert abs(att.roll_rad) < 1e-9
    assert abs(att.pitch_rad) < 1e-9
    np.testing.assert_allclose(att.thrust_n, constants.GRAVITY_M_S2, atol=1e-9)


def test_outer_loop_forward_accel_pitches_forward():
    outer = DifferentialFlatnessOuterLoop(mass_kg=1.0)
    att = outer.compute_attitude(_limited([2.0, 0.0, 0.0]))
    assert att.pitch_rad > 0.0  # +X accel -> nose-forward pitch
    assert abs(att.roll_rad) < 1e-9
    np.testing.assert_allclose(att.pitch_rad, np.arctan2(2.0, constants.GRAVITY_M_S2), atol=1e-9)


def test_outer_loop_lateral_accel_rolls():
    outer = DifferentialFlatnessOuterLoop(mass_kg=1.0)
    att = outer.compute_attitude(_limited([0.0, 2.0, 0.0]))
    assert att.roll_rad < 0.0  # +Y (left) accel -> negative roll in FLU convention
    assert abs(att.pitch_rad) < 1e-9


# ------------------------------------------------------------------ inner loop
def test_inner_loop_zero_error_zero_rate_commands_zero_torque():
    inner = AttitudePidInnerLoop(ControlParams())
    cmd = inner.track(AttitudeReference(0.0, 0.0, 0.0, thrust_n=9.81), np.zeros(3))
    np.testing.assert_allclose(cmd.torque_body_n_m, np.zeros(3), atol=1e-12)
    assert cmd.thrust_n == 9.81


def test_inner_loop_damps_body_rate():
    """With no attitude error but a spinning body, torque opposes the rate (D term)."""
    inner = AttitudePidInnerLoop(ControlParams())
    cmd = inner.track(AttitudeReference(0.0, 0.0, 0.0, thrust_n=9.81), np.array([0.0, 1.0, 0.0]))
    assert cmd.torque_body_n_m[frames.Y] < 0.0  # opposes positive pitch rate


def test_inner_loop_tracks_step_pitch_reference():
    """Closed 1-DOF pitch loop: the strapdown PD drives pitch to the reference, stably."""
    inner = AttitudePidInnerLoop(ControlParams())
    inertia = constants.QUAD_INERTIA_IYY_KG_M2
    dt = 1.0 / constants.INNER_LOOP_HZ
    target = 0.2
    ref = AttitudeReference(0.0, target, 0.0, thrust_n=9.81)
    pitch, rate = 0.0, 0.0
    for _ in range(800):
        cmd = inner.track(ref, np.array([0.0, rate, 0.0]))
        ang_acc = cmd.torque_body_n_m[frames.Y] / inertia
        rate += ang_acc * dt
        pitch += rate * dt
    assert abs(pitch - target) < 0.02  # converged
    assert abs(rate) < 0.05  # settled, not oscillating


# ------------------------------------------------------------------ motor mixer
def test_mixer_inverts_the_actuator_model():
    act = RotorActuatorModel()
    mixer = QuadMotorMixer()
    rpm_true = np.array([5200.0, 4800.0, 5100.0, 4900.0])
    force, torque, _ = act.body_wrench(rpm_true)
    cmd = BodyTorqueThrustCommand(torque_body_n_m=torque, thrust_n=float(force[2]))
    rpm_rec = mixer.mix(cmd).rotor_rpm
    np.testing.assert_allclose(rpm_rec, rpm_true, atol=1e-6)


def test_mixer_pure_hover_gives_equal_rotors():
    act = RotorActuatorModel()
    mixer = QuadMotorMixer()
    hover = act.hover_rpm()
    cmd = BodyTorqueThrustCommand(
        torque_body_n_m=np.zeros(3), thrust_n=constants.QUAD_MASS_KG * constants.GRAVITY_M_S2
    )
    rpm = mixer.mix(cmd).rotor_rpm
    np.testing.assert_allclose(rpm, np.full(4, hover), atol=1e-6)


def test_mixer_saturates_within_physical_bounds():
    """An impossible thrust demand clamps to the RPM ceiling, never exceeding it."""
    mixer = QuadMotorMixer()
    cmd = BodyTorqueThrustCommand(torque_body_n_m=np.zeros(3), thrust_n=1.0e6)
    rpm = mixer.mix(cmd).rotor_rpm
    assert np.all(rpm <= constants.MOTOR_RPM_MAX)
    assert np.all(rpm >= constants.MOTOR_RPM_MIN)
    np.testing.assert_allclose(rpm, np.full(4, constants.MOTOR_RPM_MAX))


def _limited(accel):
    from interceptor.common.types import LimitedAccelerationCommand

    return LimitedAccelerationCommand(
        acceleration_m_s2=np.asarray(accel, dtype=float),
        saturated=False,
        saturation_magnitude_m_s2=0.0,
    )
