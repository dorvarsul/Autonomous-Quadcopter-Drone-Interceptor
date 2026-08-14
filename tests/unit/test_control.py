"""Unit tests for the Flight Control & Actuation layer (Role 4): limiter, loops, mixer.

Covers saturation ownership, the flatness attitude map, inner-loop step
tracking + rate separation, and mixer inversion / RPM saturation.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

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


def test_inner_loop_thrust_projection_holds_weight_support_while_tilt_lags():
    """A tilt command against a still-level body yields weight-support thrust, not excess.

    The outer loop sizes thrust ``m*|f|`` assuming the target tilt is already reached. On
    the first tick the body is still level (the tilt lags), so projecting the desired thrust
    axis onto the actual (vertical) body axis must scale the collective back to exactly the
    vertical weight support ``m*g`` — otherwise the surplus lifts the airframe (the Z-axis
    overshoot on aggressive/lateral maneuvers).
    """
    inner = AttitudePidInnerLoop(ControlParams())
    pitch = 0.5  # rad; f = [g*tan(pitch), 0, g], so |f| = g / cos(pitch) for unit mass
    thrust_cmd = constants.GRAVITY_M_S2 / np.cos(pitch)
    cmd = inner.track(AttitudeReference(0.0, pitch, 0.0, thrust_n=thrust_cmd), np.zeros(3))
    # Projected onto the level body axis -> exactly weight support, strictly below the naive
    # pass-through the outer loop requested.
    np.testing.assert_allclose(cmd.thrust_n, constants.GRAVITY_M_S2, atol=1e-6)
    assert cmd.thrust_n < thrust_cmd


def test_inner_loop_thrust_projection_restores_full_thrust_once_settled():
    """Once the tilt has caught up to the reference, the projection factor is 1 (full thrust)."""
    inner = AttitudePidInnerLoop(ControlParams())
    dt = 1.0 / constants.INNER_LOOP_HZ
    inertia = constants.QUAD_INERTIA_IYY_KG_M2
    target = 0.2
    thrust_cmd = constants.GRAVITY_M_S2 / np.cos(target)
    ref = AttitudeReference(0.0, target, 0.0, thrust_n=thrust_cmd)
    rate, cmd = 0.0, None
    for _ in range(1200):
        cmd = inner.track(ref, np.array([0.0, rate, 0.0]))
        rate += (cmd.torque_body_n_m[frames.Y] / inertia) * dt
    assert abs(cmd.thrust_n - thrust_cmd) / thrust_cmd < 1e-3  # projection -> 1 at steady state


def test_inner_loop_level_reference_passes_thrust_through():
    """With no commanded tilt the projection is exactly 1, so thrust is untouched."""
    inner = AttitudePidInnerLoop(ControlParams())
    cmd = inner.track(AttitudeReference(0.0, 0.0, 0.0, thrust_n=12.3), np.zeros(3))
    assert cmd.thrust_n == 12.3


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


def test_mixer_reports_saturation_flag():
    """Saturation is flagged so it can be counted in the command-saturation KPI, but only
    when the airframe genuinely cannot deliver the wrench.

    A feasible hover request is unsaturated. A very large roll torque against MAX-RPM rotors
    exceeds the actuator ceiling even after the attitude-priority boost, so it must report
    ``saturated=True`` — otherwise real actuator saturation would be invisible (hidden
    saturation, AGENTS.md).
    """
    mixer = QuadMotorMixer()
    hover_thrust = constants.QUAD_MASS_KG * constants.GRAVITY_M_S2
    feasible = mixer.mix(
        BodyTorqueThrustCommand(torque_body_n_m=np.zeros(3), thrust_n=hover_thrust)
    )
    assert feasible.saturated is False
    # A huge roll torque cannot be met even after boosting collective (a rotor exceeds MAX).
    over_ceiling = mixer.mix(
        BodyTorqueThrustCommand(torque_body_n_m=np.array([50.0, 0.0, 0.0]), thrust_n=hover_thrust)
    )
    assert over_ceiling.saturated is True
    assert np.all(over_ceiling.rotor_rpm <= constants.MOTOR_RPM_MAX)


def test_mixer_boost_preserves_attitude_without_flagging_saturation():
    """A moderate torque around a low collective is realized by boosting thrust, not clipping.

    Naive per-rotor clipping would drive a rotor below 0 N (losing the commanded torque) and
    flag saturation. The attitude-priority allocation instead raises the collective uniformly
    so every rotor stays >= 0 while the torque differentials are preserved exactly — using the
    RPM headroom rather than sacrificing attitude authority. The realized torque must match the
    command and the flag must stay False.
    """
    mixer = QuadMotorMixer()
    act = RotorActuatorModel()
    # Low collective (quarter weight) + a roll torque that naive allocation cannot meet.
    low_thrust = 0.25 * constants.QUAD_MASS_KG * constants.GRAVITY_M_S2
    roll_torque = 0.3  # N*m; well inside the ceiling but infeasible at this low collective
    cmd = BodyTorqueThrustCommand(
        torque_body_n_m=np.array([roll_torque, 0.0, 0.0]), thrust_n=low_thrust
    )
    out = mixer.mix(cmd)
    assert out.saturated is False
    assert np.all(out.rotor_rpm >= constants.MOTOR_RPM_MIN)
    # The realized roll torque equals the command (attitude authority preserved); only the
    # collective is higher than requested (the boost).
    realized_force, realized_torque, _ = act.body_wrench(out.rotor_rpm)
    assert realized_torque[frames.X] == pytest.approx(roll_torque, rel=1e-6)
    assert float(realized_force[frames.Z]) > low_thrust  # collective was boosted


# ------------------------------------------------------------------ angular-accel clamp
def test_inner_loop_clamps_angular_accel_on_large_step():
    """A large attitude step is rate-limited: torque never exceeds the angular-accel cap.

    Without the clamp, ``kp*error`` on a ~70 deg step demands ~360 rad/s^2 of angular
    acceleration — far more roll/pitch torque than the rotors can allocate, so the mixer
    would clamp a rotor (actuator saturation). The inner loop caps the commanded angular
    acceleration at ``max_angular_accel_rad_s2`` so the demand stays realizable.
    """
    cap = 70.0
    params = replace_control_max_ang(cap)
    inner = AttitudePidInnerLoop(params)
    # Big pitch step, body still level and not rotating -> raw command kp*error >> cap.
    cmd = inner.track(AttitudeReference(0.0, 1.2, 0.0, thrust_n=9.81), np.zeros(3))
    ang_accel = cmd.torque_body_n_m / np.array(
        [
            constants.QUAD_INERTIA_IXX_KG_M2,
            constants.QUAD_INERTIA_IYY_KG_M2,
            constants.QUAD_INERTIA_IZZ_KG_M2,
        ]
    )
    assert np.linalg.norm(ang_accel) <= cap + 1e-9
    assert np.linalg.norm(ang_accel) == pytest.approx(cap, rel=1e-6)  # clamp is active


def test_inner_loop_small_error_below_clamp_is_untouched():
    """Steady tracking (small error) stays under the cap, so the clamp does not distort it."""
    inner = AttitudePidInnerLoop(replace_control_max_ang(70.0))
    small = 0.01  # rad; kp*error = 3 rad/s^2 << 70
    cmd = inner.track(AttitudeReference(0.0, small, 0.0, thrust_n=9.81), np.zeros(3))
    expected_torque = constants.QUAD_INERTIA_IYY_KG_M2 * ControlParams().inner_pitch.kp * small
    assert cmd.torque_body_n_m[frames.Y] == pytest.approx(expected_torque, rel=1e-6)


def replace_control_max_ang(cap: float) -> ControlParams:
    """A ControlParams with only the angular-accel cap overridden (defaults elsewhere)."""
    return replace(ControlParams(), max_angular_accel_rad_s2=cap)


def _limited(accel):
    from interceptor.common.types import LimitedAccelerationCommand

    return LimitedAccelerationCommand(
        acceleration_m_s2=np.asarray(accel, dtype=float),
        saturated=False,
        saturation_magnitude_m_s2=0.0,
    )
