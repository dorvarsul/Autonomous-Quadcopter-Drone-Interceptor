"""Each pass-through stub must satisfy its abstract interface and produce valid output."""

from __future__ import annotations

import numpy as np

from interceptor.common.types import (
    AccelerationCommand,
    AttitudeReference,
    BodyTorqueThrustCommand,
    LimitedAccelerationCommand,
    MotorCommand,
    RawSensorMeasurement,
    TargetStateEstimate,
)
from interceptor.config import constants
from interceptor.control.interfaces import (
    CommandLimiter,
    InnerLoopController,
    MotorMixer,
    OuterLoopController,
)
from interceptor.control.stubs import (
    PassThroughInnerLoop,
    PassThroughLimiter,
    PassThroughOuterLoop,
    UniformMotorMixer,
)
from interceptor.estimation.interfaces import Estimator
from interceptor.estimation.stubs import PassThroughEstimator
from interceptor.guidance.interfaces import GuidanceLaw
from interceptor.guidance.stubs import ZeroGuidance
from interceptor.simulation.interfaces import (
    Plant,
    Renderer,
    SensorModel,
    TargetTrajectory,
)
from interceptor.simulation.stubs import (
    IdealSensorModel,
    NullRenderer,
    StaticTargetTrajectory,
    StationaryPlant,
)


def test_stubs_are_instances_of_their_interfaces():
    assert isinstance(StaticTargetTrajectory(np.zeros(3)), TargetTrajectory)
    assert isinstance(IdealSensorModel(), SensorModel)
    assert isinstance(NullRenderer(), Renderer)
    assert isinstance(StationaryPlant(), Plant)
    assert isinstance(PassThroughEstimator(), Estimator)
    assert isinstance(ZeroGuidance(), GuidanceLaw)
    assert isinstance(PassThroughLimiter(), CommandLimiter)
    assert isinstance(PassThroughOuterLoop(), OuterLoopController)
    assert isinstance(PassThroughInnerLoop(), InnerLoopController)
    assert isinstance(UniformMotorMixer(), MotorMixer)


def test_null_renderer_is_headless():
    assert NullRenderer().is_headless is True


def test_ideal_sensor_recovers_geometry():
    sensor = IdealSensorModel()
    meas = sensor.measure(np.zeros(3), np.array([3.0, 4.0, 0.0]), sim_time_s=1.0)
    assert isinstance(meas, RawSensorMeasurement)
    assert meas.range_m == 5.0
    assert meas.latency_s == 0.0


def test_passthrough_estimator_inverts_to_relative_position():
    sensor = IdealSensorModel()
    target = np.array([3.0, 4.0, 12.0])
    meas = sensor.measure(np.zeros(3), target, sim_time_s=0.0)
    est = PassThroughEstimator().update(meas, dt_s=0.01)
    assert isinstance(est, TargetStateEstimate)
    np.testing.assert_allclose(est.relative_position_m, target, atol=1e-9)
    assert est.quality == 0.0  # honest: this is not a real estimate


def test_zero_guidance_requests_zero_accel():
    est = TargetStateEstimate(
        relative_position_m=np.array([1.0, 0.0, 0.0]),
        relative_velocity_m_s=np.zeros(3),
        range_m=1.0,
        los_rate_rad_s=np.zeros(2),
        angular_rates_rad_s=np.zeros(3),
        covariance=np.eye(6),
        quality=0.0,
    )
    cmd = ZeroGuidance().compute(est)
    assert isinstance(cmd, AccelerationCommand)
    np.testing.assert_array_equal(cmd.acceleration_m_s2, np.zeros(3))


def test_limiter_reports_no_saturation_for_stub():
    cmd = AccelerationCommand(acceleration_m_s2=np.array([1.0, 2.0, 3.0]))
    limited = PassThroughLimiter().limit(cmd)
    assert isinstance(limited, LimitedAccelerationCommand)
    assert limited.saturated is False


def test_mixer_outputs_four_rpms_within_limits():
    cmd_in = BodyTorqueThrustCommand(torque_body_n_m=np.zeros(3), thrust_n=0.0)
    cmd = UniformMotorMixer().mix(cmd_in)
    assert isinstance(cmd, MotorCommand)
    assert cmd.rotor_rpm.shape == (4,)
    assert np.all(cmd.rotor_rpm >= constants.MOTOR_RPM_MIN)
    assert np.all(cmd.rotor_rpm <= constants.MOTOR_RPM_MAX)


def test_inner_loop_stub_emits_zero_torque_and_passes_thrust():
    ref = AttitudeReference(roll_rad=0.1, pitch_rad=0.2, yaw_rad=0.3, thrust_n=1.0)
    out = PassThroughInnerLoop().track(ref, np.zeros(3))
    assert isinstance(out, BodyTorqueThrustCommand)
    np.testing.assert_array_equal(out.torque_body_n_m, np.zeros(3))
    assert out.thrust_n == 1.0
