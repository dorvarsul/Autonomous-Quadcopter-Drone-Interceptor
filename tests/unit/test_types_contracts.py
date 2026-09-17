"""Contract tests: each message type fails loud on NaN/Inf and bad shapes."""

from __future__ import annotations

import numpy as np
import pytest

from interceptor.common import guards
from interceptor.common.types import (
    AccelerationCommand,
    AttitudeReference,
    LimitedAccelerationCommand,
    MotorCommand,
    RawSensorMeasurement,
    TargetStateEstimate,
)


def _valid_estimate() -> TargetStateEstimate:
    return TargetStateEstimate(
        relative_position_m=np.array([1.0, 2.0, 3.0]),
        relative_velocity_m_s=np.zeros(3),
        range_m=3.7416573867739413,
        los_rate_rad_s=np.zeros(2),
        angular_rates_rad_s=np.zeros(3),
        covariance=np.eye(6),
        quality=1.0,
    )


def test_raw_measurement_rejects_nan():
    with pytest.raises(guards.NumericalInstabilityError):
        RawSensorMeasurement(
            range_m=float("nan"),
            los_azimuth_rad=0.0,
            los_elevation_rad=0.0,
            timestamp_s=0.0,
            latency_s=0.0,
        )


def test_raw_measurement_rejects_negative_range():
    with pytest.raises(guards.ContractViolationError):
        RawSensorMeasurement(
            range_m=-1.0,
            los_azimuth_rad=0.0,
            los_elevation_rad=0.0,
            timestamp_s=0.0,
            latency_s=0.0,
        )


def test_acceleration_command_rejects_inf():
    with pytest.raises(guards.NumericalInstabilityError):
        AccelerationCommand(acceleration_m_s2=np.array([0.0, np.inf, 0.0]))


def test_acceleration_command_rejects_wrong_shape():
    with pytest.raises(guards.ContractViolationError):
        AccelerationCommand(acceleration_m_s2=np.array([0.0, 0.0]))


def test_motor_command_rejects_nan():
    with pytest.raises(guards.NumericalInstabilityError):
        MotorCommand(rotor_rpm=np.array([1.0, 2.0, np.nan, 4.0]))


def test_motor_command_requires_four_rotors():
    with pytest.raises(guards.ContractViolationError):
        MotorCommand(rotor_rpm=np.array([1.0, 2.0, 3.0]))


def test_estimate_quality_out_of_range_fails():
    with pytest.raises(guards.ContractViolationError):
        est = _valid_estimate()
        TargetStateEstimate(
            relative_position_m=est.relative_position_m,
            relative_velocity_m_s=est.relative_velocity_m_s,
            range_m=est.range_m,
            los_rate_rad_s=est.los_rate_rad_s,
            angular_rates_rad_s=est.angular_rates_rad_s,
            covariance=est.covariance,
            quality=1.5,
        )


def test_estimate_covariance_must_be_square():
    with pytest.raises(guards.ContractViolationError):
        TargetStateEstimate(
            relative_position_m=np.zeros(3),
            relative_velocity_m_s=np.zeros(3),
            range_m=1.0,
            los_rate_rad_s=np.zeros(2),
            angular_rates_rad_s=np.zeros(3),
            covariance=np.zeros((6, 5)),
            quality=1.0,
        )


def test_attitude_reference_rejects_negative_thrust():
    with pytest.raises(guards.ContractViolationError):
        AttitudeReference(roll_rad=0.0, pitch_rad=0.0, yaw_rad=0.0, thrust_n=-1.0)


def test_messages_are_immutable():
    cmd = AccelerationCommand(acceleration_m_s2=np.array([1.0, 2.0, 3.0]))
    with pytest.raises(ValueError):
        cmd.acceleration_m_s2[0] = 99.0  # frozen + read-only array


def test_limited_command_valid_construction():
    cmd = LimitedAccelerationCommand(
        acceleration_m_s2=np.array([1.0, 0.0, 0.0]),
        saturated=True,
        saturation_magnitude_m_s2=2.0,
    )
    assert cmd.saturated is True
    assert cmd.saturation_magnitude_m_s2 == 2.0
