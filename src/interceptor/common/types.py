"""Pipeline data contracts: one immutable, typed message per pipeline edge.

These dataclasses are the *only* legal way data crosses a layer boundary. Each maps to
exactly one arrow in the 6-stage pipeline contract (AGENTS.md → Pipeline Contract):

    Simulation  --RawSensorMeasurement-->        Estimation
    Estimation  --TargetStateEstimate-->         Guidance
    Guidance    --AccelerationCommand-->         Command Limiter
    Cmd Limiter --LimitedAccelerationCommand-->  Flight Control (outer)
    Outer loop  --AttitudeReference-->           Flight Control (inner)
    Inner loop  --BodyTorqueThrustCommand-->     Motor Mixer
    Motor Mixer --MotorCommand-->                Simulation (actuators)

Every message is a frozen dataclass; array fields are stored read-only (``freeze``) so
a downstream layer cannot mutate a producer's data. ``__post_init__`` validates shapes
and finiteness and **fails loud** on NaN/Inf (AGENTS.md → fail loud). Units are stated
on every field; no field exists that its consuming layer does not need.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from interceptor.common import guards


@dataclass(frozen=True)
class RawSensorMeasurement:
    """Simulation -> Estimation. Raw, noisy, *delayed* sensor return.

    This is what the Estimation layer is allowed to see — never ground truth. The
    ``latency_s`` field carries the sample's age so the EKF can compensate for delay.
    """

    range_m: float  # measured interceptor->target range [m]
    los_azimuth_rad: float  # measured LOS azimuth [rad] (see frames.los_angles)
    los_elevation_rad: float  # measured LOS elevation [rad]
    timestamp_s: float  # sim time the measurement is tagged with [s]
    latency_s: float  # age/delay of this sample at delivery [s]
    # Optional measured angular rates [rad/s]; NaN-free zeros if the sensor lacks them.
    measured_rate_rad_s: NDArray[np.float64] = field(
        default_factory=lambda: np.zeros(2, dtype=np.float64)
    )

    def __post_init__(self) -> None:
        guards.ensure_finite("range_m", self.range_m)
        guards.ensure_in_range("range_m", self.range_m, 0.0, np.inf)
        guards.ensure_finite("los_azimuth_rad", self.los_azimuth_rad)
        guards.ensure_finite("los_elevation_rad", self.los_elevation_rad)
        guards.ensure_finite("timestamp_s", self.timestamp_s)
        guards.ensure_in_range("latency_s", self.latency_s, 0.0, np.inf)
        object.__setattr__(
            self,
            "measured_rate_rad_s",
            guards.freeze(guards.ensure_vector("measured_rate_rad_s", self.measured_rate_rad_s, 2)),
        )


@dataclass(frozen=True)
class TargetStateEstimate:
    """Estimation -> Guidance. Clean, filtered relative target state.

    Carries the quantities Guidance needs (relative position/velocity, range, LOS rate,
    angular rates) plus the **estimate covariance and a scalar quality** so Guidance can
    reason about uncertainty (AGENTS.md → Role 2 must expose estimate quality).
    """

    relative_position_m: NDArray[np.float64]  # target - interceptor, world frame [m]
    relative_velocity_m_s: NDArray[np.float64]  # d/dt of the above [m/s]
    range_m: float  # range to target [m]
    los_rate_rad_s: NDArray[np.float64]  # [azimuth_rate, elevation_rate] [rad/s]
    angular_rates_rad_s: NDArray[np.float64]  # body angular rates [rad/s]
    covariance: NDArray[np.float64]  # state estimate covariance [units^2], square
    quality: float  # scalar estimate quality in [0, 1]; 1 = fully confident
    # Relative acceleration (target - interceptor), world frame [m/s^2]. Feeds OGL's
    # augmented Zero-Effort-Miss term so evasive/maneuvering targets are handled. Optional
    # with a zero default so the pass-through estimator (which does not estimate
    # acceleration) still satisfies the contract; the EKF fills it in.
    relative_acceleration_m_s2: NDArray[np.float64] = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )

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
        guards.ensure_finite("range_m", self.range_m)
        guards.ensure_in_range("range_m", self.range_m, 0.0, np.inf)
        object.__setattr__(
            self,
            "los_rate_rad_s",
            guards.freeze(guards.ensure_vector("los_rate_rad_s", self.los_rate_rad_s, 2)),
        )
        object.__setattr__(
            self,
            "angular_rates_rad_s",
            guards.freeze(guards.ensure_vector("angular_rates_rad_s", self.angular_rates_rad_s, 3)),
        )
        cov = guards.ensure_finite("covariance", self.covariance)
        if cov.ndim != 2 or cov.shape[0] != cov.shape[1]:
            raise guards.ContractViolationError(
                f"covariance must be square 2-D, got shape {cov.shape}."
            )
        object.__setattr__(self, "covariance", guards.freeze(cov))
        guards.ensure_in_range("quality", self.quality, 0.0, 1.0)
        object.__setattr__(
            self,
            "relative_acceleration_m_s2",
            guards.freeze(
                guards.ensure_vector(
                    "relative_acceleration_m_s2", self.relative_acceleration_m_s2, 3
                )
            ),
        )


@dataclass(frozen=True)
class AccelerationCommand:
    """Guidance -> Command Limiter. The *ideal, unclamped* acceleration request.

    Guidance does not enforce physical limits (that is Role 4); this is the pure
    guidance-law output in the world frame.
    """

    acceleration_m_s2: NDArray[np.float64]  # desired acceleration, world frame [m/s^2]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "acceleration_m_s2",
            guards.freeze(guards.ensure_vector("acceleration_m_s2", self.acceleration_m_s2, 3)),
        )


@dataclass(frozen=True)
class LimitedAccelerationCommand:
    """Command Limiter -> Flight Control. Clamped, physically-safe acceleration.

    Saturation is *measured here* (KPI: <= 5% of flight time). ``saturated`` flags
    whether clamping occurred this step; ``saturation_magnitude_m_s2`` is how much
    acceleration was removed, for the KPI accounting.
    """

    acceleration_m_s2: NDArray[np.float64]  # clamped acceleration, world frame [m/s^2]
    saturated: bool  # True if the limiter had to clamp the request this step
    saturation_magnitude_m_s2: float  # magnitude removed by clamping [m/s^2], >= 0

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "acceleration_m_s2",
            guards.freeze(guards.ensure_vector("acceleration_m_s2", self.acceleration_m_s2, 3)),
        )
        guards.ensure_in_range(
            "saturation_magnitude_m_s2", self.saturation_magnitude_m_s2, 0.0, np.inf
        )


@dataclass(frozen=True)
class AttitudeReference:
    """Flight Control (outer) -> Motor Mixer. Target attitude + thrust.

    The outer loop turns acceleration into a desired tilt; the inner loop tracks it.
    """

    roll_rad: float  # target roll phi [rad]
    pitch_rad: float  # target pitch theta [rad]
    yaw_rad: float  # target yaw psi [rad]
    thrust_n: float  # total collective thrust [N], >= 0

    def __post_init__(self) -> None:
        guards.ensure_finite("roll_rad", self.roll_rad)
        guards.ensure_finite("pitch_rad", self.pitch_rad)
        guards.ensure_finite("yaw_rad", self.yaw_rad)
        guards.ensure_in_range("thrust_n", self.thrust_n, 0.0, np.inf)


@dataclass(frozen=True)
class BodyTorqueThrustCommand:
    """Flight Control (inner) -> Motor Mixer. Desired body torques + collective thrust.

    The inner-loop attitude/rate PID outputs the moments the airframe should generate,
    not an attitude. A dedicated message keeps torque out of the angle-named fields of
    :class:`AttitudeReference` (Clean Code -> meaningful domain names). Torque axes follow
    the body frame convention in :mod:`common.frames` (roll about +X, pitch about +Y, yaw
    about +Z); the mixer inverts the rotor model to realize them within RPM limits.
    """

    torque_body_n_m: NDArray[np.float64]  # [roll, pitch, yaw] body torque [N*m]
    thrust_n: float  # total collective thrust [N], >= 0

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "torque_body_n_m",
            guards.freeze(guards.ensure_vector("torque_body_n_m", self.torque_body_n_m, 3)),
        )
        guards.ensure_in_range("thrust_n", self.thrust_n, 0.0, np.inf)


@dataclass(frozen=True)
class MotorCommand:
    """Motor Mixer -> Simulation. Four rotor speeds [RPM].

    The mixer guarantees these sit within [MOTOR_RPM_MIN, MOTOR_RPM_MAX]; this contract
    only checks shape/finiteness (range enforcement is Role 4's job and is unit-tested
    there to avoid duplicating saturation logic).
    """

    rotor_rpm: NDArray[np.float64]  # [front, right, back, left] rotor speeds [RPM]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "rotor_rpm",
            guards.freeze(guards.ensure_vector("rotor_rpm", self.rotor_rpm, 4)),
        )
