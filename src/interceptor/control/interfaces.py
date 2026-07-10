"""Abstract interfaces owned by the Flight Control & Actuation layer (Role 4).

Four narrow contracts, kept separate so the two control loops stay distinct and run at
their own rates (AGENTS.md → "do not collapse them into a single loop"):

    CommandLimiter      : clamp guidance accel to safe bounds (SAFETY).
    OuterLoopController : accel -> target attitude (~50 Hz).
    InnerLoopController : track attitude using gyro feedback (~400 Hz).
    MotorMixer          : attitude+thrust -> four rotor RPMs (honors RPM limits).
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray

from interceptor.common.types import (
    AccelerationCommand,
    AttitudeReference,
    BodyTorqueThrustCommand,
    LimitedAccelerationCommand,
    MotorCommand,
)


class CommandLimiter(ABC):
    """Clamps an ideal acceleration request to physically safe bounds (SAFETY).

    Owned by Role 4. This is the single home of saturation handling — guidance and
    estimation must not duplicate it. Saturation is measured here for the KPI.
    """

    @abstractmethod
    def limit(self, command: AccelerationCommand) -> LimitedAccelerationCommand:
        """Return a clamped command plus its saturation flag/metric."""


class OuterLoopController(ABC):
    """~50 Hz outer loop: translate a safe acceleration into a target attitude.

    Owned by Role 4. Consumes only the limited acceleration command (Interface
    Segregation — it does not see guidance internals).
    """

    @abstractmethod
    def compute_attitude(self, command: LimitedAccelerationCommand) -> AttitudeReference:
        """Return the target roll/pitch/yaw + thrust for the inner loop to track."""


class InnerLoopController(ABC):
    """~400 Hz inner loop: track the target attitude using gyro feedback.

    Owned by Role 4. Takes the desired attitude and the measured body rates (gyro) and
    returns the **body torque + thrust** command handed to the mixer. It emits torque
    (not an attitude) because that is the physical quantity the rotors produce; the
    dedicated :class:`BodyTorqueThrustCommand` keeps torque out of angle-named fields.
    """

    @abstractmethod
    def track(
        self,
        reference: AttitudeReference,
        body_rates_rad_s: NDArray[np.float64],
    ) -> BodyTorqueThrustCommand:
        """Return the body torque + thrust command for the motor mixer."""


class MotorMixer(ABC):
    """Convert body torque + thrust into four rotor RPMs within saturation limits.

    Owned by Role 4. Guarantees outputs stay within [MOTOR_RPM_MIN, MOTOR_RPM_MAX].
    """

    @abstractmethod
    def mix(self, command: BodyTorqueThrustCommand) -> MotorCommand:
        """Return the four rotor RPM commands for the given body torque + thrust."""
