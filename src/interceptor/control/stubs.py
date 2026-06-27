"""Pass-through Flight Control & Actuation stubs for the Phase 0 skeleton.

NO real limiting, NO PID, NO physical mixing matrix. Each stub passes its input through
in the simplest contract-satisfying way so the loop closes deterministically. Phase 2
replaces them with the real command limiter, dual-loop PID, and motor mixer.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from interceptor.common.types import (
    AccelerationCommand,
    AttitudeReference,
    LimitedAccelerationCommand,
    MotorCommand,
)
from interceptor.control.interfaces import (
    CommandLimiter,
    InnerLoopController,
    MotorMixer,
    OuterLoopController,
)

# RPM the mixer stub emits for a hovering, zero-attitude command. Mid-range placeholder
# so the value is plainly inside [MOTOR_RPM_MIN, MOTOR_RPM_MAX] without claiming physics.
_STUB_HOVER_RPM = 10000.0


class PassThroughLimiter(CommandLimiter):
    """Reports no saturation and passes the acceleration through unchanged.

    Placeholder ONLY. The real limiter clamps to safe bounds and measures saturation;
    a stub that never saturates is acceptable for Phase 0 because guidance is the
    zero-stub, so nothing ever needs clamping.
    """

    def limit(self, command: AccelerationCommand) -> LimitedAccelerationCommand:
        return LimitedAccelerationCommand(
            acceleration_m_s2=np.asarray(command.acceleration_m_s2, dtype=np.float64),
            saturated=False,
            saturation_magnitude_m_s2=0.0,
        )


class PassThroughOuterLoop(OuterLoopController):
    """Emits a level (zero-tilt) attitude with zero thrust. Placeholder."""

    def compute_attitude(self, command: LimitedAccelerationCommand) -> AttitudeReference:
        return AttitudeReference(roll_rad=0.0, pitch_rad=0.0, yaw_rad=0.0, thrust_n=0.0)


class PassThroughInnerLoop(InnerLoopController):
    """Returns the desired attitude unchanged (no rate correction). Placeholder."""

    def track(
        self,
        reference: AttitudeReference,
        body_rates_rad_s: NDArray[np.float64],
    ) -> AttitudeReference:
        return reference


class UniformMotorMixer(MotorMixer):
    """Emits four equal hover-RPM values, ignoring attitude. Placeholder.

    Stays safely within the RPM saturation band; it does not implement the real mixing
    matrix that distributes thrust/torque across rotors.
    """

    def mix(self, reference: AttitudeReference) -> MotorCommand:
        return MotorCommand(
            rotor_rpm=np.full(4, _STUB_HOVER_RPM, dtype=np.float64)
        )
