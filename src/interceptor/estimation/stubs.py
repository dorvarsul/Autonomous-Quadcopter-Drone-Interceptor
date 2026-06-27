"""Pass-through Estimation stub for the Phase 0 skeleton.

NOT a filter. It reconstructs a relative position from the raw measurement geometry and
reports a fixed, honest "low confidence" quality so nothing downstream mistakes it for
a real EKF. Phase 2 replaces it with the Extended Kalman Filter.
"""

from __future__ import annotations

import numpy as np

from interceptor.common.types import RawSensorMeasurement, TargetStateEstimate
from interceptor.estimation.interfaces import Estimator

# State dimension the real EKF will carry (rel pos 3 + rel vel 3); used to size the
# placeholder covariance so the contract shape is already correct for Phase 2.
_PLACEHOLDER_STATE_DIM = 6

# Quality reported by the stub: deliberately low — this is not a real estimate.
_STUB_QUALITY = 0.0


class PassThroughEstimator(Estimator):
    """Inverts the measured (range, az, el) back into a relative position vector.

    No filtering, no velocity estimation (returns zero relative velocity and zero LOS
    rate). Exists solely to hand Guidance a correctly-typed ``TargetStateEstimate`` so
    the loop closes deterministically.
    """

    def update(self, measurement: RawSensorMeasurement, dt_s: float) -> TargetStateEstimate:
        r = measurement.range_m
        az = measurement.los_azimuth_rad
        el = measurement.los_elevation_rad
        # Spherical -> Cartesian using the frames.los_angles convention.
        horizontal = r * np.cos(el)
        rel_pos = np.array(
            [horizontal * np.cos(az), horizontal * np.sin(az), r * np.sin(el)],
            dtype=np.float64,
        )
        return TargetStateEstimate(
            relative_position_m=rel_pos,
            relative_velocity_m_s=np.zeros(3, dtype=np.float64),
            range_m=r,
            los_rate_rad_s=np.zeros(2, dtype=np.float64),
            angular_rates_rad_s=np.zeros(3, dtype=np.float64),
            covariance=np.eye(_PLACEHOLDER_STATE_DIM, dtype=np.float64),
            quality=_STUB_QUALITY,
        )
