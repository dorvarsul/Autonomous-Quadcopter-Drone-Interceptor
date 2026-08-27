"""Abstract interface owned by the Estimation layer (Role 2)."""

from __future__ import annotations

from abc import ABC, abstractmethod

from interceptor.common.types import RawSensorMeasurement, TargetStateEstimate


class Estimator(ABC):
    """Filters raw, noisy, delayed measurements into a clean target state estimate.

    Owned by Role 2. Consumes ONLY ``RawSensorMeasurement`` (never ground truth) and
    must expose estimate quality/covariance so Guidance can reason about uncertainty.
    The concrete implementation is the Extended Kalman Filter.
    """

    @abstractmethod
    def update(self, measurement: RawSensorMeasurement, dt_s: float) -> TargetStateEstimate:
        """Incorporate one measurement over a step ``dt_s`` and return the new estimate."""
