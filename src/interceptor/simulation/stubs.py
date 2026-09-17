"""Pass-through Simulation stubs for the Phase 0 end-to-end skeleton.

NO physics, NO noise model — these exist only so the orchestrator can run the full
6-stage loop deterministically before Role 1 implements real MuJoCo dynamics in
Phase 1. They are intentionally trivial and clearly labelled as placeholders.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from interceptor.common import frames
from interceptor.common.types import MotorCommand, RawSensorMeasurement
from interceptor.simulation.interfaces import (
    Plant,
    Renderer,
    SensorModel,
    TargetTrajectory,
)


class StaticTargetTrajectory(TargetTrajectory):
    """A target that sits still at a fixed world position. Deterministic by design."""

    def __init__(self, position_m: NDArray[np.float64]) -> None:
        self._position = np.asarray(position_m, dtype=np.float64).copy()

    def position_at(self, sim_time_s: float) -> NDArray[np.float64]:
        return self._position.copy()


class IdealSensorModel(SensorModel):
    """Noise-free, zero-latency sensor. PLACEHOLDER for the Phase 1 noisy/delayed model.

    Computes exact range and LOS angles from the true geometry. Phase 0 only — the real
    model must add the configurable noise/latency the EKF is designed to fight.
    """

    def measure(
        self,
        interceptor_position_m: NDArray[np.float64],
        target_position_m: NDArray[np.float64],
        sim_time_s: float,
    ) -> RawSensorMeasurement:
        rel = np.asarray(target_position_m, dtype=np.float64) - np.asarray(
            interceptor_position_m, dtype=np.float64
        )
        range_m = float(np.linalg.norm(rel))
        if range_m < 1e-9:
            # Degenerate co-location: report zero range with zeroed angles, not NaN.
            return RawSensorMeasurement(
                range_m=0.0,
                los_azimuth_rad=0.0,
                los_elevation_rad=0.0,
                timestamp_s=float(sim_time_s),
                latency_s=0.0,
            )
        azimuth, elevation = frames.los_angles(rel)
        return RawSensorMeasurement(
            range_m=range_m,
            los_azimuth_rad=azimuth,
            los_elevation_rad=elevation,
            timestamp_s=float(sim_time_s),
            latency_s=0.0,
        )


class NullRenderer(Renderer):
    """Headless no-op renderer: opens no window, draws nothing. Guarantees no GLFW hang."""

    @property
    def is_headless(self) -> bool:
        return True

    def render(self, sim_time_s: float) -> None:
        return None


class StationaryPlant(Plant):
    """Trivial plant: records the last motor command, never moves. PLACEHOLDER.

    Holds the interceptor fixed and reports zero body rates, so the closed loop runs
    deterministically without any real dynamics. Phase 1 replaces this with the
    MuJoCo-stepped quadcopter.
    """

    def __init__(self, position_m: NDArray[np.float64] | None = None) -> None:
        self._position = (
            np.zeros(3, dtype=np.float64)
            if position_m is None
            else np.asarray(position_m, dtype=np.float64).copy()
        )
        self._body_rates = np.zeros(3, dtype=np.float64)
        self.last_motor_command: MotorCommand | None = None

    def step(self, motor_command: MotorCommand, dt_s: float) -> None:
        self.last_motor_command = motor_command  # absorbed; no integration in Phase 0

    @property
    def position_m(self) -> NDArray[np.float64]:
        return self._position.copy()

    @property
    def body_rates_rad_s(self) -> NDArray[np.float64]:
        return self._body_rates.copy()
