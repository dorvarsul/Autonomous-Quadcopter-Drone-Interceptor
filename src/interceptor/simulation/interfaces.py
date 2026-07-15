"""Abstract interfaces owned by the Simulation layer (Role 1).

These are the narrow contracts the rest of the pipeline depends on (Dependency
Inversion): orchestration talks to ``SensorModel`` / ``TargetTrajectory`` / ``Renderer``
/ ``Plant`` abstractions, never concrete physics classes. Real MuJoCo implementations
swap in behind these without touching downstream code.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray

from interceptor.common.types import MotorCommand, RawSensorMeasurement


class TargetTrajectory(ABC):
    """Generates the (ground-truth) target position over time, in the world frame.

    Owned by Role 1. Ground truth lives *inside* Simulation only; it must never leak to
    Estimation/Guidance/Control. The orchestrator may read it solely to drive the
    sensor model and to score KPIs (Role 5), not to feed guidance.
    """

    @abstractmethod
    def position_at(self, sim_time_s: float) -> NDArray[np.float64]:
        """Return the target world position [m] at the given sim time."""


class SensorModel(ABC):
    """Turns ground-truth relative geometry into a raw, noisy, delayed measurement.

    Owned by Role 1. This is the *only* thing Estimation is allowed to consume. Noise
    and latency are intentional and configurable; a stub may produce a clean reading,
    but the interface is the same one the real model fills with noise/latency profiles.
    """

    @abstractmethod
    def measure(
        self,
        interceptor_position_m: NDArray[np.float64],
        target_position_m: NDArray[np.float64],
        sim_time_s: float,
    ) -> RawSensorMeasurement:
        """Produce a raw sensor measurement of the target from the interceptor."""


class Renderer(ABC):
    """Visualization sink. Automated runs require a headless (off-screen) implementation.

    Owned by Role 1. The pipeline must run with a renderer that opens **no GLFW
    window** so automated/headless runs never hang (AGENTS.md → Execution Note).
    """

    @property
    @abstractmethod
    def is_headless(self) -> bool:
        """True if this renderer performs no on-screen (windowed) drawing."""

    @abstractmethod
    def render(self, sim_time_s: float) -> None:
        """Render the current frame off-screen (or no-op for a null renderer)."""


class Plant(ABC):
    """The interceptor dynamics sink: consumes motor commands, advances state.

    Owned by Role 1. Closes the loop (Motor Mixer -> Simulation). A trivial stub backs
    the skeleton loop; the MuJoCo-stepped quad is the real plant. ``body_rates_rad_s`` is the
    real-time gyro feedback the inner control loop tracks against.
    """

    @abstractmethod
    def step(self, motor_command: MotorCommand, dt_s: float) -> None:
        """Apply the rotor commands and advance the simulation by ``dt_s`` seconds."""

    @property
    @abstractmethod
    def position_m(self) -> NDArray[np.float64]:
        """Current interceptor world position [m]."""

    @property
    @abstractmethod
    def body_rates_rad_s(self) -> NDArray[np.float64]:
        """Current body angular rates [rad/s] (gyro analogue for the inner loop)."""
