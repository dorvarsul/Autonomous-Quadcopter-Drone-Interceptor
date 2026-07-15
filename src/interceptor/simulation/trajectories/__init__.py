"""Target-trajectory generators (Role 1).

Five families behind the :class:`~interceptor.simulation.interfaces.TargetTrajectory`
contract: static, linear, sinusoidal (evasive), varying-speed, and wind-affected. Each
is deterministic and reproduces an identical path for a fixed configuration/seed.
"""

from __future__ import annotations

from interceptor.simulation.trajectories.generators import (
    LinearTrajectory,
    SinusoidalTrajectory,
    StaticTrajectory,
    VaryingSpeedTrajectory,
    WindAffectedTrajectory,
)

__all__ = [
    "StaticTrajectory",
    "LinearTrajectory",
    "SinusoidalTrajectory",
    "VaryingSpeedTrajectory",
    "WindAffectedTrajectory",
]
