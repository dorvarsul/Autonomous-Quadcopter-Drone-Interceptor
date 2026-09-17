"""Sensor models (Role 1, Phase 1 — T1.5).

Radar/LiDAR/Camera analogues that turn the ground-truth relative geometry into a raw,
noisy, **delayed** :class:`~interceptor.common.types.RawSensorMeasurement` — the only
input the Estimation layer is allowed to see.
"""

from __future__ import annotations

from interceptor.simulation.sensors.noisy_sensor import NoisyDelayedSensorModel

__all__ = ["NoisyDelayedSensorModel"]
