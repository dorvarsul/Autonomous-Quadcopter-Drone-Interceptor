"""Off-screen renderer headless behavior.

Marked ``mujoco``: constructing the renderer needs an off-screen GL context. Asserts
the renderer is headless, captures frames off-screen, and that disabling it is a no-op.
"""

from __future__ import annotations

import numpy as np
import pytest

from interceptor.common.types import MotorCommand
from interceptor.config import constants
from interceptor.simulation.actuators import RotorActuatorModel
from interceptor.simulation.mujoco_plant import MujocoPlant
from interceptor.simulation.rendering import OffscreenRenderer

pytestmark = pytest.mark.mujoco

SIM_DT = 1.0 / constants.SIM_HZ


def _hover_command() -> MotorCommand:
    return MotorCommand(rotor_rpm=np.full(4, RotorActuatorModel().hover_rpm()))


def test_renderer_is_headless_and_captures_offscreen() -> None:
    plant = MujocoPlant(initial_position_m=np.array([0.0, 0.0, 2.0]))
    renderer = OffscreenRenderer(
        plant.model, plant.data, width=160, height=120, capture_every_n_steps=10
    )
    try:
        assert renderer.is_headless
        cmd = _hover_command()
        for k in range(100):
            plant.step(cmd, SIM_DT)
            renderer.render(k * SIM_DT)
        # 100 steps, capture every 10 -> 10 frames, each off-screen (H, W, 3).
        assert renderer.frame_count == 10
        assert renderer.captured_frames[0].shape == (120, 160, 3)
    finally:
        renderer.close()


def test_disabled_renderer_captures_nothing_and_does_not_touch_physics() -> None:
    def hover_endpoint(render_enabled: bool) -> np.ndarray:
        plant = MujocoPlant(initial_position_m=np.array([0.0, 0.0, 2.0]))
        renderer = OffscreenRenderer(plant.model, plant.data, enabled=render_enabled)
        cmd = _hover_command()
        for k in range(50):
            plant.step(cmd, SIM_DT)
            renderer.render(k * SIM_DT)
        renderer.close()
        return plant.position_m

    with_render = hover_endpoint(True)
    without_render = hover_endpoint(False)
    # Rendering is a pure observer: identical physics endpoint either way.
    np.testing.assert_array_equal(with_render, without_render)


def test_disabled_renderer_reports_zero_frames() -> None:
    plant = MujocoPlant(initial_position_m=np.array([0.0, 0.0, 2.0]))
    renderer = OffscreenRenderer(plant.model, plant.data, enabled=False)
    for k in range(20):
        renderer.render(k * SIM_DT)
    assert renderer.frame_count == 0
    renderer.close()
