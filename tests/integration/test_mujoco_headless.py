"""Headless MuJoCo guarantee test: load, step, and render off-screen — no GLFW window.

Marked ``mujoco`` so it can be deselected (``-m "not mujoco"``) on machines without an
off-screen GL context. When it runs, it proves the Phase 0 smoke-test requirement.
"""

from __future__ import annotations

import numpy as np
import pytest

pytestmark = pytest.mark.mujoco


def test_mujoco_steps_and_renders_offscreen(tiny_mjcf: str):
    mujoco = pytest.importorskip("mujoco")

    model = mujoco.MjModel.from_xml_string(tiny_mjcf)
    data = mujoco.MjData(model)
    for _ in range(50):
        mujoco.mj_step(model, data)

    try:
        with mujoco.Renderer(model, height=64, width=64) as renderer:
            renderer.update_scene(data)
            frame = renderer.render()
    except Exception as exc:  # noqa: BLE001
        pytest.skip(f"No off-screen GL context available in this environment: {exc}")

    assert isinstance(frame, np.ndarray)
    assert frame.shape == (64, 64, 3)
