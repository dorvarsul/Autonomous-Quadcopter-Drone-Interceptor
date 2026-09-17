"""Off-screen rendering (Role 1, Phase 1 — T1.7).

A :class:`Renderer` that captures frames **off-screen** via ``mujoco.Renderer`` and
opens **no interactive GLFW window** — automated/headless runs must never hang
(AGENTS.md → Execution Note). Frames can be saved as debug PNGs (and stitched into a
video offline). Rendering is purely an observer: it reads model/data and writes image
files; it never steps physics, so enabling or disabling it cannot change a run's
results (T1.7 DoD).

Determinism: capture is gated to every Nth sim step so frame count is a deterministic
function of step count. Disabling rendering (``enabled=False``) is a pure no-op on the
physics/log.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray

from interceptor.simulation.interfaces import Renderer

# Default capture cadence: ~30 fps preview from a 400 Hz sim (every 13th step ~ 30.8 Hz).
_DEFAULT_CAPTURE_EVERY_N_STEPS = 13


class OffscreenRenderer(Renderer):
    """Off-screen frame capture. Headless by construction (no window is ever opened)."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        enabled: bool = True,
        width: int = 640,
        height: int = 480,
        capture_every_n_steps: int = _DEFAULT_CAPTURE_EVERY_N_STEPS,
        save_dir: str | Path | None = None,
    ) -> None:
        self._model = model
        self._data = data
        self._enabled = bool(enabled)
        self._capture_every = max(1, int(capture_every_n_steps))
        self._save_dir = Path(save_dir) if save_dir is not None else None
        self._step_index = 0
        self._frame_index = 0
        self._frames: list[NDArray[np.uint8]] = []
        self._renderer: mujoco.Renderer | None = None

        if self._enabled:
            # Construction of mujoco.Renderer allocates an off-screen GL context only;
            # it does not create a visible window.
            self._renderer = mujoco.Renderer(self._model, height=height, width=width)
            if self._save_dir is not None:
                self._save_dir.mkdir(parents=True, exist_ok=True)

    @property
    def is_headless(self) -> bool:
        return True

    def render(self, sim_time_s: float) -> None:
        """Capture the current frame off-screen if rendering is enabled and due."""
        if not self._enabled or self._renderer is None:
            self._step_index += 1
            return
        if self._step_index % self._capture_every == 0:
            self._renderer.update_scene(self._data)
            frame = self._renderer.render()  # (H, W, 3) uint8, off-screen
            self._store_frame(frame)
        self._step_index += 1

    def _store_frame(self, frame: NDArray[np.uint8]) -> None:
        if self._save_dir is not None:
            self._write_png(frame, self._save_dir / f"frame_{self._frame_index:06d}.png")
        else:
            self._frames.append(np.asarray(frame, dtype=np.uint8))
        self._frame_index += 1

    @staticmethod
    def _write_png(frame: NDArray[np.uint8], path: Path) -> None:
        # matplotlib is an approved dependency; use it as a dependency-free PNG writer.
        import matplotlib.image as mpimg

        mpimg.imsave(str(path), np.asarray(frame, dtype=np.uint8))

    @property
    def captured_frames(self) -> list[NDArray[np.uint8]]:
        """In-memory frames (empty when saving straight to disk)."""
        return self._frames

    @property
    def frame_count(self) -> int:
        return self._frame_index

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
