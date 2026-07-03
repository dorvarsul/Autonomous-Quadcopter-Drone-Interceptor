"""Interactive replay viewer (Role 1, Phase 1 — T1.10).

Watch a *logged* interception in a live MuJoCo window. This is the project's only
sanctioned interactive window and it is **opt-in and replay-only**: it consumes an
already-written ``results/<run_id>/run_log.csv`` and drives the model bodies to the
logged poses. It never re-runs the sim, never re-steps physics, and never reads ground
truth live, so it cannot affect any result (AGENTS.md → headless rule carve-out).

Because playback is a pure consumer of a deterministic artifact, replaying the same log
twice looks identical. Real-time pacing here affects only the playback clock.

Usage (manual; never invoked by tests/CI)::

    python scripts/replay.py results/<run_id>
    python scripts/replay.py results/<run_id> --speed 0.5 --loop

The :class:`ReplaySession` core (model load + per-frame pose application) is headless
and unit-tested; only :meth:`ReplaySession.play` opens the interactive window.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray

DEFAULT_SCENE_PATH = Path("models/scene.xml")

# Log columns required to reconstruct the scene pose at each frame.
_POSE_COLUMNS = (
    "sim_time_s",
    "interceptor_x_m",
    "interceptor_y_m",
    "interceptor_z_m",
    "interceptor_qw",
    "interceptor_qx",
    "interceptor_qy",
    "interceptor_qz",
    "target_x_m",
    "target_y_m",
    "target_z_m",
)


class ReplaySession:
    """Loads a scene + a pose log and applies logged poses to the model (no physics)."""

    def __init__(self, run_dir: str | Path, scene_path: str | Path = DEFAULT_SCENE_PATH) -> None:
        run_dir = Path(run_dir)
        log_path = run_dir / "run_log.csv"
        if not log_path.exists():
            raise FileNotFoundError(f"No run_log.csv in {run_dir}.")

        self._model = mujoco.MjModel.from_xml_path(str(scene_path))
        self._data = mujoco.MjData(self._model)

        joint_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, "interceptor_root")
        self._qpos_adr = int(self._model.jnt_qposadr[joint_id])
        target_body = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, "target")
        self._target_mocap_id = int(self._model.body_mocapid[target_body])

        self._frames = self._load_frames(log_path)
        if not self._frames:
            raise ValueError(f"run_log.csv in {run_dir} has no rows to replay.")

    @staticmethod
    def _load_frames(log_path: Path) -> list[dict[str, float]]:
        with log_path.open("r", newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            missing = [c for c in _POSE_COLUMNS if c not in (reader.fieldnames or [])]
            if missing:
                raise KeyError(
                    f"run_log.csv is missing pose columns {missing}; it predates the "
                    "T1.10 pose-augmented schema and cannot be replayed."
                )
            return [{c: float(row[c]) for c in _POSE_COLUMNS} for row in reader]

    @property
    def num_frames(self) -> int:
        return len(self._frames)

    @property
    def model(self) -> mujoco.MjModel:
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        return self._data

    def apply_frame(self, index: int) -> None:
        """Set interceptor + target poses from logged frame ``index`` (no physics step)."""
        frame = self._frames[index]
        a = self._qpos_adr
        self._data.qpos[a : a + 3] = self._vec(
            frame, "interceptor_x_m", "interceptor_y_m", "interceptor_z_m"
        )
        self._data.qpos[a + 3 : a + 7] = self._quat(frame)
        self._data.mocap_pos[self._target_mocap_id] = self._vec(
            frame, "target_x_m", "target_y_m", "target_z_m"
        )
        # Pure kinematic refresh so geom world transforms match the logged pose.
        mujoco.mj_forward(self._model, self._data)

    def frame_dt(self, index: int) -> float:
        """Wall-clock spacing to the next frame from logged sim time (>= 0)."""
        if index + 1 >= self.num_frames:
            return 0.0
        return max(0.0, self._frames[index + 1]["sim_time_s"] - self._frames[index]["sim_time_s"])

    @staticmethod
    def _vec(frame: dict[str, float], *keys: str) -> NDArray[np.float64]:
        return np.array([frame[k] for k in keys], dtype=np.float64)

    @staticmethod
    def _quat(frame: dict[str, float]) -> NDArray[np.float64]:
        q = np.array(
            [
                frame["interceptor_qw"],
                frame["interceptor_qx"],
                frame["interceptor_qy"],
                frame["interceptor_qz"],
            ],
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(q))
        # A logged identity/zero quaternion (stub plant) replays as no rotation.
        return q / norm if norm > 1e-9 else np.array([1.0, 0.0, 0.0, 0.0])

    def play(self, speed: float = 1.0, loop: bool = False) -> None:  # pragma: no cover
        """Open the interactive viewer and play the logged run back in real time.

        Not covered by tests: this opens a live window and blocks. Tests exercise the
        headless core (load + ``apply_frame``) instead.
        """
        import mujoco.viewer

        speed = max(1e-3, float(speed))
        with mujoco.viewer.launch_passive(self._model, self._data) as viewer:
            while viewer.is_running():
                for i in range(self.num_frames):
                    if not viewer.is_running():
                        break
                    self.apply_frame(i)
                    viewer.sync()
                    time.sleep(self.frame_dt(i) / speed)
                if not loop:
                    break


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Replay a logged interception (opt-in viewer).")
    parser.add_argument("run_dir", help="results/<run_id> directory containing run_log.csv")
    parser.add_argument("--scene", default=str(DEFAULT_SCENE_PATH), help="MJCF scene path")
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier")
    parser.add_argument("--loop", action="store_true", help="loop the playback")
    args = parser.parse_args(argv)

    session = ReplaySession(args.run_dir, args.scene)
    print(f"Replaying {session.num_frames} frames from {args.run_dir} (speed x{args.speed}).")
    session.play(speed=args.speed, loop=args.loop)
    return 0


if __name__ == "__main__":
    sys.exit(main())
