"""Interactive replay viewer (Role 1).

Watch a *logged* interception in a live MuJoCo window. This is the project's only
sanctioned interactive window and it is **opt-in and replay-only**: it consumes an
already-written ``results/<run_id>/run_log.csv`` and drives the model bodies to the
logged poses. It never re-runs the sim, never re-steps physics, and never reads ground
truth live, so it cannot affect any result (AGENTS.md → headless rule carve-out).

Because playback is a pure consumer of a deterministic artifact, replaying the same log
twice looks identical. Real-time pacing here affects only the playback clock.

Two camera views make the engagement legible (a raw free camera loses the drones):

* ``top`` (default) — a fixed top-down **isometric** view framed to contain the entire
  engagement, so both the interceptor and the target stay in frame the whole time.
* ``interceptor`` — a **chase** camera locked onto the interceptor body, to watch the
  drone's own maneuver up close.

Both views overlay the growing **trajectory trails** of the interceptor (blue) and the
target (orange) so the geometry of the intercept is visible.

Usage (manual; never invoked by tests/CI)::

    python scripts/replay.py results/<run_id>
    python scripts/replay.py results/<run_id> --view interceptor
    python scripts/replay.py results/<run_id> --speed 0.5 --loop

The :class:`ReplaySession` core (model load + per-frame pose application + camera
framing + trail geometry) is headless and unit-tested; only :meth:`ReplaySession.play`
opens the interactive window.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from math import ceil
from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray

DEFAULT_SCENE_PATH = Path("models/scene.xml")

# Camera views the viewer can open with (see module docstring).
VIEW_TOP = "top"
VIEW_INTERCEPTOR = "interceptor"
VIEWS = (VIEW_TOP, VIEW_INTERCEPTOR)

# --- Camera geometry (tuning-only; purely cosmetic, affects no result) ------------
# Top isometric: azimuth 45° + a steep-ish elevation give the "looking down at an
# angle" framing; the distance is derived per-run from the engagement's bounding box.
_TOP_AZIMUTH_DEG = 45.0
_TOP_ELEVATION_DEG = -55.0
_TOP_DISTANCE_MARGIN = 1.35  # fraction of the diagonal to leave as padding
_TOP_MIN_DISTANCE_M = 4.0
# Interceptor chase: orbit slightly behind/above the tracked body.
_CHASE_AZIMUTH_DEG = 90.0
_CHASE_ELEVATION_DEG = -25.0
_CHASE_DISTANCE_SCALE = 0.55  # relative to engagement diagonal
_CHASE_DISTANCE_BOUNDS_M = (4.0, 15.0)

# --- Trajectory trails ------------------------------------------------------------
# Trails are drawn as connected line segments in the viewer's user scene. The log has
# thousands of frames; decimating to a bounded number of vertices keeps the overlay
# cheap without visibly changing the path.
_MAX_TRAIL_VERTICES = 600
_TRAIL_WIDTH_PX = 4.0
_INTERCEPTOR_TRAIL_RGBA = np.array([0.20, 0.55, 1.00, 1.0], dtype=np.float64)
_TARGET_TRAIL_RGBA = np.array([1.00, 0.45, 0.15, 1.0], dtype=np.float64)

# Idle refresh period while the viewer is paused on the intercept frame (keeps the
# window responsive to orbit/zoom without spinning the CPU).
_HOLD_REFRESH_S = 1.0 / 60.0

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
        self._interceptor_body_id = int(
            mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, "interceptor")
        )
        target_body = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, "target")
        self._target_mocap_id = int(self._model.body_mocapid[target_body])

        self._frames = self._load_frames(log_path)
        if not self._frames:
            raise ValueError(f"run_log.csv in {run_dir} has no rows to replay.")

        # Cached position tracks (N, 3) used for camera framing and trail overlays.
        self._interceptor_xyz = self._position_track(
            "interceptor_x_m", "interceptor_y_m", "interceptor_z_m"
        )
        self._target_xyz = self._position_track("target_x_m", "target_y_m", "target_z_m")

    @staticmethod
    def _load_frames(log_path: Path) -> list[dict[str, float]]:
        with log_path.open("r", newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            missing = [c for c in _POSE_COLUMNS if c not in (reader.fieldnames or [])]
            if missing:
                raise KeyError(
                    f"run_log.csv is missing pose columns {missing}; it predates the "
                    "pose-augmented schema and cannot be replayed."
                )
            return [{c: float(row[c]) for c in _POSE_COLUMNS} for row in reader]

    def _position_track(self, x_key: str, y_key: str, z_key: str) -> NDArray[np.float64]:
        return np.array([[f[x_key], f[y_key], f[z_key]] for f in self._frames], dtype=np.float64)

    @property
    def num_frames(self) -> int:
        return len(self._frames)

    @property
    def model(self) -> mujoco.MjModel:
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        return self._data

    @property
    def interceptor_track(self) -> NDArray[np.float64]:
        return self._interceptor_xyz

    @property
    def target_track(self) -> NDArray[np.float64]:
        return self._target_xyz

    def interception_frame(self) -> int:
        """Index of closest approach — the frame the non-looping viewer pauses on.

        With engagement-termination on this is the last frame; with a full-duration
        ``--no-terminate`` log it is the intercept point *before* the flyby, so the
        viewer freezes on the meaningful trajectory rather than the divergence tail.
        """
        ranges = np.linalg.norm(self._target_xyz - self._interceptor_xyz, axis=1)
        return int(np.argmin(ranges))

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

    # --- Camera framing (headless, unit-tested) -----------------------------------

    def engagement_bounds(self) -> tuple[NDArray[np.float64], float]:
        """Return (center, diagonal) of the box containing both full position tracks.

        ``center`` is the midpoint the camera should look at; ``diagonal`` is the
        length of the bounding box's space diagonal, used to size the view distance so
        the whole engagement fits in frame.
        """
        pts = np.vstack([self._interceptor_xyz, self._target_xyz])
        low = pts.min(axis=0)
        high = pts.max(axis=0)
        center = 0.5 * (low + high)
        diagonal = float(np.linalg.norm(high - low))
        return center, diagonal

    def configure_camera(self, cam: mujoco.MjvCamera, view: str) -> None:
        """Point ``cam`` at the engagement for the requested view (no window needed)."""
        center, diagonal = self.engagement_bounds()
        if view == VIEW_INTERCEPTOR:
            cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            cam.trackbodyid = self._interceptor_body_id
            cam.azimuth = _CHASE_AZIMUTH_DEG
            cam.elevation = _CHASE_ELEVATION_DEG
            lo, hi = _CHASE_DISTANCE_BOUNDS_M
            cam.distance = float(np.clip(diagonal * _CHASE_DISTANCE_SCALE, lo, hi))
        elif view == VIEW_TOP:
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.lookat[:] = center
            cam.azimuth = _TOP_AZIMUTH_DEG
            cam.elevation = _TOP_ELEVATION_DEG
            cam.distance = max(_TOP_MIN_DISTANCE_M, diagonal * _TOP_DISTANCE_MARGIN)
        else:  # pragma: no cover - guarded by argparse choices
            raise ValueError(f"Unknown view {view!r}; expected one of {VIEWS}.")

    # --- Trajectory trails (headless-buildable, unit-tested) ----------------------

    def _decimated_trail(
        self, track: NDArray[np.float64]
    ) -> tuple[NDArray[np.int_], NDArray[np.float64]]:
        """Decimate a position track to <= ``_MAX_TRAIL_VERTICES`` vertices.

        Returns the retained frame indices (always including the first and last frame)
        and their positions, so the overlay can grow the trail in step with playback.
        """
        n = len(track)
        if n <= _MAX_TRAIL_VERTICES:
            idx = np.arange(n)
        else:
            stride = ceil(n / _MAX_TRAIL_VERTICES)
            idx = np.arange(0, n, stride)
            if idx[-1] != n - 1:
                idx = np.append(idx, n - 1)
        return idx, track[idx]

    def build_trail_state(self) -> TrailState:
        """Precompute both decimated trails for incremental drawing during playback."""
        i_idx, i_pts = self._decimated_trail(self._interceptor_xyz)
        t_idx, t_pts = self._decimated_trail(self._target_xyz)
        return TrailState(
            (i_idx, i_pts, _INTERCEPTOR_TRAIL_RGBA),
            (t_idx, t_pts, _TARGET_TRAIL_RGBA),
        )

    def play(  # pragma: no cover
        self, view: str = VIEW_TOP, speed: float = 1.0, loop: bool = False
    ) -> None:
        """Open the interactive viewer and play the logged run back in real time.

        When ``loop`` is off the playback advances to the intercept frame and then
        **freezes there with the window still open** — the drones no longer fly off past
        interception, and the fully-drawn trajectory stays on screen so it can be
        inspected/orbited until the window is closed. ``loop`` replays continuously.

        Not covered by tests: this opens a live window and blocks. Tests exercise the
        headless core (load, ``apply_frame``, camera framing, trail decimation, intercept
        detection) instead.
        """
        import mujoco.viewer

        speed = max(1e-3, float(speed))
        trail_state = self.build_trail_state()
        hold_index = self.interception_frame()
        with mujoco.viewer.launch_passive(self._model, self._data) as viewer:
            self.configure_camera(viewer.cam, view)
            while viewer.is_running():
                trail_state.reset()
                viewer.user_scn.ngeom = 0
                for i in range(self.num_frames):
                    if not viewer.is_running():
                        break
                    self.apply_frame(i)
                    trail_state.extend_to(viewer.user_scn, i)
                    viewer.sync()
                    time.sleep(self.frame_dt(i) / speed)
                    # Not looping: stop at intercept, don't play the post-intercept flyby.
                    if not loop and i >= hold_index:
                        break
                if loop:
                    continue
                self._hold_open(viewer)
                break

    def _hold_open(self, viewer) -> None:  # pragma: no cover
        """Keep the viewer window open and interactive until the user closes it."""
        while viewer.is_running():
            viewer.sync()
            time.sleep(_HOLD_REFRESH_S)


class TrailState:
    """Incrementally appends decimated trail segments to a viewer ``user_scn``.

    Rebuilding thousands of line geoms every frame is wasteful; instead each trail keeps
    a cursor and only appends the segments whose end vertex has become due at the current
    playback frame. Total work across a full playback is O(retained vertices).
    """

    def __init__(
        self,
        *trails: tuple[NDArray[np.int_], NDArray[np.float64], NDArray[np.float64]],
    ) -> None:
        self._trails = trails
        self._cursors = [1] * len(trails)  # next vertex to connect (segment i-1 -> i)

    def reset(self) -> None:
        self._cursors = [1] * len(self._trails)

    def extend_to(self, scene: mujoco.MjvScene, frame_index: int) -> None:
        """Append any trail segments that have come due at ``frame_index``."""
        for t, (indices, points, rgba) in enumerate(self._trails):
            cursor = self._cursors[t]
            while cursor < len(indices) and indices[cursor] <= frame_index:
                if not _append_segment(scene, points[cursor - 1], points[cursor], rgba):
                    break
                cursor += 1
            self._cursors[t] = cursor


def _append_segment(
    scene: mujoco.MjvScene,
    start: NDArray[np.float64],
    end: NDArray[np.float64],
    rgba: NDArray[np.float64],
) -> bool:
    """Add one line segment geom to ``scene``; return False if the scene is full."""
    if scene.ngeom >= scene.maxgeom:
        return False
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_LINE,
        np.zeros(3),
        np.zeros(3),
        np.zeros(9),
        rgba.astype(np.float32),
    )
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, _TRAIL_WIDTH_PX, start, end)
    scene.ngeom += 1
    return True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Replay a logged interception (opt-in viewer).")
    parser.add_argument("run_dir", help="results/<run_id> directory containing run_log.csv")
    parser.add_argument("--scene", default=str(DEFAULT_SCENE_PATH), help="MJCF scene path")
    parser.add_argument(
        "--view",
        choices=VIEWS,
        default=VIEW_TOP,
        help="Camera: 'top' isometric framing both drones (default), or 'interceptor' chase.",
    )
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier")
    parser.add_argument("--loop", action="store_true", help="loop the playback")
    args = parser.parse_args(argv)

    session = ReplaySession(args.run_dir, args.scene)
    print(
        f"Replaying {session.num_frames} frames from {args.run_dir} "
        f"(view={args.view}, speed x{args.speed})."
    )
    session.play(view=args.view, speed=args.speed, loop=args.loop)
    return 0


if __name__ == "__main__":
    sys.exit(main())
