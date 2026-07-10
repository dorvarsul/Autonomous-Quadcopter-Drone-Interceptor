"""Replay-session headless smoke test (Phase 1 — T1.10).

Exercises the replay *core* without ever opening a window: build a tiny canned pose
log, construct a ReplaySession, and apply a frame. The interactive ``play()`` window is
manual-only and deliberately not invoked here (CI must stay headless).

Marked ``mujoco`` because it loads the scene model (no GL context is needed for
``mj_forward``, but model loading lives with the other engine-backed tests).
"""

from __future__ import annotations

import csv

import mujoco
import numpy as np
import pytest

from interceptor.pipeline.orchestrator import LOG_FIELDS
from replay import VIEW_INTERCEPTOR, VIEW_TOP, ReplaySession

pytestmark = pytest.mark.mujoco


def _write_canned_log(run_dir, rows: list[dict]) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    with (run_dir / "run_log.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(LOG_FIELDS), lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _row(step: int, t: float, ipos, tpos) -> dict:
    base = {name: 0.0 for name in LOG_FIELDS}
    base.update(
        step_index=step,
        sim_time_s=t,
        interceptor_x_m=ipos[0],
        interceptor_y_m=ipos[1],
        interceptor_z_m=ipos[2],
        interceptor_qw=1.0,
        target_x_m=tpos[0],
        target_y_m=tpos[1],
        target_z_m=tpos[2],
        saturated=0,
    )
    return base


def test_replay_session_loads_and_applies_frames(tmp_path) -> None:
    run_dir = tmp_path / "run"
    rows = [
        _row(0, 0.0, (0.0, 0.0, 2.0), (10.0, 0.0, 5.0)),
        _row(1, 0.0025, (0.1, 0.0, 2.0), (9.9, 0.0, 5.0)),
        _row(2, 0.005, (0.2, 0.0, 2.0), (9.8, 0.0, 5.0)),
    ]
    _write_canned_log(run_dir, rows)

    session = ReplaySession(run_dir)
    assert session.num_frames == 3

    # Applying a frame sets the interceptor + target world poses (no physics step).
    session.apply_frame(2)
    interceptor_id = session.model.body("interceptor").id
    target_id = session.model.body("target").id
    np.testing.assert_allclose(session.data.xpos[interceptor_id], [0.2, 0.0, 2.0], atol=1e-6)
    np.testing.assert_allclose(session.data.xpos[target_id], [9.8, 0.0, 5.0], atol=1e-6)
    assert session.frame_dt(0) == pytest.approx(0.0025)
    assert session.frame_dt(2) == 0.0  # last frame


def test_replay_rejects_log_without_pose_columns(tmp_path) -> None:
    run_dir = tmp_path / "run"
    run_dir.mkdir(parents=True)
    with (run_dir / "run_log.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["step_index", "sim_time_s"])
        writer.writeheader()
        writer.writerow({"step_index": 0, "sim_time_s": 0.0})
    with pytest.raises(KeyError):
        ReplaySession(run_dir)


def test_replay_missing_log_fails_loud(tmp_path) -> None:
    with pytest.raises(FileNotFoundError):
        ReplaySession(tmp_path / "does_not_exist")


def _session_over_box(tmp_path) -> ReplaySession:
    """A short run where the two drones span a known axis-aligned box."""
    run_dir = tmp_path / "run"
    rows = [
        _row(0, 0.0, (0.0, 0.0, 2.0), (10.0, 4.0, 6.0)),
        _row(1, 0.0025, (2.0, 1.0, 3.0), (8.0, 3.0, 5.0)),
        _row(2, 0.005, (4.0, 2.0, 4.0), (6.0, 2.0, 4.0)),
    ]
    _write_canned_log(run_dir, rows)
    return ReplaySession(run_dir)


def test_engagement_bounds_span_both_tracks(tmp_path) -> None:
    session = _session_over_box(tmp_path)
    center, diagonal = session.engagement_bounds()
    # Box spans x[0,10], y[0,4], z[2,6] over both drones' positions.
    np.testing.assert_allclose(center, [5.0, 2.0, 4.0], atol=1e-9)
    assert diagonal == pytest.approx(np.linalg.norm([10.0, 4.0, 4.0]))


def test_top_view_is_free_camera_framing_the_engagement(tmp_path) -> None:
    session = _session_over_box(tmp_path)
    center, diagonal = session.engagement_bounds()
    cam = mujoco.MjvCamera()
    session.configure_camera(cam, VIEW_TOP)
    assert cam.type == mujoco.mjtCamera.mjCAMERA_FREE
    np.testing.assert_allclose(cam.lookat, center, atol=1e-9)
    # Distance covers the whole engagement (>= its diagonal) so nothing leaves frame.
    assert cam.distance >= diagonal


def test_interceptor_view_tracks_the_interceptor_body(tmp_path) -> None:
    session = _session_over_box(tmp_path)
    cam = mujoco.MjvCamera()
    session.configure_camera(cam, VIEW_INTERCEPTOR)
    assert cam.type == mujoco.mjtCamera.mjCAMERA_TRACKING
    assert cam.trackbodyid == session.model.body("interceptor").id


def test_interception_frame_is_closest_approach(tmp_path) -> None:
    run_dir = tmp_path / "run"
    # Range shrinks to a minimum at frame 2, then grows (the flyby) — as a --no-terminate
    # log would. The viewer should pause on frame 2, not the receding tail.
    rows = [
        _row(0, 0.0, (0.0, 0.0, 2.0), (3.0, 0.0, 2.0)),   # range 3.0
        _row(1, 0.0025, (2.0, 0.0, 2.0), (3.0, 0.0, 2.0)),  # range 1.0
        _row(2, 0.005, (3.05, 0.0, 2.0), (3.0, 0.0, 2.0)),  # range 0.05 (closest)
        _row(3, 0.0075, (5.0, 0.0, 2.0), (3.0, 0.0, 2.0)),  # range 2.0 (flyby)
        _row(4, 0.010, (8.0, 0.0, 2.0), (3.0, 0.0, 2.0)),   # range 5.0
    ]
    _write_canned_log(run_dir, rows)
    assert ReplaySession(run_dir).interception_frame() == 2


def test_trail_state_appends_both_paths_incrementally(tmp_path) -> None:
    session = _session_over_box(tmp_path)
    scene = mujoco.MjvScene(session.model, maxgeom=1000)
    scene.ngeom = 0
    trail = session.build_trail_state()

    # No frames due before playback starts.
    assert scene.ngeom == 0
    # Segments accumulate as frames are reached; the trail only grows.
    for i in range(session.num_frames):
        before = scene.ngeom
        trail.extend_to(scene, i)
        assert scene.ngeom >= before
    # Both 3-vertex tracks contribute two segments each once fully played.
    assert scene.ngeom == 4

    # Replaying (loop) resets the cursors so the trail redraws from scratch.
    trail.reset()
    scene.ngeom = 0
    trail.extend_to(scene, session.num_frames - 1)
    assert scene.ngeom == 4
