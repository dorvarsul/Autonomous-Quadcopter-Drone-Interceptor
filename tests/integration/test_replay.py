"""Replay-session headless smoke test (Phase 1 — T1.10).

Exercises the replay *core* without ever opening a window: build a tiny canned pose
log, construct a ReplaySession, and apply a frame. The interactive ``play()`` window is
manual-only and deliberately not invoked here (CI must stay headless).

Marked ``mujoco`` because it loads the scene model (no GL context is needed for
``mj_forward``, but model loading lives with the other engine-backed tests).
"""

from __future__ import annotations

import csv

import numpy as np
import pytest

from interceptor.pipeline.orchestrator import LOG_FIELDS
from replay import ReplaySession

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
