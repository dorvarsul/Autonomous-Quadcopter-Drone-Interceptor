"""Integration: the wired pipeline intercepts a static target.

Marked ``mujoco`` because it drives the real physics through the full 6-stage loop
(Simulation -> EKF -> OGL -> Limiter -> outer -> inner -> mixer -> Simulation). Headless
and deterministic; no window is opened.
"""

from __future__ import annotations

import csv
import logging
from pathlib import Path

import numpy as np
import pytest

from interceptor.common.rng import RngFactory
from interceptor.config import constants
from interceptor.config.params import default_params
from interceptor.pipeline.orchestrator import PipelineComponents, StubOrchestrator

pytestmark = pytest.mark.mujoco

# Silence the intentional saturation warnings the terminal phase emits.
logging.getLogger("interceptor.control").setLevel(logging.ERROR)


def _run(target, run_dir: Path, *, seconds: float = 9.0, seed: int = 0,
         start=(0.0, 0.0, 2.0), terminate_on_intercept: bool = False):
    rng = RngFactory(seed)
    params = default_params()
    components = PipelineComponents.intercept(
        rng, params,
        interceptor_position_m=np.array(start, dtype=float),
        target_position_m=np.array(target, dtype=float),
    )
    orch = StubOrchestrator(components=components, params=params, seed=seed)
    return orch.run(
        num_steps=int(seconds * constants.SIM_HZ), run_dir=run_dir, run_id="intercept",
        terminate_on_intercept=terminate_on_intercept,
    )


def _ranges(log_path: Path) -> np.ndarray:
    out = []
    for r in csv.DictReader(log_path.open()):
        ip = np.array([float(r[f"interceptor_{a}_m"]) for a in "xyz"])
        tp = np.array([float(r[f"target_{a}_m"]) for a in "xyz"])
        out.append(float(np.linalg.norm(tp - ip)))
    return np.array(out)


def _min_range(log_path: Path) -> float:
    best = float("inf")
    for r in csv.DictReader(log_path.open()):
        ip = np.array([float(r[f"interceptor_{a}_m"]) for a in "xyz"])
        tp = np.array([float(r[f"target_{a}_m"]) for a in "xyz"])
        best = min(best, float(np.linalg.norm(tp - ip)))
    return best


@pytest.mark.parametrize("target", [[6.0, 0.0, 4.0], [8.0, 3.0, 6.0], [5.0, -4.0, 2.0]])
def test_pipeline_intercepts_static_target(target, run_dir: Path):
    """The closed loop closes to well within the miss-distance KPI on varied geometries."""
    min_range = _min_range(_run(target, run_dir).log_path)
    assert min_range <= constants.R_MISS_MAX_M  # <= 1.05 m


def test_intercept_run_is_deterministic(tmp_path: Path):
    """Identical seed + config -> byte-identical run log (reproducibility contract)."""
    a = _run([6.0, 0.0, 4.0], tmp_path / "a", seconds=3.0).log_path
    b = _run([6.0, 0.0, 4.0], tmp_path / "b", seconds=3.0).log_path
    assert a.read_bytes() == b.read_bytes()


def test_terminate_on_intercept_stops_at_closest_approach(run_dir: Path):
    """With termination on, the run ends at closest approach — no post-intercept flyby.

    The engagement stops the moment the range starts growing again after entering the
    capture radius, so (a) it uses far fewer than the max steps, and (b) the final logged
    frame *is* the minimum range (the intercept), still well within the miss KPI.
    """
    max_steps = int(9.0 * constants.SIM_HZ)
    result = _run([8.0, 3.0, 6.0], run_dir, terminate_on_intercept=True)
    ranges = _ranges(result.log_path)

    assert result.num_steps == len(ranges)
    assert result.num_steps < max_steps  # terminated early, did not fly the full duration
    # The last frame is the closest approach (nothing recedes after it).
    assert int(np.argmin(ranges)) == len(ranges) - 1
    assert ranges[-1] <= constants.R_MISS_MAX_M
    # And it actually entered the terminal endgame before stopping.
    assert ranges[-1] <= constants.INTERCEPT_CAPTURE_RADIUS_M


def test_terminate_flag_trims_the_flyby_tail(run_dir: Path, tmp_path: Path):
    """The terminated log is a strict prefix-in-spirit: far shorter than the full run,
    and its min range matches the full run's (same approach, just no divergence tail)."""
    full = _run([8.0, 3.0, 6.0], tmp_path / "full", terminate_on_intercept=False)
    trimmed = _run([8.0, 3.0, 6.0], run_dir, terminate_on_intercept=True)
    assert trimmed.num_steps < full.num_steps
    assert _min_range(trimmed.log_path) == pytest.approx(_min_range(full.log_path), abs=1e-9)


def test_active_guidance_law_is_ogl(run_dir: Path):
    """The wired pipeline reports OGL as its guidance law in the run snapshot metadata."""
    rng = RngFactory(0)
    params = default_params()
    components = PipelineComponents.intercept(
        rng, params,
        interceptor_position_m=np.array([0.0, 0.0, 2.0]),
        target_position_m=np.array([6.0, 0.0, 4.0]),
    )
    assert components.guidance.name == "OGL"
    assert components.estimator.__class__.__name__ == "ExtendedKalmanFilter"
