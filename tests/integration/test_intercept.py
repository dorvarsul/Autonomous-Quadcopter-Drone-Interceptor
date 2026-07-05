"""Phase 2 integration: the wired pipeline intercepts a static target (T2.8/T2.9).

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

# Silence the intentional saturation warnings the terminal phase emits (Phase 3 tunes it).
logging.getLogger("interceptor.control").setLevel(logging.ERROR)


def _run(target, run_dir: Path, *, seconds: float = 9.0, seed: int = 0,
         start=(0.0, 0.0, 2.0)) -> Path:
    rng = RngFactory(seed)
    params = default_params()
    components = PipelineComponents.phase2_intercept(
        rng, params,
        interceptor_position_m=np.array(start, dtype=float),
        target_position_m=np.array(target, dtype=float),
    )
    orch = StubOrchestrator(components=components, params=params, seed=seed)
    result = orch.run(
        num_steps=int(seconds * constants.SIM_HZ), run_dir=run_dir, run_id="intercept"
    )
    return result.log_path


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
    log_path = _run(target, run_dir)
    min_range = _min_range(log_path)
    assert min_range <= constants.R_MISS_MAX_M  # <= 1.05 m


def test_intercept_run_is_deterministic(tmp_path: Path):
    """Identical seed + config -> byte-identical run log (reproducibility contract)."""
    a = _run([6.0, 0.0, 4.0], tmp_path / "a", seconds=3.0)
    b = _run([6.0, 0.0, 4.0], tmp_path / "b", seconds=3.0)
    assert a.read_bytes() == b.read_bytes()


def test_active_guidance_law_is_ogl(run_dir: Path):
    """The wired pipeline reports OGL as its guidance law in the run snapshot metadata."""
    rng = RngFactory(0)
    params = default_params()
    components = PipelineComponents.phase2_intercept(
        rng, params,
        interceptor_position_m=np.array([0.0, 0.0, 2.0]),
        target_position_m=np.array([6.0, 0.0, 4.0]),
    )
    assert components.guidance.name == "OGL"
    assert components.estimator.__class__.__name__ == "ExtendedKalmanFilter"
