"""Unit tests for the declarative scenario layer (Phase 3 T3.2).

These cover parsing, trajectory construction, params-override merging, and fail-loud
validation — everything *except* actually flying the pipeline (that needs MuJoCo and lives
in ``tests/integration/test_scenarios.py``). So they run in the fast, dependency-free suite.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from interceptor.analysis.scenarios import (
    build_trajectory,
    load_scenario,
    scenario_from_dict,
)
from interceptor.config import constants
from interceptor.simulation.trajectories.generators import (
    LinearTrajectory,
    StaticTrajectory,
)


def _static_spec(**overrides) -> dict:
    spec = {
        "name": "s",
        "seed": 3,
        "time_limit_s": 12.0,
        "interceptor": {"start_m": [0.0, 0.0, 2.0]},
        "target": {"type": "static", "position_m": [6.0, 0.0, 4.0]},
    }
    spec.update(overrides)
    return spec


def test_static_scenario_parses_and_defaults_static_class():
    scenario = scenario_from_dict(_static_spec())
    assert scenario.name == "s"
    assert scenario.seed == 3
    assert scenario.target_class == "static"
    assert scenario.time_to_intercept_max_s == constants.T_INT_STATIC_MAX_S
    np.testing.assert_allclose(scenario.interceptor_start_m, [0.0, 0.0, 2.0])
    assert isinstance(scenario.build_trajectory(), StaticTrajectory)


def test_linear_scenario_defaults_moving_class():
    spec = _static_spec(target={
        "type": "linear",
        "start_position_m": [10.0, 0.0, 5.0],
        "velocity_m_s": [-1.0, 0.0, 0.0],
    })
    scenario = scenario_from_dict(spec)
    assert scenario.target_class == "moving"
    assert scenario.time_to_intercept_max_s == constants.T_INT_MOVING_MAX_S
    traj = scenario.build_trajectory()
    assert isinstance(traj, LinearTrajectory)
    np.testing.assert_allclose(traj.position_at(2.0), [8.0, 0.0, 5.0])


def test_params_override_deep_merges_onto_defaults():
    """A scenario's params block overrides only the named knob; the rest keep defaults."""
    scenario = scenario_from_dict(_static_spec(params={"guidance": {"altitude_penalty_b": 0.0}}))
    assert scenario.params.guidance.altitude_penalty_b == 0.0
    # An untouched knob keeps its default (Phase 4 tuning: limiter accel default is 40.0).
    assert scenario.params.limiter.max_acceleration_m_s2 == 40.0


def test_build_trajectory_unknown_type_fails_loud():
    with pytest.raises(ValueError, match="Unknown target trajectory type"):
        build_trajectory({"type": "teleport"})


def test_missing_required_key_fails_loud():
    spec = _static_spec()
    del spec["time_limit_s"]
    with pytest.raises(KeyError, match="time_limit_s"):
        scenario_from_dict(spec)


def test_non_ogl_guidance_law_rejected():
    with pytest.raises(ValueError, match="OGL is the sole guidance law"):
        scenario_from_dict(_static_spec(guidance_law="APN"))


def test_bad_target_class_rejected():
    with pytest.raises(ValueError, match="target_class"):
        scenario_from_dict(_static_spec(target_class="orbiting"))


def test_bad_vector_shape_fails_loud():
    spec = _static_spec(interceptor={"start_m": [0.0, 0.0]})
    with pytest.raises(ValueError, match="start_m"):
        scenario_from_dict(spec)


def test_load_scenario_from_file_defaults_name_to_stem(tmp_path: Path):
    path = tmp_path / "my_case.yaml"
    path.write_text(
        "seed: 1\n"
        "time_limit_s: 10.0\n"
        "interceptor:\n  start_m: [0.0, 0.0, 2.0]\n"
        "target:\n  type: static\n  position_m: [5.0, 0.0, 5.0]\n",
        encoding="utf-8",
    )
    scenario = load_scenario(path)
    assert scenario.name == "my_case"  # filled from the file stem
