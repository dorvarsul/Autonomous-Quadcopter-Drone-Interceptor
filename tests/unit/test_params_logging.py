"""Unit tests for parameter loading/overrides and the logging/snapshot infra."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from interceptor.common.logging import RunLogger, write_run_snapshot
from interceptor.config.params import default_params, load_params


def test_default_params_serialize_to_dict():
    d = default_params().to_dict()
    assert "ekf" in d and "control" in d and "guidance" in d and "limiter" in d


def test_yaml_override_merges_partially(tmp_path: Path):
    yaml_path = tmp_path / "override.yaml"
    yaml_path.write_text(
        "limiter:\n  max_acceleration_m_s2: 12.5\n", encoding="utf-8"
    )
    params = load_params(yaml_path)
    assert params.limiter.max_acceleration_m_s2 == 12.5
    # Untouched fields keep their defaults.
    assert params.guidance.altitude_penalty_b == default_params().guidance.altitude_penalty_b


def test_unknown_param_key_fails_loud(tmp_path: Path):
    yaml_path = tmp_path / "bad.yaml"
    yaml_path.write_text("limiter:\n  not_a_real_knob: 1.0\n", encoding="utf-8")
    with pytest.raises(KeyError):
        load_params(yaml_path)


def test_run_logger_writes_header_and_rows(run_dir: Path):
    fields = ("a", "b")
    with RunLogger(run_dir, fields) as logger:
        logger.log_step({"a": 1, "b": 2.5})
        logger.log_step({"a": 3, "b": 4.0})
    content = (run_dir / "run_log.csv").read_text(encoding="utf-8")
    lines = content.strip().split("\n")
    assert lines[0] == "a,b"
    assert lines[1] == "1,2.5"


def test_run_logger_rejects_mismatched_row(run_dir: Path):
    with RunLogger(run_dir, ("a", "b")) as logger:
        with pytest.raises(KeyError):
            logger.log_step({"a": 1})  # missing "b"


def test_snapshot_records_seed_params_and_git(run_dir: Path):
    path = write_run_snapshot(
        run_dir, seed=42, params=default_params().to_dict(), metadata={"run_id": "t"}
    )
    snap = json.loads(Path(path).read_text(encoding="utf-8"))
    assert snap["seed"] == 42
    assert "git_hash" in snap
    assert snap["metadata"]["run_id"] == "t"
    assert "limiter" in snap["params"]


def test_snapshot_is_byte_identical_for_same_inputs(tmp_path: Path):
    a = write_run_snapshot(tmp_path / "a", seed=1, params=default_params().to_dict())
    b = write_run_snapshot(tmp_path / "b", seed=1, params=default_params().to_dict())
    assert Path(a).read_bytes() == Path(b).read_bytes()
