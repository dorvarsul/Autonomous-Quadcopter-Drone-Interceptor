"""Integration tests: the full 6-stage stub loop runs headless and deterministically."""

from __future__ import annotations

from pathlib import Path

import pytest

from interceptor.pipeline.orchestrator import (
    LOG_FIELDS,
    PipelineComponents,
    StubOrchestrator,
)
from interceptor.simulation.interfaces import Renderer


def test_full_loop_runs_and_logs(run_dir: Path):
    result = StubOrchestrator(seed=0).run(num_steps=400, run_dir=run_dir)
    assert result.num_steps == 400
    assert result.log_path.exists()
    assert result.snapshot_path.exists()

    lines = result.log_path.read_text(encoding="utf-8").strip().split("\n")
    assert lines[0] == ",".join(LOG_FIELDS)
    assert len(lines) == 401  # header + one row per step


def test_run_is_deterministic_byte_for_byte(tmp_path: Path):
    a = StubOrchestrator(seed=0).run(num_steps=200, run_dir=tmp_path / "a")
    b = StubOrchestrator(seed=0).run(num_steps=200, run_dir=tmp_path / "b")
    assert a.log_path.read_bytes() == b.log_path.read_bytes()


def test_headless_guarantee_rejects_windowed_renderer(run_dir: Path):
    """A non-headless renderer must fail loud rather than risk a hanging window."""

    class WindowedRenderer(Renderer):
        @property
        def is_headless(self) -> bool:
            return False

        def render(self, sim_time_s: float) -> None:  # pragma: no cover - never reached
            raise AssertionError("should not render")

    components = PipelineComponents.default_stubs()
    windowed = PipelineComponents(
        trajectory=components.trajectory,
        sensor=components.sensor,
        estimator=components.estimator,
        guidance=components.guidance,
        limiter=components.limiter,
        outer_loop=components.outer_loop,
        inner_loop=components.inner_loop,
        mixer=components.mixer,
        plant=components.plant,
        renderer=WindowedRenderer(),
    )
    with pytest.raises(RuntimeError, match="headless"):
        StubOrchestrator(components=windowed).run(num_steps=10, run_dir=run_dir)


def test_loop_closes_with_no_contract_violations(run_dir: Path):
    # If any message contract were violated mid-loop, construction would have raised.
    result = StubOrchestrator(seed=1).run(num_steps=50, run_dir=run_dir)
    assert result.final_motor_command.rotor_rpm.shape == (4,)
