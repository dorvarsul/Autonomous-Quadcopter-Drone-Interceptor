"""Integration tests for the randomized Monte-Carlo batch harness (Phase 4 T4.4/T4.5).

Marked ``mujoco`` (each trial flies the real closed loop); headless and deterministic. These
verify the batch is reproducible and that the delivered tuning clears the mission-success gate
with margin on a fixed seed. Trial counts are kept modest so the suite stays fast while still
giving a meaningful signal.
"""

from __future__ import annotations

import logging

import pytest

from interceptor.analysis.montecarlo import run_montecarlo

pytestmark = pytest.mark.mujoco

logging.getLogger("interceptor.control").setLevel(logging.ERROR)


def test_batch_is_reproducible(tmp_path):
    """A fixed (master_seed, num_trials) reproduces identical per-trial miss distances."""
    a = run_montecarlo(6, 0, tmp_path / "a")
    b = run_montecarlo(6, 0, tmp_path / "b")
    assert [r.kpis.miss_distance_m for r in a.results] == [
        r.kpis.miss_distance_m for r in b.results
    ]
    assert a.mission_success_rate == b.mission_success_rate


def test_batch_clears_mission_success_gate(tmp_path):
    """The delivered tuning intercepts a clear majority of a fixed randomized batch.

    The full ≥90% headline is validated over larger multi-seed batches in the progress doc;
    this is a fast regression guard that the tuning has not collapsed. The threshold is set
    conservatively below the observed ~90%+ so run-to-run family sampling never flakes it.
    """
    summary = run_montecarlo(24, 0, tmp_path)
    assert summary.mission_success_rate >= 0.80, (
        f"mission success {100 * summary.mission_success_rate:.0f}% regressed "
        f"(intercepted {summary.num_intercepted}/{summary.num_trials})"
    )
