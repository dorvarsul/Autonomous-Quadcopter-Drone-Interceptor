"""Phase 3 regression suite: the static/linear scenarios meet every KPI (T3.9).

Each scenario YAML under ``scenarios/`` is flown through the real closed loop and graded by
the KPI module. A failure names the offending KPI so a future change that regresses a metric
is caught and localized immediately (not just "a run got worse"). Marked ``mujoco`` because
it drives the physics engine; headless and deterministic.

These lock the *user-approved Phase 3 tuning* (soft launch + 45 deg tilt authority): every
static and linear geometry in the spread meets miss distance, time-to-intercept, Z-overshoot,
and command saturation. The ablation scenarios (``scenarios/ablation``) are intentionally
off-spec controls (b = 0) and are excluded here.
"""

from __future__ import annotations

import logging
from pathlib import Path

import pytest

from interceptor.analysis.scenarios import discover_scenarios, load_scenario, run_scenario

pytestmark = pytest.mark.mujoco

# The saturation events are the limiter/mixer failing loud; the KPI quantifies them, so keep
# the regression output readable.
logging.getLogger("interceptor.control").setLevel(logging.ERROR)

_SCENARIO_DIR = Path(__file__).resolve().parents[2] / "scenarios"
_SCENARIO_FILES = discover_scenarios(_SCENARIO_DIR)  # top-level only; excludes ablation/


def _ids(paths):
    return [p.stem for p in paths]


@pytest.mark.parametrize("scenario_path", _SCENARIO_FILES, ids=_ids(_SCENARIO_FILES))
def test_scenario_meets_all_kpis(scenario_path: Path, tmp_path: Path):
    """Every declared static/linear scenario meets all four Phase 3 KPIs."""
    scenario = load_scenario(scenario_path)
    result = run_scenario(scenario, tmp_path)
    k = result.kpis

    # Assert each KPI separately so a failure pinpoints the offending metric.
    assert k.miss_ok, f"{scenario.name}: miss distance {k.miss_distance_m:.3f} m exceeds KPI"
    assert k.time_ok, (
        f"{scenario.name}: time-to-intercept {k.time_to_intercept_s:.2f} s exceeds "
        f"{k.time_to_intercept_max_s:.0f} s"
    )
    assert k.z_overshoot_ok, f"{scenario.name}: Z-overshoot {k.z_overshoot_m:.3f} m exceeds KPI"
    assert k.saturation_ok, (
        f"{scenario.name}: command saturation {100 * k.command_saturation_frac:.1f}% exceeds 5%"
    )
    assert k.success


def test_scenario_run_is_deterministic(tmp_path: Path):
    """Re-running a scenario reproduces a byte-identical log (reproducibility, T3.2)."""
    scenario = load_scenario(_SCENARIO_DIR / "static_diagonal.yaml")
    a = run_scenario(scenario, tmp_path / "a").run.log_path
    b = run_scenario(scenario, tmp_path / "b").run.log_path
    assert a.read_bytes() == b.read_bytes()


def test_b_penalty_ablation_still_intercepts(tmp_path: Path):
    """The b=0 ablation control still intercepts (it is a measurement control, not off-mission).

    T3.8 found Z-overshoot is negligible with or without the penalty in this control
    architecture; this guards that the ablation scenarios remain valid, flyable runs.
    """
    ablation = load_scenario(_SCENARIO_DIR / "ablation" / "static_high_b0.yaml")
    assert ablation.params.guidance.altitude_penalty_b == 0.0
    result = run_scenario(ablation, tmp_path)
    assert result.kpis.miss_ok
