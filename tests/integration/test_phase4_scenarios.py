"""Phase 4 regression suite: evasive / high-speed / wind named scenarios (T4.1-T4.3).

Marked ``mujoco`` (drives the physics engine); headless and deterministic. Unlike the Phase 3
suite, the Phase 4 scenarios are deliberate **extreme-corner stress probes**, so not all of
them meet *every* KPI (a sub-2 s 90 km/h intercept transiently exceeds the 5% saturation KPI —
a filed finding, not a control defect). The regression contract here is therefore two-tier:

* **Every** Phase 4 scenario must still *intercept* (miss within R_miss) — the mission-success
  KPI — so a change that breaks interception is caught.
* The **expected-pass subset** (wind + the milder evasive geometries) must additionally meet
  *all* KPIs, locking the Phase 4 tuning against a quality regression.
"""

from __future__ import annotations

import logging
from pathlib import Path

import pytest

from interceptor.analysis.scenarios import discover_scenarios, load_scenario, run_scenario

pytestmark = pytest.mark.mujoco

logging.getLogger("interceptor.control").setLevel(logging.ERROR)

_PHASE4_DIR = Path(__file__).resolve().parents[2] / "scenarios" / "phase4"
_PHASE4_FILES = discover_scenarios(_PHASE4_DIR)

# Scenarios expected to meet *every* KPI with the Phase 4 tuning (not just intercept). The
# high-speed / fast-juke corners are intentionally excluded — they intercept but breach the
# saturation KPI on their very short engagements (documented in docs/phase4_progress.md).
_EXPECTED_FULL_PASS = {
    "sinusoidal_vertical_bob",
    "sinusoidal_3d_spiral",
    "wind_static_moderate",
    "wind_static_gusty",
    "wind_linear_gusty",
    "wind_evasive_moderate",
}


def _ids(paths):
    return [p.stem for p in paths]


def test_phase4_scenarios_exist():
    """Guard that the Phase 4 scenario library is present (fail loud if it went missing)."""
    assert _PHASE4_FILES, "no Phase 4 scenarios found under scenarios/phase4/"


@pytest.mark.parametrize("scenario_path", _PHASE4_FILES, ids=_ids(_PHASE4_FILES))
def test_phase4_scenario_intercepts(scenario_path: Path, tmp_path: Path):
    """Every Phase 4 stress scenario intercepts; the milder ones also meet every KPI."""
    scenario = load_scenario(scenario_path)
    result = run_scenario(scenario, tmp_path)
    k = result.kpis

    assert k.miss_ok, (
        f"{scenario.name}: miss distance {k.miss_distance_m:.3f} m — failed to intercept"
    )

    if scenario.name in _EXPECTED_FULL_PASS:
        assert k.time_ok, (
            f"{scenario.name}: time-to-intercept {k.time_to_intercept_s:.2f} s exceeds KPI"
        )
        assert k.z_overshoot_ok, f"{scenario.name}: Z-overshoot {k.z_overshoot_m:.3f} m exceeds KPI"
        assert k.saturation_ok, (
            f"{scenario.name}: saturation {100 * k.command_saturation_frac:.1f}% exceeds 5%"
        )


def test_wind_scenario_is_deterministic(tmp_path: Path):
    """A gusty-wind scenario reproduces a byte-identical log — the wind RNG stream is seeded."""
    scenario = load_scenario(_PHASE4_DIR / "wind_static_gusty.yaml")
    a = run_scenario(scenario, tmp_path / "a").run.log_path
    b = run_scenario(scenario, tmp_path / "b").run.log_path
    assert a.read_bytes() == b.read_bytes()
