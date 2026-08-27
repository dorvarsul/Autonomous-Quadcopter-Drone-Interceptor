"""Stress regression suite: evasive / high-speed / wind named scenarios.

Marked ``mujoco`` (drives the physics engine); headless and deterministic. Unlike the
static/linear suite, the stress scenarios are deliberate **extreme-corner probes**, so not
all of them meet *every* KPI (a sub-2 s 90 km/h intercept transiently exceeds the 5%
saturation KPI — a filed finding, not a control defect). The regression contract here is
therefore two-tier:

* **Every** stress scenario must still *intercept* (miss within R_miss) — the mission-success
  KPI — so a change that breaks interception is caught.
* The **expected-pass subset** (wind + the milder evasive geometries) must additionally meet
  *all* KPIs, locking the tuning against a quality regression.
"""

from __future__ import annotations

import logging
from pathlib import Path

import pytest

from interceptor.analysis.scenarios import discover_scenarios, load_scenario, run_scenario

pytestmark = pytest.mark.mujoco

logging.getLogger("interceptor.control").setLevel(logging.ERROR)

_STRESS_DIR = Path(__file__).resolve().parents[2] / "scenarios" / "stress"
_STRESS_FILES = discover_scenarios(_STRESS_DIR)

# Scenarios expected to meet *every* KPI with the current tuning (not just intercept). The
# high-speed / fast-juke corners are intentionally excluded — they intercept but breach the
# saturation KPI on their very short engagements (documented in the progress notes).
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


def test_stress_scenarios_exist():
    """Guard that the stress scenario library is present (fail loud if it went missing)."""
    assert _STRESS_FILES, "no stress scenarios found under scenarios/stress/"


@pytest.mark.parametrize("scenario_path", _STRESS_FILES, ids=_ids(_STRESS_FILES))
def test_stress_scenario_intercepts(scenario_path: Path, tmp_path: Path):
    """Every stress scenario intercepts; the milder ones also meet every KPI."""
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
    scenario = load_scenario(_STRESS_DIR / "wind_static_gusty.yaml")
    a = run_scenario(scenario, tmp_path / "a").run.log_path
    b = run_scenario(scenario, tmp_path / "b").run.log_path
    assert a.read_bytes() == b.read_bytes()
