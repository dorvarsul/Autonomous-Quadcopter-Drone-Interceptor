"""Unit tests for the Monte-Carlo harness + aggregation (Phase 4 T4.4/T4.5).

These cover the *sampling* and *aggregation* logic without flying the pipeline (that needs
MuJoCo and lives in ``tests/integration/test_montecarlo.py``), so they run in the fast,
dependency-free suite. The key contracts: sampling is reproducible and always yields a valid
scenario, and mission success is measured as **interception** with the other KPIs reported
as separate compliance rates (matching the Design Review).
"""

from __future__ import annotations

import numpy as np

from interceptor.analysis.kpis import KpiRecord
from interceptor.analysis.montecarlo import (
    BatchSummary,
    TrialResult,
    sample_trial,
)
from interceptor.analysis.scenarios import scenario_from_dict


def _rng(seed: int = 0) -> np.random.Generator:
    return np.random.default_rng(np.random.SeedSequence(seed))


def test_sample_trial_is_reproducible():
    """The same index drawn from an identically-seeded RNG yields an identical spec."""
    a = sample_trial(5, _rng(0))
    b = sample_trial(5, _rng(0))
    assert a.raw_spec == b.raw_spec
    assert a.seed == b.seed == 5


def test_sample_trial_always_valid_and_ogl():
    """Every sampled trial is a valid OGL scenario across a spread of indices/families."""
    rng = _rng(0)
    families = set()
    for i in range(40):
        s = sample_trial(i, rng)
        families.add(s.target_spec["type"])
        assert s.raw_spec["guidance_law"] == "OGL"
        # A moving family must map to the moving time-to-intercept KPI bound.
        if s.target_spec["type"] == "static":
            assert s.target_class == "static"
        else:
            assert s.target_class == "moving"
    # The sampler should exercise more than one family over 40 draws.
    assert len(families) >= 2


def test_sample_trial_linear_stays_airborne():
    """The fair envelope keeps linear targets' vertical rate gentle (no ground-diving)."""
    rng = _rng(1)
    for i in range(60):
        s = sample_trial(i, rng)
        if s.target_spec["type"] == "linear":
            vz = s.target_spec["velocity_m_s"][2]
            assert -0.31 <= vz <= 0.51


def _kpi(*, miss_ok=True, time_ok=True, z_ok=True, sat_ok=True, miss=0.1, speed=10.0) -> KpiRecord:
    return KpiRecord(
        miss_distance_m=miss,
        time_to_intercept_s=3.0 if miss_ok else float("inf"),
        z_overshoot_m=0.1,
        command_saturation_frac=0.02,
        max_target_speed_kmh=speed,
        time_to_intercept_max_s=20.0,
        miss_ok=miss_ok,
        time_ok=time_ok,
        z_overshoot_ok=z_ok,
        saturation_ok=sat_ok,
    )


def _trial(family: str, wind: str, kpis: KpiRecord) -> TrialResult:
    scenario = scenario_from_dict({
        "name": f"{family}_{wind}",
        "time_limit_s": 12.0,
        "interceptor": {"start_m": [0.0, 0.0, 2.0]},
        "target": {"type": "static", "position_m": [6.0, 0.0, 4.0]},
    })
    return TrialResult(scenario=scenario, kpis=kpis, family=family, wind_preset=wind)


def test_mission_success_is_interception_based():
    """Mission success counts interceptions (miss_ok), not the all-KPI conjunction."""
    results = [
        _trial("linear", "calm", _kpi(miss_ok=True, sat_ok=False)),  # intercept, over-sat
        _trial("linear", "calm", _kpi(miss_ok=True)),
        _trial("static", "calm", _kpi(miss_ok=False, miss=4.0)),  # a genuine miss
    ]
    summary = BatchSummary(master_seed=0, num_trials=3, results=results)
    # 2 of 3 intercepted -> mission success 2/3, even though one intercept broke saturation.
    assert summary.num_intercepted == 2
    assert summary.mission_success_rate == 2 / 3
    assert summary.num_full_kpi_pass == 1  # only the clean intercept passes every KPI
    assert len(summary.kpi_exceedances()) == 1  # the over-sat intercept
    assert len(summary.failures()) == 1  # the genuine miss


def test_kpi_compliance_and_breakdowns():
    """Per-KPI compliance and family/wind breakdowns aggregate interceptions correctly."""
    results = [
        _trial("sinusoidal", "gusty", _kpi(miss_ok=True, sat_ok=False)),
        _trial("sinusoidal", "gusty", _kpi(miss_ok=True)),
        _trial("varying_speed", "calm", _kpi(miss_ok=False, miss=9.0, speed=90.0)),
    ]
    summary = BatchSummary(master_seed=0, num_trials=3, results=results)
    assert summary.kpi_compliance["miss"] == (2, 3)
    assert summary.kpi_compliance["saturation"] == (2, 3)
    assert summary.by_family["sinusoidal"] == (2, 2)
    assert summary.by_family["varying_speed"] == (0, 1)
    assert summary.by_wind["gusty"] == (2, 2)


def test_max_intercepted_speed_ignores_missed_fast_targets():
    """The certified max speed counts only *intercepted* trials, not fast misses."""
    results = [
        _trial("varying_speed", "calm", _kpi(miss_ok=True, speed=85.0)),
        _trial("varying_speed", "calm", _kpi(miss_ok=False, miss=8.0, speed=95.0)),
    ]
    summary = BatchSummary(master_seed=0, num_trials=2, results=results)
    # The 95 km/h target was missed, so the certified speed is the 85 km/h intercept.
    assert summary.max_intercepted_speed_kmh == 85.0
