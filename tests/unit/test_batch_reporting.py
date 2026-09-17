"""Unit tests for the Monte-Carlo batch report writers.

Covers the deterministic per-trial KPI dataset (CSV) and the reproducibility manifest (JSON)
without rendering plots (the plotting path is exercised by the integration batch run). These
guarantee the *final dataset* is well-formed and the manifest records the headline verdicts.
"""

from __future__ import annotations

import json
from pathlib import Path

from interceptor.analysis.kpis import KpiRecord
from interceptor.analysis.montecarlo import BatchSummary, TrialResult
from interceptor.analysis.reporting import (
    format_batch_summary_markdown,
    write_batch_kpis_csv,
    write_batch_manifest,
)
from interceptor.analysis.scenarios import scenario_from_dict


def _kpi(miss_ok: bool, speed: float) -> KpiRecord:
    return KpiRecord(
        miss_distance_m=0.2 if miss_ok else 5.0,
        time_to_intercept_s=3.0 if miss_ok else float("inf"),
        z_overshoot_m=0.1,
        command_saturation_frac=0.03,
        max_target_speed_kmh=speed,
        time_to_intercept_max_s=20.0,
        miss_ok=miss_ok,
        time_ok=miss_ok,
        z_overshoot_ok=True,
        saturation_ok=True,
    )


def _summary() -> BatchSummary:
    scenario = scenario_from_dict({
        "name": "t",
        "time_limit_s": 12.0,
        "interceptor": {"start_m": [0.0, 0.0, 2.0]},
        "target": {"type": "static", "position_m": [6.0, 0.0, 4.0]},
    })
    results = [
        TrialResult(scenario, _kpi(True, 84.0), "varying_speed", "calm"),
        TrialResult(scenario, _kpi(True, 20.0), "linear", "gusty"),
        TrialResult(scenario, _kpi(False, 90.0), "varying_speed", "calm"),
    ]
    return BatchSummary(master_seed=3, num_trials=3, results=results)


def test_batch_kpis_csv_has_header_and_row_per_trial(tmp_path: Path):
    out = write_batch_kpis_csv(_summary(), tmp_path / "batch_kpis.csv")
    lines = out.read_text(encoding="utf-8").strip().splitlines()
    assert lines[0].startswith("trial,family,wind")
    assert len(lines) == 1 + 3  # header + 3 trials


def test_batch_manifest_records_headline_verdicts(tmp_path: Path):
    out = write_batch_manifest(_summary(), tmp_path / "batch_manifest.json")
    manifest = json.loads(out.read_text(encoding="utf-8"))
    assert manifest["master_seed"] == 3
    assert manifest["num_trials"] == 3
    # 2/3 intercepted -> below the 90% gate; the max intercepted speed is the 84 km/h hit.
    assert manifest["results"]["num_intercepted"] == 2
    assert manifest["results"]["mission_success_pass"] is False
    assert manifest["results"]["max_intercepted_speed_kmh"] == 84.0
    assert manifest["results"]["max_speed_pass"] is True  # 84 >= 83.6
    # The committed tuning is snapshotted for reproducibility.
    assert "max_tilt_rad" in manifest["tuning"]


def test_batch_summary_markdown_mentions_mission_success():
    md = format_batch_summary_markdown(_summary())
    assert "Mission Success Rate" in md
    assert "varying_speed" in md
