"""KPI reporting & plots — Role 5, Phase 3 T3.8.

Turns a batch of :class:`~interceptor.analysis.scenarios.ScenarioResult` into a
reproducible, human-readable report: a KPI summary table (CSV + Markdown) and per-scenario
diagnostic plots, plus a b-penalty ablation comparison. Everything is written to disk; this
module never opens a window (matplotlib is forced to the headless **Agg** backend so it is
safe in CI / automated runs — AGENTS.md → no hanging GLFW/GUI).

The plots visualize exactly the quantities the KPIs grade, so a failing metric is legible
at a glance:

- **altitude vs time** overlays interceptor and target Z with the Z-overshoot band, making
  the b-penalty's effect on overshoot visible.
- **range vs time** shows the approach and marks the miss-distance KPI and closest approach.
- **command effort** plots the commanded acceleration norm with saturated frames shaded,
  so the saturation KPI is visible frame-by-frame.
"""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless: render to files only, never a window (must precede pyplot)

import matplotlib.pyplot as plt  # noqa: E402  (backend must be set first)
import numpy as np  # noqa: E402

from interceptor.analysis.kpis import RunTrace, load_run_trace  # noqa: E402
from interceptor.analysis.montecarlo import BatchSummary  # noqa: E402
from interceptor.analysis.scenarios import ScenarioResult  # noqa: E402
from interceptor.common.logging import get_git_hash  # noqa: E402
from interceptor.config import constants  # noqa: E402
from interceptor.config.params import default_params  # noqa: E402

# The columns of the KPI summary table, in a fixed order for a deterministic CSV.
_SUMMARY_COLUMNS = (
    "scenario",
    "target_class",
    "miss_distance_m",
    "miss_ok",
    "time_to_intercept_s",
    "time_ok",
    "z_overshoot_m",
    "z_overshoot_ok",
    "command_saturation_frac",
    "saturation_ok",
    "max_target_speed_kmh",
    "success",
)


def _summary_row(result: ScenarioResult) -> dict[str, object]:
    """One KPI-summary row for a scenario result."""
    k = result.kpis
    return {
        "scenario": result.scenario.name,
        "target_class": result.scenario.target_class,
        "miss_distance_m": f"{k.miss_distance_m:.4f}",
        "miss_ok": int(k.miss_ok),
        "time_to_intercept_s": f"{k.time_to_intercept_s:.3f}",
        "time_ok": int(k.time_ok),
        "z_overshoot_m": f"{k.z_overshoot_m:.4f}",
        "z_overshoot_ok": int(k.z_overshoot_ok),
        "command_saturation_frac": f"{k.command_saturation_frac:.4f}",
        "saturation_ok": int(k.saturation_ok),
        "max_target_speed_kmh": f"{k.max_target_speed_kmh:.2f}",
        "success": int(k.success),
    }


def write_kpi_summary_csv(results: list[ScenarioResult], out_path: str | Path) -> Path:
    """Write the KPI summary table as a deterministic CSV and return its path."""
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=_SUMMARY_COLUMNS, lineterminator="\n")
        writer.writeheader()
        for result in results:
            writer.writerow(_summary_row(result))
    return out_path


def format_kpi_summary_markdown(results: list[ScenarioResult]) -> str:
    """Render the KPI summary as a Markdown table (for the progress report / console)."""
    header = (
        "| Scenario | Class | R_miss (m) | t_int (s) | Z-over (m) | Sat % | Max spd (km/h) | Pass |"
    )
    sep = "| :--- | :--- | ---: | ---: | ---: | ---: | ---: | :---: |"
    lines = [header, sep]
    for result in results:
        k = result.kpis
        t_int = "—" if not k.intercepted else f"{k.time_to_intercept_s:.2f}"
        lines.append(
            f"| {result.scenario.name} | {result.scenario.target_class} "
            f"| {k.miss_distance_m:.3f} | {t_int} | {k.z_overshoot_m:.3f} "
            f"| {100 * k.command_saturation_frac:.1f} | {k.max_target_speed_kmh:.1f} "
            f"| {'✅' if k.success else '❌'} |"
        )
    return "\n".join(lines)


def _load_accel_norm(csv_path: str | Path) -> np.ndarray:
    """Read the per-frame commanded-acceleration norm column from a run log [m/s²]."""
    values: list[float] = []
    with Path(csv_path).open(newline="") as handle:
        for row in csv.DictReader(handle):
            values.append(float(row["accel_cmd_norm_m_s2"]))
    return np.asarray(values, dtype=np.float64)


def _plot_scenario(trace: RunTrace, accel_norm: np.ndarray, title: str, out_path: Path) -> None:
    """Three stacked diagnostic panels for a single run: geometry, altitude, effort."""
    range_m = trace.range_m
    intercept_idx = int(np.argmin(range_m))
    t = trace.time_s

    fig, (ax_xy, ax_alt, ax_range) = plt.subplots(3, 1, figsize=(8, 10))
    fig.suptitle(title)

    # (1) Horizontal (X-Y) geometry — the chase in plan view.
    ax_xy.plot(trace.interceptor_pos_m[:, 0], trace.interceptor_pos_m[:, 1],
               label="interceptor", color="tab:blue")
    ax_xy.plot(trace.target_pos_m[:, 0], trace.target_pos_m[:, 1],
               label="target", color="tab:orange")
    ax_xy.scatter(*trace.interceptor_pos_m[intercept_idx, :2], color="tab:blue", marker="o")
    ax_xy.scatter(*trace.target_pos_m[intercept_idx, :2], color="tab:orange", marker="x")
    ax_xy.set_xlabel("X [m]")
    ax_xy.set_ylabel("Y [m]")
    ax_xy.set_aspect("equal", "datalim")
    ax_xy.legend()
    ax_xy.set_title("Horizontal geometry")

    # (2) Altitude vs time — the Z-overshoot band above the target (b-penalty target).
    ax_alt.plot(t, trace.interceptor_pos_m[:, 2], label="interceptor Z", color="tab:blue")
    ax_alt.plot(t, trace.target_pos_m[:, 2], label="target Z", color="tab:orange")
    ax_alt.fill_between(
        t, trace.target_pos_m[:, 2], trace.interceptor_pos_m[:, 2],
        where=trace.interceptor_pos_m[:, 2] > trace.target_pos_m[:, 2],
        color="tab:red", alpha=0.2, label="Z overshoot",
    )
    ax_alt.set_xlabel("time [s]")
    ax_alt.set_ylabel("altitude Z [m]")
    ax_alt.legend()
    ax_alt.set_title("Altitude vs time")

    # (3) Range + command effort with saturated frames shaded.
    ax_range.plot(t, range_m, color="tab:green", label="range")
    ax_range.axhline(constants.R_MISS_MAX_M, ls="--", color="grey", label="R_miss KPI")
    ax_effort = ax_range.twinx()
    ax_effort.plot(t, accel_norm, color="tab:purple", alpha=0.6, label="|a_cmd|")
    _shade_saturation(ax_range, t, trace.saturated)
    ax_range.set_xlabel("time [s]")
    ax_range.set_ylabel("range [m]")
    ax_effort.set_ylabel("|a_cmd| [m/s²]")
    ax_range.legend(loc="upper right")
    ax_range.set_title("Range & command effort")

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def _shade_saturation(ax, t: np.ndarray, saturated: np.ndarray) -> None:
    """Shade the time spans where the command was saturated."""
    if not saturated.any():
        return
    ax.fill_between(t, 0, 1, where=saturated, transform=ax.get_xaxis_transform(),
                    color="tab:red", alpha=0.12, label="saturated")


@dataclass(frozen=True)
class ReportPaths:
    """Where a suite report was written."""

    summary_csv: Path
    summary_markdown: str
    plot_paths: list[Path]


def write_report(results: list[ScenarioResult], report_dir: str | Path) -> ReportPaths:
    """Write the full suite report (summary table + per-scenario plots) to ``report_dir``.

    Returns the artifact paths and the Markdown summary string (handy for pasting into the
    progress doc). Each plot is loaded straight from the scenario's own ``run_log.csv`` so
    the report is regenerable from logs alone, without re-running the physics.
    """
    report_dir = Path(report_dir)
    report_dir.mkdir(parents=True, exist_ok=True)
    summary_csv = write_kpi_summary_csv(results, report_dir / "kpi_summary.csv")
    summary_md = format_kpi_summary_markdown(results)

    plot_paths: list[Path] = []
    for result in results:
        trace = load_run_trace(result.run.log_path)
        accel_norm = _load_accel_norm(result.run.log_path)
        out = report_dir / f"{result.scenario.name}.png"
        _plot_scenario(trace, accel_norm, title=result.scenario.name, out_path=out)
        plot_paths.append(out)

    return ReportPaths(summary_csv=summary_csv, summary_markdown=summary_md, plot_paths=plot_paths)


def write_ablation_plot(
    baseline: ScenarioResult, ablated: ScenarioResult, out_path: str | Path
) -> Path:
    """Overlay altitude-vs-time for two runs (e.g. b=0.1 vs b=0) to isolate the b-penalty.

    Both runs should share the same geometry/seed and differ only in the penalty, so any
    divergence in the altitude traces is attributable to ``altitude_penalty_b`` (T3.8).
    """
    out_path = Path(out_path)
    base = load_run_trace(baseline.run.log_path)
    abl = load_run_trace(ablated.run.log_path)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(base.time_s, base.interceptor_pos_m[:, 2],
            color="tab:blue", label=f"interceptor Z (b={_b(baseline)})")
    ax.plot(abl.time_s, abl.interceptor_pos_m[:, 2],
            color="tab:red", label=f"interceptor Z (b={_b(ablated)})")
    ax.plot(base.time_s, base.target_pos_m[:, 2], color="tab:orange", ls="--", label="target Z")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("altitude Z [m]")
    ax.set_title(f"b-penalty ablation: {baseline.scenario.name}")
    ax.legend()
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return out_path


def _b(result: ScenarioResult) -> float:
    """The altitude penalty b actually used by a scenario result."""
    return result.scenario.params.guidance.altitude_penalty_b


# --------------------------------------------------------------------------------
# Monte-Carlo batch reporting (Phase 4 T4.6) — the final KPI dataset + plots.
# --------------------------------------------------------------------------------

# Per-trial KPI dataset columns, fixed order for a deterministic CSV.
_BATCH_COLUMNS = (
    "trial",
    "family",
    "wind",
    "seed",
    "intercepted",
    "miss_distance_m",
    "time_to_intercept_s",
    "time_ok",
    "z_overshoot_m",
    "z_overshoot_ok",
    "command_saturation_frac",
    "saturation_ok",
    "max_target_speed_kmh",
    "full_kpi_pass",
)


def write_batch_kpis_csv(summary: BatchSummary, out_path: str | Path) -> Path:
    """Write the per-trial KPI dataset (the Phase 4 final dataset) as a deterministic CSV."""
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=_BATCH_COLUMNS, lineterminator="\n")
        writer.writeheader()
        for r in summary.results:
            k = r.kpis
            writer.writerow({
                "trial": r.name,
                "family": r.family,
                "wind": r.wind_preset,
                "seed": r.scenario.seed,
                "intercepted": int(k.miss_ok),
                "miss_distance_m": f"{k.miss_distance_m:.4f}",
                "time_to_intercept_s": f"{k.time_to_intercept_s:.3f}",
                "time_ok": int(k.time_ok),
                "z_overshoot_m": f"{k.z_overshoot_m:.4f}",
                "z_overshoot_ok": int(k.z_overshoot_ok),
                "command_saturation_frac": f"{k.command_saturation_frac:.4f}",
                "saturation_ok": int(k.saturation_ok),
                "max_target_speed_kmh": f"{k.max_target_speed_kmh:.2f}",
                "full_kpi_pass": int(k.success),
            })
    return out_path


def _batch_manifest_dict(summary: BatchSummary) -> dict:
    """Assemble the reproducibility + headline-KPI manifest for a batch."""
    p = default_params()
    rate = summary.mission_success_rate
    max_speed = summary.max_intercepted_speed_kmh
    return {
        "master_seed": summary.master_seed,
        "num_trials": summary.num_trials,
        "git_hash": get_git_hash(),
        "tuning": {
            "max_tilt_rad": p.limiter.max_tilt_rad,
            "max_acceleration_m_s2": p.limiter.max_acceleration_m_s2,
            "reference_closing_speed_m_s": p.guidance.reference_closing_speed_m_s,
            "altitude_penalty_b": p.guidance.altitude_penalty_b,
        },
        "kpi_targets": {
            "r_miss_max_m": constants.R_MISS_MAX_M,
            "z_overshoot_max_m": constants.Z_OVERSHOOT_MAX_M,
            "cmd_saturation_max_frac": constants.CMD_SATURATION_MAX_FRAC,
            "max_target_speed_min_kmh": constants.MAX_TARGET_SPEED_MIN_KMH,
            "mission_success_min": constants.MISSION_SUCCESS_MIN,
        },
        "results": {
            "mission_success_rate": rate,
            "mission_success_pass": rate >= constants.MISSION_SUCCESS_MIN,
            "num_intercepted": summary.num_intercepted,
            "num_full_kpi_pass": summary.num_full_kpi_pass,
            "max_intercepted_speed_kmh": max_speed,
            "max_speed_pass": max_speed >= constants.MAX_TARGET_SPEED_MIN_KMH,
            "kpi_compliance": {k: list(v) for k, v in summary.kpi_compliance.items()},
            "interception_by_family": {k: list(v) for k, v in summary.by_family.items()},
            "interception_by_wind": {k: list(v) for k, v in summary.by_wind.items()},
        },
    }


def write_batch_manifest(summary: BatchSummary, out_path: str | Path) -> Path:
    """Write the batch manifest (seed + git hash + tuning + headline KPIs) as sorted JSON."""
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(_batch_manifest_dict(summary), handle, indent=2, sort_keys=True)
        handle.write("\n")
    return out_path


def format_batch_summary_markdown(summary: BatchSummary) -> str:
    """Render the batch headline KPIs + breakdowns as Markdown (for the progress doc)."""
    rate = summary.mission_success_rate
    lines = [
        f"**Mission Success Rate (interception):** {summary.num_intercepted}/{summary.num_trials} "
        f"= {100 * rate:.1f}% (KPI ≥ {100 * constants.MISSION_SUCCESS_MIN:.0f}% — "
        f"{'PASS' if rate >= constants.MISSION_SUCCESS_MIN else 'FAIL'})",
        f"**Max intercepted target speed:** {summary.max_intercepted_speed_kmh:.1f} km/h "
        f"(KPI ≥ {constants.MAX_TARGET_SPEED_MIN_KMH:.1f})",
        "",
        "| KPI | Compliance |",
        "| :--- | ---: |",
    ]
    for name, (met, total) in summary.kpi_compliance.items():
        lines.append(f"| {name} | {met}/{total} ({100 * met / total:.0f}%) |")
    lines += ["", "| Family | Interception |", "| :--- | ---: |"]
    for family, (ok, total) in summary.by_family.items():
        lines.append(f"| {family} | {ok}/{total} ({100 * ok / total:.0f}%) |")
    return "\n".join(lines)


def _plot_batch_distributions(summary: BatchSummary, report_dir: Path) -> list[Path]:
    """Four distribution panels that visualize the batch KPIs; returns the plot paths."""
    miss = np.array([r.kpis.miss_distance_m for r in summary.results])
    speed = np.array([r.kpis.max_target_speed_kmh for r in summary.results])
    sat = np.array([100 * r.kpis.command_saturation_frac for r in summary.results])
    hit = np.array([r.kpis.miss_ok for r in summary.results])

    out = report_dir / "batch_distributions.png"
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    fig.suptitle(
        f"Monte-Carlo batch (seed {summary.master_seed}, {summary.num_trials} trials) — "
        f"mission success {100 * summary.mission_success_rate:.1f}%"
    )

    # (1) Miss-distance distribution (clipped so the long tail stays legible).
    ax = axes[0, 0]
    ax.hist(np.clip(miss, 0, 5), bins=25, color="tab:green", alpha=0.8)
    ax.axvline(constants.R_MISS_MAX_M, ls="--", color="black", label="R_miss KPI")
    ax.set_xlabel("miss distance [m] (clipped at 5)")
    ax.set_ylabel("trials")
    ax.set_title("Miss-distance distribution")
    ax.legend()

    # (2) Interception vs target speed — shows fast targets are still intercepted.
    ax = axes[0, 1]
    ax.scatter(speed[hit], miss[hit], s=18, color="tab:blue", label="intercepted")
    ax.scatter(speed[~hit], np.clip(miss[~hit], 0, 5), s=18, color="tab:red", label="missed")
    ax.axhline(constants.R_MISS_MAX_M, ls="--", color="black")
    ax.axvline(constants.MAX_TARGET_SPEED_MIN_KMH, ls=":", color="grey", label="speed KPI")
    ax.set_xlabel("max target speed [km/h]")
    ax.set_ylabel("miss distance [m] (clipped)")
    ax.set_title("Miss vs target speed")
    ax.legend()

    # (3) Interception by family.
    ax = axes[1, 0]
    fam = summary.by_family
    names = list(fam)
    rates = [100 * fam[n][0] / fam[n][1] for n in names]
    ax.bar(names, rates, color="tab:purple", alpha=0.8)
    ax.axhline(100 * constants.MISSION_SUCCESS_MIN, ls="--", color="black", label="90% KPI")
    ax.set_ylabel("interception rate [%]")
    ax.set_ylim(0, 105)
    ax.set_title("Interception by target family")
    ax.legend()

    # (4) Command-saturation distribution against the 5% KPI.
    ax = axes[1, 1]
    ax.hist(np.clip(sat, 0, 40), bins=25, color="tab:orange", alpha=0.8)
    ax.axvline(100 * constants.CMD_SATURATION_MAX_FRAC, ls="--", color="black", label="5% KPI")
    ax.set_xlabel("command saturation [% of frames] (clipped at 40)")
    ax.set_ylabel("trials")
    ax.set_title("Command-saturation distribution")
    ax.legend()

    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return [out]


@dataclass(frozen=True)
class BatchReportPaths:
    """Where a Monte-Carlo batch report was written."""

    kpi_csv: Path
    manifest_json: Path
    summary_markdown: str
    plot_paths: list[Path]


def write_batch_report(summary: BatchSummary, report_dir: str | Path) -> BatchReportPaths:
    """Write the final Phase 4 batch report: per-trial dataset, manifest, and distribution plots."""
    report_dir = Path(report_dir)
    report_dir.mkdir(parents=True, exist_ok=True)
    kpi_csv = write_batch_kpis_csv(summary, report_dir / "batch_kpis.csv")
    manifest = write_batch_manifest(summary, report_dir / "batch_manifest.json")
    plots = _plot_batch_distributions(summary, report_dir)
    return BatchReportPaths(
        kpi_csv=kpi_csv,
        manifest_json=manifest,
        summary_markdown=format_batch_summary_markdown(summary),
        plot_paths=plots,
    )
