"""Run the randomized 3D Monte-Carlo trial batch headlessly and report mission success.

The Phase 4 Role-5 entry point for the **Mission Success Rate** KPI (Design Review §7): it
samples a seeded batch across the whole threat envelope (family, geometry, speed, wind),
flies each trial through the same closed loop as the named scenarios, and prints the overall
success rate with a per-family / per-wind breakdown. With ``--report`` it also writes the
final KPI dataset (per-trial CSV), a batch manifest (master seed + git hash), and the
distribution plots to the report directory.

Reproducible: a given ``--seed`` and ``--trials`` reproduce the whole batch byte-for-byte.

Examples::

    python scripts/run_montecarlo.py --trials 100 --seed 0 --report
    python scripts/run_montecarlo.py --trials 30 --seed 7 --results-dir results/phase4/mc
"""

from __future__ import annotations

import argparse
import contextlib
import logging
import sys
from pathlib import Path

from interceptor.analysis.montecarlo import BatchSummary, run_montecarlo


def _print_summary(summary: BatchSummary) -> None:
    """Print the headline mission-success KPI, per-KPI compliance, and family/wind breakdown."""
    from interceptor.config import constants

    rate = summary.mission_success_rate
    target = constants.MISSION_SUCCESS_MIN
    verdict = "PASS" if rate >= target else "FAIL"
    print(
        f"\nMission Success Rate (interception): {summary.num_intercepted}/{summary.num_trials} "
        f"= {100 * rate:.1f}%  (KPI >= {100 * target:.0f}%)  [{verdict}]"
    )
    print(
        f"Max intercepted target speed: {summary.max_intercepted_speed_kmh:.1f} km/h "
        f"(KPI >= {constants.MAX_TARGET_SPEED_MIN_KMH:.1f})"
    )

    print("\nPer-KPI compliance across the batch (met / trials):")
    for name, (met, total) in summary.kpi_compliance.items():
        print(f"  {name:<12} {met:>3}/{total:<3}  ({100 * met / total:.0f}%)")

    print("\nInterception by target family (hits / trials):")
    for family, (ok, total) in summary.by_family.items():
        print(f"  {family:<14} {ok:>3}/{total:<3}  ({100 * ok / total:.0f}%)")
    print("Interception by wind preset (hits / trials):")
    for wind, (ok, total) in summary.by_wind.items():
        print(f"  {wind:<14} {ok:>3}/{total:<3}  ({100 * ok / total:.0f}%)")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run the randomized Monte-Carlo KPI batch.")
    parser.add_argument("--trials", type=int, default=100, help="Number of randomized trials.")
    parser.add_argument("--seed", type=int, default=0, help="Batch master seed (reproducible).")
    parser.add_argument("--results-dir", default="results/phase4/montecarlo",
                        help="Base directory for per-trial run logs/snapshots.")
    parser.add_argument("--report", action="store_true",
                        help="Also write the batch KPI dataset, manifest, and plots.")
    parser.add_argument("--report-dir", default=None,
                        help="Where to write the report (default: <results-dir>).")
    args = parser.parse_args(argv)

    with contextlib.suppress(AttributeError):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    # Per-event saturation warnings flood a large batch; the saturation KPI already
    # quantifies them (script-level quieting only — the library still logs for single runs).
    logging.getLogger("interceptor.control").setLevel(logging.ERROR)

    summary = run_montecarlo(args.trials, args.seed, args.results_dir)
    _print_summary(summary)

    if args.report:
        from interceptor.analysis.reporting import write_batch_report

        report_dir = Path(args.report_dir or args.results_dir)
        paths = write_batch_report(summary, report_dir)
        print(f"\nWrote batch KPI dataset: {paths.kpi_csv}")
        print(f"Wrote batch manifest:    {paths.manifest_json}")
        print(f"Wrote {len(paths.plot_paths)} distribution plots to {report_dir}")

    from interceptor.config import constants

    return 0 if summary.mission_success_rate >= constants.MISSION_SUCCESS_MIN else 1


if __name__ == "__main__":
    raise SystemExit(main())
