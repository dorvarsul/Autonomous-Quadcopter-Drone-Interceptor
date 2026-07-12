"""Run a declarative scenario (or a directory of them) headlessly and report KPIs.

This is the Phase 3 Role-5 entry point — the moving-target counterpart to
``run_intercept.py``. It drives the *same* closed loop against any target trajectory
declared in a scenario YAML, measures every KPI over the terminated engagement, and prints
a pass/fail table. With ``--report`` it also writes per-scenario diagnostic plots and a KPI
summary table to ``results/phase3/`` (see :mod:`interceptor.analysis.reporting`).

Every run persists its resolved params + seed + git hash + scenario spec (reproducibility
contract), so a result is fully traceable to the file that produced it. Headless and
deterministic: a fixed seed reproduces a byte-identical run log.

Examples::

    python scripts/run_scenarios.py scenarios/static_diagonal.yaml
    python scripts/run_scenarios.py scenarios/ --report
    python scripts/run_scenarios.py scenarios/ablation --report --report-dir results/phase3/ablation
"""

from __future__ import annotations

import argparse
import contextlib
import logging
import sys
from pathlib import Path

from interceptor.analysis.scenarios import (
    ScenarioResult,
    discover_scenarios,
    load_scenario,
    run_scenario,
)


def _print_table(results: list[ScenarioResult]) -> bool:
    """Print a compact KPI pass/fail table; return True iff every scenario succeeded."""
    from interceptor.analysis.reporting import format_kpi_summary_markdown

    print(format_kpi_summary_markdown(results))
    passed = sum(r.kpis.success for r in results)
    print(f"\n{passed}/{len(results)} scenarios met all KPIs.")
    for r in results:
        if r.kpis.success:
            continue
        k = r.kpis
        fails = [
            name
            for name, ok in (
                ("miss_distance", k.miss_ok),
                ("time_to_intercept", k.time_ok),
                ("z_overshoot", k.z_overshoot_ok),
                ("command_saturation", k.saturation_ok),
            )
            if not ok
        ]
        print(f"  FAILED {r.scenario.name}: {', '.join(fails)}")
    return passed == len(results)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run declarative KPI scenarios (headless).")
    parser.add_argument("path", help="A scenario .yaml file or a directory of them.")
    parser.add_argument("--results-dir", default="results/phase3",
                        help="Base directory for per-scenario run logs/snapshots.")
    parser.add_argument("--report", action="store_true",
                        help="Also write the KPI summary table + per-scenario plots.")
    parser.add_argument("--report-dir", default=None,
                        help="Where to write the report (default: <results-dir>).")
    args = parser.parse_args(argv)

    # The KPI table uses ✅/❌; make the (possibly cp1252) console tolerate Unicode.
    with contextlib.suppress(AttributeError):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    # Per-event saturation warnings are the limiter/mixer failing loud; over a batch they
    # flood the console and the saturation KPI already quantifies them, so quiet them here
    # (script-level only — the library still logs them for single runs / tests).
    logging.getLogger("interceptor.control").setLevel(logging.ERROR)

    scenario_paths = discover_scenarios(args.path)
    if not scenario_paths:
        parser.error(f"No scenario .yaml files found at {args.path}")

    results = [run_scenario(load_scenario(p), args.results_dir) for p in scenario_paths]
    all_passed = _print_table(results)

    if args.report:
        from interceptor.analysis.reporting import write_report

        report_dir = Path(args.report_dir or args.results_dir)
        paths = write_report(results, report_dir)
        print(f"\nWrote KPI summary: {paths.summary_csv}")
        print(f"Wrote {len(paths.plot_paths)} per-scenario plots to {report_dir}")

    # Non-zero exit when a KPI failed so CI / batch callers notice a regression.
    return 0 if all_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
