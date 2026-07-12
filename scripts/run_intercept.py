"""Run the Phase 2 guided interception headlessly and write a replayable, logged run.

This is the end-to-end Phase 2 demonstration: the full 6-stage pipeline with the *real*
components — MuJoCo plant, noisy/delayed sensor, EKF, OGL, command limiter, dual-loop
control, and motor mixer — closing on a static target. Output (pose+KPI run log + config
snapshot) lands in ``results/<run_id>/`` and can be viewed with the replay tool::

    python scripts/run_intercept.py --target 8 3 6 --seconds 9
    python scripts/replay.py results/intercept

Headless and deterministic: no window is opened and a fixed seed reproduces the run
byte-for-byte.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from interceptor.analysis.kpis import compute_kpis, load_run_trace
from interceptor.common.rng import RngFactory
from interceptor.config import constants
from interceptor.config.params import default_params, load_params
from interceptor.pipeline.orchestrator import PipelineComponents, StubOrchestrator


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run the Phase 2 guided interception (headless).")
    parser.add_argument("--target", type=float, nargs=3, default=[6.0, 0.0, 4.0],
                        metavar=("X", "Y", "Z"), help="Static target world position [m].")
    parser.add_argument("--start", type=float, nargs=3, default=[0.0, 0.0, 2.0],
                        metavar=("X", "Y", "Z"), help="Interceptor start position [m].")
    parser.add_argument("--seconds", type=float, default=9.0,
                        help="Max flight duration [s] (upper bound; the run ends at "
                             "intercept unless --no-terminate).")
    parser.add_argument("--no-terminate", dest="terminate", action="store_false",
                        help="Fly the full --seconds instead of stopping at closest "
                             "approach (keeps the post-intercept flyby in the log).")
    parser.add_argument("--seed", type=int, default=0, help="Root RNG seed (determinism).")
    parser.add_argument("--run-id", default="intercept", help="Run identifier / folder name.")
    parser.add_argument("--results-dir", default="results", help="Base directory for artifacts.")
    parser.add_argument("--params", default=None, help="Optional YAML parameter override file.")
    args = parser.parse_args(argv)

    params = load_params(args.params) if args.params else default_params()
    rng = RngFactory(args.seed)
    components = PipelineComponents.phase2_intercept(
        rng, params,
        interceptor_position_m=np.array(args.start, dtype=float),
        target_position_m=np.array(args.target, dtype=float),
    )
    run_dir = Path(args.results_dir) / args.run_id
    orchestrator = StubOrchestrator(components=components, params=params, seed=args.seed)
    result = orchestrator.run(
        num_steps=int(args.seconds * constants.SIM_HZ), run_dir=run_dir, run_id=args.run_id,
        terminate_on_intercept=args.terminate,
    )

    # Reuse the Phase 3 KPI module as the single source of truth for miss distance / t_int.
    trace = load_run_trace(result.log_path)
    kpi = compute_kpis(trace, time_to_intercept_max_s=constants.T_INT_STATIC_MAX_S)
    min_range = kpi.miss_distance_m
    t_int = float(trace.time_s[int(np.argmin(trace.range_m))])
    hit = "HIT" if kpi.miss_ok else "miss"
    print(f"Ran {result.num_steps} steps headlessly against target {args.target}.")
    print(f"  min miss distance: {min_range:.3f} m at t={t_int:.2f} s  [{hit} vs "
          f"R_miss <= {constants.R_MISS_MAX_M} m]")
    print(f"  run log:  {result.log_path}")
    print(f"  snapshot: {result.snapshot_path}")
    print(f"View it with:  python scripts/replay.py {run_dir}")
    print("  (top isometric by default; chase cam: --view interceptor)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
