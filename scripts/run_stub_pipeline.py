"""Run the Phase 0 stub pipeline headlessly and write a logged, reproducible run.

This is the end-to-end demonstration that the full 6-stage loop closes on stubs:
Simulation -> Estimation -> Guidance -> Limiter -> Outer -> Inner -> Mixer -> Simulation.
Output (run log + config snapshot) lands in results/<run_id>/.

Run:  python scripts/run_stub_pipeline.py --steps 400 --seed 0
"""

from __future__ import annotations

import argparse
from pathlib import Path

from interceptor.pipeline.orchestrator import StubOrchestrator


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the Phase 0 stub pipeline (headless).")
    parser.add_argument("--steps", type=int, default=400, help="Number of sim steps to run.")
    parser.add_argument("--seed", type=int, default=0, help="Root RNG seed (determinism).")
    parser.add_argument("--run-id", default="phase0_stub", help="Run identifier / folder name.")
    parser.add_argument(
        "--results-dir", default="results", help="Base directory for run artifacts."
    )
    args = parser.parse_args()

    run_dir = Path(args.results_dir) / args.run_id
    orchestrator = StubOrchestrator(seed=args.seed)
    result = orchestrator.run(num_steps=args.steps, run_dir=run_dir, run_id=args.run_id)

    print(f"Ran {result.num_steps} steps headlessly.")
    print(f"  run log:  {result.log_path}")
    print(f"  snapshot: {result.snapshot_path}")
    print(f"  final rotor RPM: {result.final_motor_command.rotor_rpm}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
