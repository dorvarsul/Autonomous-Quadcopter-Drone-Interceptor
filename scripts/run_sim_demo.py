"""Simulation demo (Role 1) — produce a replayable real-physics pose log.

Drives the **real** MuJoCo plant (hovering interceptor) alongside a weaving target
trajectory and writes a pose-augmented run log to ``results/<run_id>/``. It runs no
guidance/control, so this is a flight/sensor demo, not a guided
interception — but it exercises the whole Simulation layer end-to-end (plant, actuator,
trajectory, optional wind) and yields a concrete artifact to view with::

    python scripts/run_sim_demo.py --run-id sim_demo --seconds 8 --wind moderate
    python scripts/replay.py results/sim_demo

Headless and deterministic: no window is opened here (rendering optional/off-screen),
and a fixed seed reproduces the log byte-for-byte.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from interceptor.common.logging import RunLogger, write_run_snapshot
from interceptor.common.rng import RngFactory
from interceptor.config import constants
from interceptor.config.params import default_params
from interceptor.pipeline.orchestrator import LOG_FIELDS
from interceptor.simulation.actuators import RotorActuatorModel
from interceptor.simulation.mujoco_plant import MujocoPlant
from interceptor.simulation.trajectories import SinusoidalTrajectory
from interceptor.simulation.wind import WIND_PRESETS, WindField


def _pose_row(step: int, t: float, plant: MujocoPlant, target_pos: np.ndarray) -> dict:
    """Assemble one pose-log row (sim-layer fields only; guidance columns are zero)."""
    pos = plant.position_m
    quat = plant.orientation_quat
    rpm = RotorActuatorModel().hover_rpm()
    row = {name: 0.0 for name in LOG_FIELDS}
    row.update(
        step_index=step,
        sim_time_s=t,
        interceptor_x_m=float(pos[0]),
        interceptor_y_m=float(pos[1]),
        interceptor_z_m=float(pos[2]),
        interceptor_qw=float(quat[0]),
        interceptor_qx=float(quat[1]),
        interceptor_qy=float(quat[2]),
        interceptor_qz=float(quat[3]),
        target_x_m=float(target_pos[0]),
        target_y_m=float(target_pos[1]),
        target_z_m=float(target_pos[2]),
        saturated=0,
        rotor_rpm_0=rpm,
        rotor_rpm_1=rpm,
        rotor_rpm_2=rpm,
        rotor_rpm_3=rpm,
    )
    return row


def run_demo(run_id: str, seconds: float, seed: int, wind_preset: str, results_dir: Path) -> Path:
    from interceptor.common.types import MotorCommand

    rng_factory = RngFactory(seed)
    wind = None
    if wind_preset != "none":
        wind = WindField(
            WIND_PRESETS[wind_preset](),
            rng=rng_factory.stream("wind"),
            horizon_s=seconds + 1.0,
        )

    plant = MujocoPlant(initial_position_m=np.array([0.0, 0.0, 2.0]), wind=wind)
    target = SinusoidalTrajectory(
        start_position_m=np.array([8.0, 0.0, 5.0]),
        drift_velocity_m_s=np.array([0.0, 0.0, 0.0]),
        amplitude_m=np.array([0.0, 3.0, 1.0]),
        frequency_hz=0.3,
    )
    hover = MotorCommand(rotor_rpm=np.full(4, RotorActuatorModel().hover_rpm()))

    run_dir = results_dir / run_id
    snapshot = write_run_snapshot(
        run_dir,
        seed=seed,
        params=default_params().to_dict(),
        metadata={"run_id": run_id, "demo": "sim_demo", "wind": wind_preset},
    )

    dt = 1.0 / constants.SIM_HZ
    num_steps = int(seconds * constants.SIM_HZ)
    with RunLogger(run_dir, LOG_FIELDS) as logger:
        for step in range(num_steps):
            t = step * dt
            target_pos = target.position_at(t)
            plant.set_target_pose(target_pos)
            logger.log_step(_pose_row(step, t, plant, target_pos))
            plant.step(hover, dt)

    print(f"Wrote {num_steps} steps to {run_dir} (snapshot {snapshot.name}).")
    print(f"View it with:  python scripts/replay.py {run_dir}")
    return run_dir


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Simulation demo (writes a pose log).")
    parser.add_argument("--run-id", default="sim_demo")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--wind", choices=["none", *WIND_PRESETS], default="none")
    parser.add_argument("--results-dir", default="results")
    args = parser.parse_args(argv)
    run_demo(args.run_id, args.seconds, args.seed, args.wind, Path(args.results_dir))
    return 0


if __name__ == "__main__":
    sys.exit(main())
