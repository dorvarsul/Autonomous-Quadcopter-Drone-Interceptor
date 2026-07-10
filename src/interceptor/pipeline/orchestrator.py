"""Stub orchestrator: runs the full 6-stage pipeline end-to-end, headless.

Wires Simulation -> Estimation -> Guidance -> Command Limiter -> Outer -> Inner ->
Motor Mixer -> Simulation using injected components (Dependency Inversion). Phase 0
runs it with the pass-through stubs to prove the loop closes deterministically; later
phases inject real implementations behind the *same* interfaces with no orchestrator
change.

Determinism: the only randomness enters through the injected RNG factory; with stub
components there is none, so a given (seed, params, step count) yields a byte-identical
run log. The orchestrator respects the multi-rate schedule and never lets a layer read
across a contract boundary (guidance sees estimates, not sensors; control sees limited
acceleration, not guidance internals).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from interceptor.common import frames
from interceptor.common.logging import RunLogger, write_run_snapshot
from interceptor.common.rng import RngFactory
from interceptor.common.types import (
    AttitudeReference,
    LimitedAccelerationCommand,
    MotorCommand,
    TargetStateEstimate,
)
from interceptor.config import constants
from interceptor.config.params import Params, default_params
from interceptor.control.command_limiter import AccelerationCommandLimiter
from interceptor.control.inner_loop import AttitudePidInnerLoop
from interceptor.control.interfaces import (
    CommandLimiter,
    InnerLoopController,
    MotorMixer,
    OuterLoopController,
)
from interceptor.control.motor_mixer import QuadMotorMixer
from interceptor.control.outer_loop import DifferentialFlatnessOuterLoop
from interceptor.control.stubs import (
    PassThroughInnerLoop,
    PassThroughLimiter,
    PassThroughOuterLoop,
    UniformMotorMixer,
)
from interceptor.estimation.ekf import ExtendedKalmanFilter
from interceptor.estimation.interfaces import Estimator
from interceptor.estimation.stubs import PassThroughEstimator
from interceptor.guidance.interfaces import GuidanceLaw
from interceptor.guidance.ogl import OptimalGuidanceLaw
from interceptor.guidance.stubs import ZeroGuidance
from interceptor.pipeline.scheduler import MultiRateScheduler
from interceptor.simulation.interfaces import (
    Plant,
    Renderer,
    SensorModel,
    TargetTrajectory,
)
from interceptor.simulation.sensors.noisy_sensor import NoisyDelayedSensorModel
from interceptor.simulation.stubs import (
    IdealSensorModel,
    NullRenderer,
    StaticTargetTrajectory,
    StationaryPlant,
)
from interceptor.simulation.trajectories.generators import StaticTrajectory

# Columns of the per-step run log. Fixed order => deterministic CSV.
# The pose columns (interceptor quaternion + target position) make a run replayable in
# an interactive viewer (Phase 1 T1.10) without re-running the sim. They are additive:
# downstream analysis keys by column name, so appending columns is safe.
LOG_FIELDS = (
    "step_index",
    "sim_time_s",
    "interceptor_x_m",
    "interceptor_y_m",
    "interceptor_z_m",
    "interceptor_qw",
    "interceptor_qx",
    "interceptor_qy",
    "interceptor_qz",
    "target_x_m",
    "target_y_m",
    "target_z_m",
    "estimate_range_m",
    "accel_cmd_norm_m_s2",
    "saturated",
    "rotor_rpm_0",
    "rotor_rpm_1",
    "rotor_rpm_2",
    "rotor_rpm_3",
)

# Identity orientation used when a plant does not expose an attitude (e.g. the Phase 0
# stub plant): body frame aligned with world.
_IDENTITY_QUAT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


@dataclass(frozen=True)
class PipelineComponents:
    """The injected, interface-typed components wired into the loop.

    Defaulting to the Phase 0 stubs keeps the call site trivial while leaving every slot
    swappable for a real implementation (Open/Closed, Dependency Inversion).
    """

    trajectory: TargetTrajectory
    sensor: SensorModel
    estimator: Estimator
    guidance: GuidanceLaw
    limiter: CommandLimiter
    outer_loop: OuterLoopController
    inner_loop: InnerLoopController
    mixer: MotorMixer
    plant: Plant
    renderer: Renderer

    @staticmethod
    def default_stubs() -> PipelineComponents:
        """All-stub wiring used by Phase 0 and by infra tests."""
        return PipelineComponents(
            trajectory=StaticTargetTrajectory(np.array([10.0, 0.0, 5.0])),
            sensor=IdealSensorModel(),
            estimator=PassThroughEstimator(),
            guidance=ZeroGuidance(),
            limiter=PassThroughLimiter(),
            outer_loop=PassThroughOuterLoop(),
            inner_loop=PassThroughInnerLoop(),
            mixer=UniformMotorMixer(),
            plant=StationaryPlant(np.zeros(3)),
            renderer=NullRenderer(),
        )

    @staticmethod
    def phase2_intercept(
        rng: RngFactory,
        params: Params,
        *,
        interceptor_position_m: np.ndarray,
        target_position_m: np.ndarray,
        scene_path: str | Path | None = None,
    ) -> PipelineComponents:
        """Real Phase 2 wiring: MuJoCo plant + noisy sensor + EKF + OGL + control + mixer.

        This is the closed loop the Phase 2 exit criterion exercises (static-target
        interception). Every slot is a real implementation behind the same interface the
        stubs satisfied, so the orchestrator itself is unchanged (Open/Closed). MuJoCo is
        imported lazily so importing this module never requires the physics engine.
        """
        # Lazy import: keeps the stub path (and non-mujoco tests) free of the native dep.
        from interceptor.simulation.mujoco_plant import DEFAULT_SCENE_PATH, MujocoPlant

        plant = MujocoPlant(
            scene_path=scene_path or DEFAULT_SCENE_PATH,
            initial_position_m=np.asarray(interceptor_position_m, dtype=np.float64),
        )
        return PipelineComponents(
            trajectory=StaticTrajectory(np.asarray(target_position_m, dtype=np.float64)),
            sensor=NoisyDelayedSensorModel(params.sensor, rng.stream("sensor")),
            estimator=ExtendedKalmanFilter(params.ekf),
            guidance=OptimalGuidanceLaw(params.guidance),
            limiter=AccelerationCommandLimiter(params.limiter),
            outer_loop=DifferentialFlatnessOuterLoop(),
            inner_loop=AttitudePidInnerLoop(params.control),
            mixer=QuadMotorMixer(),
            plant=plant,
            renderer=NullRenderer(),
        )


@dataclass(frozen=True)
class RunResult:
    """Outcome of a pipeline run; everything needed to verify/replay it."""

    num_steps: int
    run_dir: Path
    log_path: Path
    snapshot_path: Path
    final_motor_command: MotorCommand


class StubOrchestrator:
    """Runs the multi-rate pipeline for a fixed number of steps, logging each step."""

    def __init__(
        self,
        components: PipelineComponents | None = None,
        params: Params | None = None,
        seed: int = 0,
    ) -> None:
        self._components = components or PipelineComponents.default_stubs()
        self._params = params or default_params()
        self._rng = RngFactory(seed)
        self._scheduler = MultiRateScheduler(
            sim_hz=constants.SIM_HZ,
            inner_loop_hz=constants.INNER_LOOP_HZ,
            outer_loop_hz=constants.OUTER_LOOP_HZ,
            estimation_hz=constants.ESTIMATION_HZ,
            guidance_hz=constants.GUIDANCE_HZ,
        )

    def run(
        self,
        num_steps: int,
        run_dir: Path,
        run_id: str = "phase0_stub",
        *,
        terminate_on_intercept: bool = False,
        capture_radius_m: float = constants.INTERCEPT_CAPTURE_RADIUS_M,
    ) -> RunResult:
        """Execute the loop headlessly and return a :class:`RunResult`.

        Enforces the headless guarantee up front: a non-headless renderer is a defect in
        an automated run and fails loud (AGENTS.md → no hanging GLFW window).

        ``num_steps`` is the *maximum* duration. When ``terminate_on_intercept`` is set,
        the run stops at closest approach — once the true interceptor↔target range has
        come within ``capture_radius_m`` and then starts increasing, the engagement is
        over (Role 5/6). The last logged frame is exactly that closest-approach point, so
        no physically meaningless post-intercept flyby is recorded. Off by default so the
        Phase 0/2 fixed-duration runs and their determinism tests are unaffected.
        """
        if not self._components.renderer.is_headless:
            raise RuntimeError(
                "Orchestrator requires a headless renderer for automated runs; "
                "got a windowed renderer."
            )

        run_dir = Path(run_dir)
        snapshot_path = write_run_snapshot(
            run_dir,
            seed=self._rng.seed,
            params=self._params.to_dict(),
            metadata={
                "run_id": run_id,
                "num_steps": num_steps,
                "sim_hz": constants.SIM_HZ,
                "inner_loop_hz": constants.INNER_LOOP_HZ,
                "outer_loop_hz": constants.OUTER_LOOP_HZ,
                "estimation_hz": constants.ESTIMATION_HZ,
                "guidance_hz": constants.GUIDANCE_HZ,
                "guidance_law": self._components.guidance.name,
            },
        )

        dt = 1.0 / constants.SIM_HZ
        # Each layer is advanced with the elapsed time for *its own* rate, not the sim dt.
        estimation_dt = 1.0 / constants.ESTIMATION_HZ
        c = self._components

        # State carried across ticks (slower loops reuse their latest output).
        estimate: TargetStateEstimate | None = None
        limited: LimitedAccelerationCommand | None = None
        desired_attitude: AttitudeReference | None = None
        motor_command = MotorCommand(rotor_rpm=np.full(4, constants.MOTOR_RPM_MIN))

        # Engagement-termination state (closest-approach detection).
        previous_range_m: float | None = None
        capture_armed = False
        steps_completed = 0

        with RunLogger(run_dir, LOG_FIELDS) as logger:
            for tick in self._scheduler.ticks(num_steps):
                interceptor_pos = c.plant.position_m
                target_pos = c.trajectory.position_at(tick.sim_time_s)

                # --- Engagement termination (Role 5/6) -------------------------------
                # Stop at closest approach: once inside the capture radius, the first
                # frame where the range grows means the previous (already-logged) frame
                # was the intercept point. Break *before* stepping/logging this receding
                # frame so the log ends exactly at closest approach.
                if terminate_on_intercept:
                    true_range_m = float(np.linalg.norm(target_pos - interceptor_pos))
                    if true_range_m <= capture_radius_m:
                        capture_armed = True
                    if (
                        capture_armed
                        and previous_range_m is not None
                        and true_range_m > previous_range_m
                    ):
                        break
                    previous_range_m = true_range_m

                # Drive the kinematic target body (mocap) so a real plant renders/replays
                # the target; stub plants without this method are unaffected.
                set_target_pose = getattr(c.plant, "set_target_pose", None)
                if set_target_pose is not None:
                    set_target_pose(target_pos)

                # --- Estimation (consumes ONLY the raw sensor measurement) -----------
                if tick.run_estimation:
                    measurement = c.sensor.measure(interceptor_pos, target_pos, tick.sim_time_s)
                    estimate = c.estimator.update(measurement, estimation_dt)

                # --- Guidance + Command Limiter (consume ONLY the estimate) ----------
                if tick.run_guidance and estimate is not None:
                    accel_cmd = c.guidance.compute(estimate)
                    limited = c.limiter.limit(accel_cmd)

                # --- Outer loop (consumes ONLY the limited acceleration) -------------
                if tick.run_outer_loop and limited is not None:
                    desired_attitude = c.outer_loop.compute_attitude(limited)

                # --- Inner loop + Motor Mixer (fast loop, gyro feedback) -------------
                if tick.run_inner_loop and desired_attitude is not None:
                    commanded = c.inner_loop.track(desired_attitude, c.plant.body_rates_rad_s)
                    motor_command = c.mixer.mix(commanded)

                # Body attitude for pose logging/replay; stubs without an attitude
                # report identity (body aligned with world).
                interceptor_quat = getattr(c.plant, "orientation_quat", _IDENTITY_QUAT)

                # --- Simulation step (actuators back into the world) -----------------
                c.plant.step(motor_command, dt)
                c.renderer.render(tick.sim_time_s)

                logger.log_step(
                    self._log_row(
                        tick,
                        interceptor_pos,
                        interceptor_quat,
                        target_pos,
                        estimate,
                        limited,
                        motor_command,
                    )
                )
                steps_completed += 1

        return RunResult(
            num_steps=steps_completed,
            run_dir=run_dir,
            log_path=run_dir / "run_log.csv",
            snapshot_path=snapshot_path,
            final_motor_command=motor_command,
        )

    @staticmethod
    def _log_row(
        tick,
        interceptor_pos: np.ndarray,
        interceptor_quat: np.ndarray,
        target_pos: np.ndarray,
        estimate: TargetStateEstimate | None,
        limited: LimitedAccelerationCommand | None,
        motor_command: MotorCommand,
    ) -> dict:
        """Assemble one deterministic log row from the current loop state."""
        accel_norm = (
            float(np.linalg.norm(limited.acceleration_m_s2)) if limited is not None else 0.0
        )
        quat = np.asarray(interceptor_quat, dtype=np.float64)
        return {
            "step_index": tick.step_index,
            "sim_time_s": tick.sim_time_s,
            "interceptor_x_m": float(interceptor_pos[frames.X]),
            "interceptor_y_m": float(interceptor_pos[frames.Y]),
            "interceptor_z_m": float(interceptor_pos[frames.Z]),
            "interceptor_qw": float(quat[0]),
            "interceptor_qx": float(quat[1]),
            "interceptor_qy": float(quat[2]),
            "interceptor_qz": float(quat[3]),
            "target_x_m": float(target_pos[frames.X]),
            "target_y_m": float(target_pos[frames.Y]),
            "target_z_m": float(target_pos[frames.Z]),
            "estimate_range_m": float(estimate.range_m) if estimate is not None else 0.0,
            "accel_cmd_norm_m_s2": accel_norm,
            "saturated": bool(limited.saturated) if limited is not None else False,
            "rotor_rpm_0": float(motor_command.rotor_rpm[0]),
            "rotor_rpm_1": float(motor_command.rotor_rpm[1]),
            "rotor_rpm_2": float(motor_command.rotor_rpm[2]),
            "rotor_rpm_3": float(motor_command.rotor_rpm[3]),
        }
