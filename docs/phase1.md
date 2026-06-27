# Phase 1 — Simulation Environment & Sensor Models

> **Roadmap window:** Jun 17 – Jun 30.
> **Primary role:** Role 1 — Simulation Environment Engineer.
> **Pipeline stage owned:** Simulation (physics, active-space modeling, feedback loop).
> **Goal:** Build the world the other layers perceive and act within — a numerically
> stable MuJoCo quadcopter + target, faithful sensor models with **configurable noise
> and latency**, the full set of target-trajectory generators, a wind/gust
> disturbance model, and off-screen rendering. This phase produces **only** the
> environment and sensor outputs; it implements **no** estimation, guidance, or
> control math.

---

## Entry Criteria

- Phase 0 complete: repo skeleton, constants, data contracts (`RawSensorMeasurement`,
  etc.), interfaces (`SensorModel`, `TargetTrajectory`, `Renderer`), seeded RNG and
  logging, stub orchestrator running headless.

## Exit Criteria (Definition of Done for the phase)

- [ ] Quadcopter MJCF hovers in stable equilibrium (thrust ≈ `mass·g`) with no drift
      and conserved energy over a long headless run.
- [ ] Motor actuator model respects `MOTOR_RPM_MIN/MAX` saturation and the
      thrust/torque coefficient relations.
- [ ] All five target trajectory families exist behind `TargetTrajectory`: static,
      linear, sinusoidal, varying-speed (parametrizable to ≥ 90 km/h), and a
      configurable wind-affected variant.
- [ ] Ground-truth relative kinematics (relative position, range, LOS angles, LOS
      rate, closing velocity) computed correctly — available **only** to the
      Simulation/sensor layer.
- [ ] `SensorModel` emits `RawSensorMeasurement` with configurable Gaussian noise,
      bias, and **latency**; emitted noise statistics match the configured profile.
- [ ] Wind/gust disturbance model applies seeded, reproducible perturbations.
- [ ] Everything runs headless/off-screen and is deterministic given a seed.

---

## Tasks

### T1.1 — Quadcopter MJCF model
**Role:** 1 · **Depends on:** Phase 0

- [ ] Author `models/quadcopter.xml`: airframe body, four arms/rotor sites in the
      standard X (or +) configuration, collision/visual geoms.
- [ ] Set mass and inertia (Ixx/Iyy/Izz) consistent with `config/constants.py`; place
      the center of mass correctly.
- [ ] Define the four rotor thrust sites and their spin directions (two CW, two CCW)
      so yaw torque can be produced by differential drag.
- [ ] **DoD:** model loads; static hover at the computed equilibrium thrust holds
      position to within a small tolerance over ≥ 30 s sim time.

### T1.2 — Rotor actuator & motor dynamics
**Role:** 1 · **Depends on:** T1.1

- [ ] Implement the actuator mapping consumed by the Motor Mixer later: per-rotor
      command → RPM → thrust `= THRUST_COEFF_KT · RPM²` and reaction torque
      `= TORQUE_COEFF_KQ · RPM²`.
- [ ] Enforce `MOTOR_RPM_MIN`/`MOTOR_RPM_MAX` saturation at the actuator boundary
      (the physical limit; the mixer in Phase 2 must respect the same bound).
- [ ] (Optional, document if included) first-order motor spin-up lag — keep separate
      from the guidance-level tilt delay `1/(Ts+1)`.
- [ ] **DoD:** commanding max RPM yields the expected max thrust; commands beyond
      limits clamp and **log a saturation event**, not silently.

### T1.3 — Target drone model & trajectory generators
**Role:** 1 · **Depends on:** Phase 0

- [ ] Author `models/target.xml` (or a kinematically driven body) for the target UAV.
- [ ] Implement `TargetTrajectory` concrete generators, each seeded and configurable:
  - [ ] **Static** — fixed 3D point.
  - [ ] **Linear** — constant-velocity straight line.
  - [ ] **Sinusoidal** — evasive weave (configurable amplitude/frequency, 3D).
  - [ ] **Varying-speed** — ramp/scale up to and beyond 90 km/h (25 m/s) to exercise
        `MAX_TARGET_SPEED_MIN_KMH`.
  - [ ] **Wind-affected** — a trajectory perturbed by the T1.6 wind field.
- [ ] Drive the target as a kinematic/mocap body so its motion is prescribed (it is a
      threat to track, not a controlled interceptor).
- [ ] **DoD:** each generator reproduces an identical path for a fixed seed; speeds and
      shapes match config; unit-tested.

### T1.4 — Active 3D space & relative kinematics (ground truth)
**Role:** 1 · **Depends on:** T1.1, T1.3

- [ ] Compute, each step, the true relative state between interceptor and target:
      relative position & velocity, range, **LOS angles**, **LOS rate**, closing
      velocity, look-angles — using the Phase 0 frame conventions.
- [ ] Expose these **only** to the Simulation/sensor layer (they are the raw truth the
      sensors corrupt; Estimation/Guidance/Control must never read them directly).
- [ ] **DoD:** relative kinematics validated against hand-computed cases for known
      geometries; sign/axis conventions match `common/frames.py`.

### T1.5 — Sensor models (Radar/LiDAR/Camera analogues): noise + latency
**Role:** 1 · **Depends on:** T1.4

- [ ] Implement `SensorModel` concretes that turn ground-truth relative kinematics into
      `RawSensorMeasurement` (measured range, LOS bearing/elevation, and any measured
      rate), with per-channel:
  - [ ] configurable **Gaussian noise** (std) and optional **bias**;
  - [ ] optional quantization;
  - [ ] a finite **update rate** (sensor slower than sim);
  - [ ] a configurable **latency** via a delay buffer, stamping each measurement with
        its timestamp/age.
- [ ] Make noise/latency parameters live in `config/params.py` / scenario YAML — never
      hard-coded, and never "cleaned up" to flatter downstream layers.
- [ ] **Fail loud** if a sensor is constructed without a noise/latency profile.
- [ ] **DoD:** over a long run, the measured-minus-true residual statistics match the
      configured noise std/bias; the measurement delay equals the configured latency.

### T1.6 — Wind & gust disturbance model
**Role:** 1 · **Depends on:** T1.1

- [ ] Implement a configurable external disturbance: steady wind vector + stochastic
      gusts (seeded), applied as forces/perturbations in the physics step.
- [ ] Provide presets (calm, moderate, gusty) referenced by scenario configs; used
      heavily in Phase 4 but built and unit-tested here.
- [ ] **DoD:** with a fixed seed the disturbance time-series is reproducible; zero-wind
      preset reduces to the undisturbed dynamics exactly.

### T1.7 — Off-screen rendering
**Role:** 1 · **Depends on:** T1.1

- [ ] Implement the `Renderer` concrete for **off-screen** frame capture (debug
      images / optional video to `results/`), with absolutely no interactive GLFW
      window in automated runs.
- [ ] Provide a flag to disable rendering entirely for fastest headless batch runs.
- [ ] **DoD:** rendering works in a headless process; disabling it changes nothing in
      the physics/log (determinism preserved).

### T1.8 — Solver configuration & numerical stability
**Role:** 1 · **Depends on:** T1.1

- [ ] Choose and document the integrator, `SIM_HZ`/timestep, and solver settings so
      the sim is stable and energy-conserving; ensure `SIM_HZ` is compatible with the
      400 Hz inner loop.
- [ ] Add a stability/energy-conservation check (free-fall, hover, and a torque
      impulse) over a long horizon.
- [ ] **DoD:** no NaN/blow-up over an extended headless run; energy drift within a
      documented tolerance; timestep choice justified with a `Why` comment.

### T1.9 — Phase 1 unit tests
**Role:** 1 · **Depends on:** T1.1–T1.8

- [ ] Hover equilibrium & no-drift test.
- [ ] RPM saturation + thrust/torque coefficient test.
- [ ] Trajectory generator correctness + determinism tests (all five families).
- [ ] Relative-kinematics correctness tests (LOS angle/rate for known geometries).
- [ ] Sensor noise-statistics and latency-delay tests.
- [ ] Wind reproducibility test.
- [ ] **DoD:** all Phase 1 unit tests pass headlessly and deterministically.

### T1.10 — Interactive replay viewer (opt-in, off the automated path)
**Role:** 1 · **Depends on:** T1.1, T1.7

> **Intent:** a developer/demo tool to *watch* a logged interception in a live MuJoCo
> window. It is a **consumer of a deterministic artifact**, never part of the sim or
> control loop, so it cannot affect physics, estimation, guidance, control, or results.
> This is the only sanctioned interactive window in the project and it must remain
> strictly opt-in — never invoked by tests, batch trials, or CI.

- [ ] Extend the run-log schema to record **full poses** needed to replay faithfully:
      interceptor *and* target **position and orientation quaternion** per step (the
      current log carries only interceptor position). Treat the log schema as the
      shared contract it is — add columns, do not repurpose existing ones.
- [ ] Implement `scripts/replay.py`: load `models/scene.xml` + a `results/<run_id>/`
      run log, set body poses from each logged step, and play back in an interactive
      `mujoco.viewer` window with pause and time-scrub. Real-time pacing here affects
      *only the playback clock*, never a physics step.
- [ ] The viewer reads logged state only — it must **not** re-run the sim, re-step
      physics, or read ground truth live. Replaying the same log twice looks identical.
- [ ] Provide a **non-interactive smoke test**: construct the replay session, load a
      short canned log, and advance one frame **without blocking** (headless/off-screen,
      so CI never opens a window). The interactive window itself is exercised manually.
- [ ] **DoD:** `python scripts/replay.py results/<run_id>` opens a live window playing
      back the logged interception; the automated test suite stays fully headless and
      deterministic, and disabling/never-launching the viewer changes nothing in any
      run log.

---

## Deliverables

- `models/quadcopter.xml`, `models/target.xml`, `models/scene.xml`.
- `simulation/` package: world, motor/actuator model, `trajectories/`, `sensors/`,
  wind model, off-screen `Renderer`.
- `scripts/replay.py` — opt-in interactive replay viewer (consumes logged runs only).
- Pose-augmented run-log schema (interceptor + target position & orientation).
- Phase 1 unit-test suite.

## KPIs Touched (enabling, not yet measured)

- Provides the substrate for **Max Target Speed ≥ 83.6 km/h** (T1.3 varying-speed),
  **Command Saturation** measurability (T1.2 RPM limits), and the noisy/delayed inputs
  that the EKF must tame (T1.5) — all evaluated in Phases 3–4.

## Risks & Mitigations

- **Unstable physics / energy injection** → T1.8 stability gate before any closed-loop
  work; justify timestep.
- **Sensor model too clean** → noise/latency are intentional and mandatory; T1.5 DoD
  verifies residual statistics; do not sanitize signals for downstream convenience.
- **Hidden render window** → T1.7 enforces off-screen only; the T1.10 replay viewer is
  the sole sanctioned interactive window and is opt-in, replay-only, and never on the
  test/batch/CI path.
- **Live viewer contaminating determinism** → T1.10 forbids attaching a viewer to the
  running loop; it replays logged poses only, so playback can never alter a result.

## Boundaries (do not cross)

- No EKF, guidance, or control math here.
- Do not alter physical constants to flatter future KPI results.
- Ground-truth relative state stays inside the Simulation/sensor layer.

## References

- Design Review §3 (MuJoCo pipeline), §5.1 (Simulation), §7 (Scenarios), §8 Phase 1.
- AGENTS.md → Role 1, Pipeline Contract, Physics & Numerical Integrity.
