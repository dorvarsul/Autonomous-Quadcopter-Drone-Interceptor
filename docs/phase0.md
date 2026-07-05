# Phase 0 — Foundations, Scaffolding & Pipeline Contracts

> **Roadmap window:** Pre-Phase-1 setup (enabling work before Jun 17 feature work).
> **Primary role:** Role 6 — Integration Architect (cross-cutting).
> **Pipeline stages touched:** All (defines interfaces and wiring; implements none of
> the layer math).
> **Goal:** Stand up a deterministic, headless, testable project skeleton in which
> every pipeline stage exists as a typed, swappable stub. By the end of Phase 0 the
> full Simulation→…→Motor Mixer→Simulation loop runs end-to-end with pass-through
> stubs, seeded and logged, so later phases drop real algorithms into stable slots.

This phase writes **no algorithm logic** (no EKF, no guidance, no control law). It
builds structure, contracts, constants, and infrastructure only.

---

## Entry Criteria

- MuJoCo installed at `C:/Dev/Libraries/mujoco`.
- Access to the project root `C:/Dev/University/Workshop_Autonomous_Systems`.
- Design Review and AGENTS.md available.

## Exit Criteria (Definition of Done for the phase)

- [ ] Python environment reproducibly installable; MuJoCo binding smoke test passes
      headless.
- [ ] Repository layout from [implementation_plan.md](./implementation_plan.md#4-target-module-layout)
      exists with package init files.
- [ ] All shared constants live in `config/` with units + `Why` comments — zero magic
      numbers downstream.
- [ ] All inter-layer **data contracts** (typed messages) and **abstract interfaces**
      are defined and documented.
- [ ] A stub orchestrator runs the full 6-stage loop headless and deterministically,
      writing a run log + config snapshot.
- [ ] `pytest` runs green on contract/infra tests; everything is non-interactive.

---

## Tasks

### T0.1 — Environment & dependency setup
**Role:** 6 · **Depends on:** —

- [ ] Create dependency manifest (`pyproject.toml` or `requirements.txt`) pinning:
      `mujoco`, `numpy`, `scipy`, `pyyaml`, `matplotlib`, `pytest`. (No others without
      user confirmation.)
- [ ] Document the env activation steps: append `C:/Dev/Libraries/mujoco/bin` to
      `PATH`; note the legacy `MUJOCO_PY_MUJOCO_PATH` variable only if ever needed.
- [ ] Add `scripts/check_env.py` (env doctor): verifies Python version, imports
      `mujoco`, prints MuJoCo version, confirms binaries resolvable.
- [ ] **Smoke test:** load a trivial MJCF, step the sim N times, and render one frame
      **off-screen** — proving headless rendering works with no GLFW window.
- [ ] **DoD:** `python scripts/check_env.py` exits 0; smoke test produces a frame
      buffer headlessly.

### T0.2 — Repository structure
**Role:** 6 · **Depends on:** T0.1

- [ ] Create the directory tree exactly as in the implementation plan (`models/`,
      `scenarios/`, `results/`, `scripts/`, `src/interceptor/{config,common,simulation,
      estimation,guidance,control,pipeline,analysis}`, `tests/{unit,integration}`).
- [ ] Add package `__init__.py` files and a top-level `README.md` describing how to
      run sims and tests headlessly.
- [ ] Add `.gitignore` for `results/`, caches, virtual envs, generated media.
- [ ] **DoD:** package imports resolve; empty modules are importable.

### T0.3 — Shared constants (single source of truth)
**Role:** 6 · **Depends on:** T0.2

- [ ] `config/constants.py` — physical & system constants, each with units + a `Why`
      comment citing the Design Review. Minimum set:
  - Physics: `GRAVITY_M_S2`, `AIR_DENSITY_KG_M3`.
  - Airframe (placeholders, refined in Phase 1): `QUAD_MASS_KG`,
    `QUAD_INERTIA_KG_M2` (Ixx/Iyy/Izz), `ARM_LENGTH_M`.
  - Motors: `MOTOR_RPM_MIN`, `MOTOR_RPM_MAX`, `THRUST_COEFF_KT`, `TORQUE_COEFF_KQ`.
  - Loop rates: `SIM_HZ`, `INNER_LOOP_HZ = 400`, `OUTER_LOOP_HZ = 50`,
    `ESTIMATION_HZ` (tie to sensor rate).
  - Guidance: `TILT_DELAY_TIME_CONSTANT_S` (the `T` in `1/(Ts+1)`),
    `ALTITUDE_PENALTY_B = 0.1`, nav-ratio term placeholders.
  - KPI thresholds: `R_MISS_MAX_M = 1.05`, `T_INT_STATIC_MAX_S = 10`,
    `T_INT_MOVING_MAX_S = 20`, `Z_OVERSHOOT_MAX_M = 0.5`,
    `CMD_SATURATION_MAX_FRAC = 0.05`, `MAX_TARGET_SPEED_MIN_KMH = 83.6`,
    `MISSION_SUCCESS_MIN = 0.90`.
- [ ] `config/params.py` — runtime-tunable parameters (EKF Q/R, PID gains, N'
      schedule, limiter bounds) with safe defaults, loadable/overridable from YAML.
- [ ] **DoD:** no downstream module hard-codes any of these values; values carry units
      and rationale.

> **Constraint:** Changing any physical constant or KPI-affecting tuning value later
> requires user confirmation (AGENTS.md → Workflow).

### T0.4 — Coordinate frames & conventions
**Role:** 6 · **Depends on:** T0.2

- [ ] `common/frames.py` + a documented convention note: define world frame (Z-up
      altitude axis), body frame, rotation representation (quaternion primary, Euler
      roll φ / pitch θ / yaw ψ secondary), and sign conventions.
- [ ] Define **LOS angle** and **LOS rate** conventions and the relative-kinematics
      sign rules used by Estimation/Guidance.
- [ ] Provide frame-transform helpers (world↔body) with unit tests.
- [ ] **DoD:** a documented, tested convention exists; the Z/altitude axis is called
      out explicitly given known overshoot sensitivity.

### T0.5 — Pipeline data contracts (typed messages)
**Role:** 6 · **Depends on:** T0.4

- [ ] `common/types.py` — immutable, typed dataclasses, one per pipeline edge, each
      field documented with units. At minimum:
  - `RawSensorMeasurement` — measured range / LOS angles (and any measured rates),
    measurement timestamp, and the latency/age of the sample.
  - `TargetStateEstimate` — relative position, relative velocity, range, **LOS rate**,
    angular rates, plus **estimate covariance / quality** for Guidance to reason about.
  - `AccelerationCommand` — guidance-requested acceleration vector (ideal, unclamped).
  - `LimitedAccelerationCommand` — clamped accel + saturation flag/metric.
  - `AttitudeReference` — target roll/pitch/yaw + thrust.
  - `MotorCommand` — four rotor RPM values.
- [ ] Add validation/`__post_init__` guards (shape, finiteness) that **fail loud** on
      NaN/Inf.
- [ ] **DoD:** every inter-layer hand-off in the contract has exactly one message type;
      no layer needs a field outside its contract.

### T0.6 — Abstract interfaces (Open/Closed + Dependency Inversion)
**Role:** 6 · **Depends on:** T0.5

- [ ] Define narrow abstract base classes (interfaces), one per swappable component;
      orchestration depends on these, not concretes:
  - `SensorModel` (Role 1), `TargetTrajectory` (Role 1), `Renderer` (Role 1).
  - `Estimator` (Role 2) — consumes `RawSensorMeasurement`, returns
    `TargetStateEstimate`.
  - `GuidanceLaw` (Role 3) — consumes `TargetStateEstimate`, returns
    `AccelerationCommand`. Any guidance law (OGL) must satisfy this (Liskov).
  - `CommandLimiter`, `FlightController` (outer + inner), `MotorMixer` (Role 4).
- [ ] Keep interfaces minimal — e.g., Control depends only on "give me an acceleration
      command," not on guidance internals (Interface Segregation).
- [ ] **DoD:** each interface has a docstring stating inputs/outputs/units and the role
      that owns it; a stub implementation of each exists for T0.8.

### T0.7 — Determinism, RNG & logging infrastructure
**Role:** 6 · **Depends on:** T0.2

- [ ] `common/rng.py` — centralized seeded RNG factory; all stochastic components draw
      from injected generators (no global `random`/`np.random` calls).
- [ ] `common/logging.py` — per-timestep structured run logger (CSV/Parquet) and a
      run-config snapshot writer that records all params, the seed, and the git hash
      to `results/<run_id>/`.
- [ ] Add shared **fail-loud** guards (NaN/instability detectors) usable by any layer.
- [ ] **DoD:** identical seed + identical config ⇒ byte-identical run log.

### T0.8 — Multi-rate scheduler & stub orchestrator
**Role:** 6 · **Depends on:** T0.3, T0.6, T0.7

- [ ] `pipeline/scheduler.py` — a deterministic multi-rate clock coordinating sim
      step, **inner loop 400 Hz**, **outer loop 50 Hz**, and the estimation/guidance
      cadence, without collapsing loops.
- [ ] `pipeline/orchestrator.py` — wires `SensorModel → Estimator → GuidanceLaw →
      CommandLimiter → outer → inner → MotorMixer → sim`, using **stub** (pass-through)
      implementations from T0.6.
- [ ] The stub loop must run a fixed number of steps headless and produce a run log.
- [ ] **DoD:** the full 6-stage loop executes end-to-end on stubs, headless,
      deterministic, with no contract violations and no GLFW window.

### T0.9 — Test harness & quality gates
**Role:** 6 · **Depends on:** T0.7, T0.8

- [ ] Configure `pytest`; add fixtures for seeded RNG and a tiny MJCF.
- [ ] Add contract tests: each message type rejects NaN; each interface stub satisfies
      its ABC; scheduler produces the expected tick counts per rate.
- [ ] Add a determinism test (same seed ⇒ same log) and a headless guarantee test.
- [ ] Add lint/format config (e.g., `ruff`/`black`) — confirm tooling with user if it
      adds dependencies.
- [ ] **DoD:** `pytest` green; all tests non-interactive and headless.

---

## Deliverables

- Reproducible Python environment + `scripts/check_env.py`.
- Full repository skeleton with package modules.
- `config/constants.py`, `config/params.py` (single source of truth).
- `common/{frames,types,rng,logging}.py` with data contracts and interfaces.
- `pipeline/{scheduler,orchestrator}.py` running the stubbed pipeline.
- Passing `pytest` infrastructure suite.

## Risks & Mitigations

- **MuJoCo DLLs not found** → `check_env.py` validates `PATH` early; document the fix.
- **Hidden GLFW window hangs automation** → enforce off-screen rendering in the smoke
  test and a headless guarantee test from day one.
- **Interface churn later** → keep interfaces narrow and message-typed now; any change
  is a deliberate, user-confirmed contract change (Role 6).

## References

- Design Review §3 (MuJoCo pipeline), §5 (6-stage architecture), §8 (Roadmap).
- AGENTS.md → MuJoCo Execution Environment, Pipeline Contract, Coding Standards.
