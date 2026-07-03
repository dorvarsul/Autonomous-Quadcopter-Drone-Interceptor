# Phase 1 — Progress Report

> **Status: COMPLETE.** All exit criteria met. Implemented as Role 1 (Simulation
> Environment Engineer). This phase builds **only** the world the other layers perceive
> and act within — physics, sensors, trajectories, wind, rendering. It implements **no**
> estimation, guidance, or control math (those are Phase 2).

This report maps every Phase 1 task (`T1.1`–`T1.10`) to what was built, where it lives,
and how its Definition of Done was verified.

---

## How to reproduce the verification

```powershell
.\.venv\Scripts\Activate.ps1
pytest                               # 107 passed (16 MuJoCo-marked + 91 pure-Python)
ruff check src tests scripts         # All checks passed
python scripts/check_env.py          # env doctor + off-screen render -> exit 0

# Produce and watch a real-physics flight log (opt-in interactive window):
python scripts/run_sim_demo.py --run-id sim_demo --seconds 8 --wind moderate
python scripts/replay.py results/sim_demo
```

**Observed results (this machine):**

- `pytest` → **107 passed** in ~1.7 s (91 pure-Python + 16 MuJoCo-marked).
- `ruff check` → **All checks passed**.
- `check_env.py` → exit 0; one frame rendered **off-screen** `(120, 160, 3)`, no window.
- MuJoCo plant hovers at equilibrium thrust over **30 s** with < 5 cm drift; free-fall
  matches `0.5·g·t²`; stepping is deterministic.
- `run_sim_demo.py` with a fixed seed produces a **byte-identical** `run_log.csv`
  across runs (verified with `cmp`, incl. the gusty-wind preset).

---

## Exit criteria checklist (from phase1.md)

- [x] Quadcopter MJCF hovers in stable equilibrium (thrust ≈ `mass·g`), no drift, stable
      over a long headless run.
- [x] Motor actuator model respects `MOTOR_RPM_MIN/MAX` saturation and the
      thrust/torque coefficient relations.
- [x] All five target trajectory families exist behind `TargetTrajectory`.
- [x] Ground-truth relative kinematics (rel pos/vel, range, LOS angles, LOS rate,
      closing velocity) computed correctly — confined to the Simulation/sensor layer.
- [x] `SensorModel` emits `RawSensorMeasurement` with configurable Gaussian noise,
      bias, and latency; residual statistics match the configured profile.
- [x] Wind/gust disturbance model applies seeded, reproducible perturbations.
- [x] Everything runs headless/off-screen and is deterministic given a seed.

---

## Task-by-task

### T1.1 — Quadcopter MJCF model ✅
- **`models/quadcopter.xml`** — "+"-configuration airframe (free joint, four arms/rotor
  sites in MotorCommand order `[front, right, back, left]`), with an explicit
  `<inertial>` sourcing mass/inertia from `constants.py`. A fragment `<include>`d by
  `scene.xml`; also loads standalone.
- **DoD:** the plant hovers at the computed equilibrium thrust holding position to
  < 5 cm over 30 s (`test_hover_holds_altitude_over_30_seconds`). ✔

### T1.2 — Rotor actuator & motor dynamics ✅
- **`src/interceptor/simulation/actuators.py`** — `RotorActuatorModel`: quadratic rotor
  model `thrust = kT·rpm²`, yaw drag `= kQ·rpm²`, "+"-config arm-lever roll/pitch and
  differential-drag yaw. Enforces `MOTOR_RPM_MIN/MAX` at the physical boundary and
  reports a `RotorSaturationEvent` (fail-loud, KPI-measurable) rather than clamping
  silently. `hover_rpm()` helper = `sqrt(m·g / (4·kT))`.
- **DoD:** max RPM → expected max thrust; over-limit commands clamp and flag saturation;
  balanced hover → zero torque; differential thrust → correct roll/pitch/yaw signs
  (`test_actuators.py`, `test_saturation_is_reported_on_overspeed_command`). ✔

### T1.3 — Target model & trajectory generators ✅
- **`models/target.xml`** — kinematic **mocap** target body (prescribed motion, immune
  to contact/wake).
- **`src/interceptor/simulation/trajectories/`** — five generators behind
  `TargetTrajectory`, each with an analytic `velocity_at` for the kinematics layer:
  `StaticTrajectory`, `LinearTrajectory`, `SinusoidalTrajectory` (3D evasive weave),
  `VaryingSpeedTrajectory` (ramps past 25 m/s = 90 km/h, exact distance integral),
  `WindAffectedTrajectory` (base path + integral of a seeded wind field).
- **DoD:** each reproduces an identical path for a fixed config/seed; varying-speed
  exceeds the KPI speed; analytic velocity matches finite differences
  (`test_trajectories.py`). ✔

### T1.4 — Active 3D space & relative kinematics (ground truth) ✅
- **`src/interceptor/simulation/kinematics.py`** — `compute_relative_state` returns a
  `GroundTruthRelativeState` (rel pos/vel, range, LOS azimuth/elevation, analytic LOS
  rate, closing speed) using the `common/frames.py` conventions. The type is
  deliberately **not** a pipeline message — it must never leave the Simulation layer.
- **DoD:** validated against hand-computed geometries (target along ±X/±Y/+Z, crossing
  azimuth rate `= 0.2 rad/s`, closing speed signs); zero range fails loud
  (`test_kinematics.py`). ✔

### T1.5 — Sensor models: noise + latency ✅
- **`src/interceptor/simulation/sensors/noisy_sensor.py`** — `NoisyDelayedSensorModel`:
  per-channel Gaussian noise + bias + optional quantization, a finite **update rate**
  (sensor slower than sim), and a **latency** delay buffer that stamps each emitted
  sample with its true age. **Fails loud** if constructed without a profile, or with
  noise but no seed.
- **Config:** `SensorParams` added to `config/params.py` (YAML-overridable; never
  hard-coded).
- **DoD:** over 20 000 samples the measured−true residual mean ≈ bias and std ≈
  configured σ; emitted latency ≈ configured latency; value held between updates
  (`test_sensors.py`). ✔

### T1.6 — Wind & gust disturbance ✅
- **`src/interceptor/simulation/wind.py`** — `WindField`: steady wind + seeded
  Ornstein-Uhlenbeck gusts, **precomputed** so `velocity_at(t)` is a pure function of
  time (used by both the plant force and the wind-affected trajectory). `force_on` =
  `k·(v_wind − v_body)` via `WIND_DRAG_COEFF_N_PER_M_S`. Presets `calm`/`moderate`/`gusty`.
- **Config:** `WindParams` added to `config/params.py`.
- **DoD:** fixed seed ⇒ reproducible series; **calm preset reduces to undisturbed
  dynamics exactly** (`cmp` byte-identical hover with/without calm wind); gusts perturb
  a hover (`test_wind.py`, `test_calm_wind_matches_no_wind_exactly`). ✔

### T1.7 — Off-screen rendering ✅
- **`src/interceptor/simulation/rendering.py`** — `OffscreenRenderer` wraps
  `mujoco.Renderer` (off-screen only, **no GLFW window**), captures frames every Nth
  step (deterministic frame count), saves PNGs or buffers them, and has an `enabled`
  flag to disable rendering entirely.
- **DoD:** headless capture `(120, 160, 3)`; disabling rendering yields a **bit-identical
  physics endpoint** (pure observer) (`test_rendering.py`). ✔

### T1.8 — Solver configuration & numerical stability ✅
- **`models/scene.xml`** — `timestep = 1/SIM_HZ = 0.0025 s`, `integrator = RK4`,
  built-in aerodynamics off (`density/viscosity = 0`) since wind is modelled explicitly.
  `MujocoPlant` **asserts** the model timestep, mass, and step `dt` match the constants
  on load/step, so the XML and `constants.py` can never silently drift.
- **DoD:** no NaN/blow-up over long headless runs (30 s hover, 500-step determinism);
  free-fall matches gravity within 10 %; timestep justified with a `Why` comment
  (`test_mujoco_plant.py`). ✔

### T1.9 — Phase 1 unit tests ✅
- New suites: `test_actuators.py`, `test_trajectories.py`, `test_kinematics.py`,
  `test_sensors.py`, `test_wind.py` (pure-Python), and `test_mujoco_plant.py`,
  `test_rendering.py`, `test_replay.py` (MuJoCo-marked). Cover hover/no-drift, RPM
  saturation + coefficients, all five trajectories + determinism, LOS angle/rate,
  sensor noise statistics + latency, and wind reproducibility.
- **DoD:** all pass headlessly and deterministically — **107 passed** total. ✔

### T1.10 — Interactive replay viewer (opt-in) ✅
- **Pose-augmented log schema:** `orchestrator.LOG_FIELDS` extended with interceptor
  quaternion + target position (additive columns; downstream keys by name). The
  orchestrator logs attitude via an optional `orientation_quat` (stubs report identity).
- **`scripts/replay.py`** — `ReplaySession` loads `scene.xml` + a `run_log.csv` and
  drives the bodies to logged poses (`apply_frame`, `mj_forward` only — **no physics
  re-step, no ground truth**); `play()` opens the live `mujoco.viewer` window with
  real-time pacing, speed, and loop. The only sanctioned interactive window, opt-in and
  replay-only.
- **`scripts/run_sim_demo.py`** — Phase-1 utility that flies the real plant + weaving
  target and writes a replayable pose log (so there is something to watch before
  Phase 2 guidance exists).
- **DoD:** headless smoke test builds a session and applies frames without opening a
  window; logs lacking pose columns fail loud; the viewer never runs in CI
  (`test_replay.py`). ✔

---

## Deliverables produced

- `models/quadcopter.xml`, `models/target.xml`, `models/scene.xml`.
- `simulation/` additions: `actuators.py`, `mujoco_plant.py`, `kinematics.py`,
  `wind.py`, `rendering.py`, `trajectories/`, `sensors/`.
- `config/params.py`: `SensorParams`, `WindParams`; `config/constants.py`:
  `WIND_DRAG_COEFF_N_PER_M_S`.
- Pose-augmented run-log schema; `scripts/replay.py`; `scripts/run_sim_demo.py`.
- Eight new test modules (Phase 1 suite).

## Notes, decisions & deviations

- **Forces via `xfrc_applied`, not XML actuators.** The nonlinear `rpm²` thrust/drag
  mapping is computed explicitly in Python and applied as a world-frame body wrench.
  This keeps the actuator physics auditable in one place (`actuators.py`) and matches
  the kT/kQ constants exactly. No XML `<actuator>` is used.
- **World linear velocity by finite difference.** To stay frame-unambiguous, the plant
  derives world velocity from successive world positions rather than relying on free-
  joint `qvel` frame conventions; body **angular** rates come straight from the joint
  DOFs (the gyro analogue for the inner loop).
- **Airframe/motor constants unchanged.** The Phase 0 placeholders are already mutually
  consistent (hover ≈ 4952 RPM, well under the 25 000 RPM ceiling), so no KPI-affecting
  constant was altered — only the new wind coefficient and sensor/wind *params* were
  added. Finalizing any placeholder remains a user-confirmation action.
- **No guidance/control.** `run_sim_demo.py` hovers the quad while the target weaves; it
  is a Simulation-layer demo, not a guided interception. Closing the loop is Phase 2.

## Ready for Phase 2

The Simulation layer is complete behind the existing interfaces. Phase 2 (Roles 2/3/4)
can now inject the EKF, OGL/PN/APN, command limiter, and dual-loop control + mixer into
the orchestrator's `PipelineComponents`, replacing the pass-through stubs, with the real
`MujocoPlant`, `NoisyDelayedSensorModel`, trajectory generators, and `WindField` already
available — and `scripts/replay.py` ready to visualize the resulting interceptions.
