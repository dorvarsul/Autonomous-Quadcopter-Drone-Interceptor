# Phase 0 — Progress Report

> **Status: COMPLETE.** All exit criteria met. Implemented as Role 6 (Integration
> Architect). No algorithm logic was written (no EKF, guidance law, or control law) —
> only structure, contracts, constants, and infrastructure, exactly as Phase 0 scopes.

This report maps every Phase 0 task (`T0.1`–`T0.9`) to what was built, where it lives,
and how its Definition of Done was verified.

---

## How to reproduce the verification

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -e ".[dev]"

python scripts/check_env.py          # env doctor + headless smoke test -> exit 0
python scripts/run_stub_pipeline.py  # full 6-stage loop, headless, logged
pytest                               # 55 tests green
ruff check src tests scripts         # lint clean
```

**Observed results (this machine):**

- `check_env.py` → exit 0; rendered one frame **off-screen**, shape `(120, 160, 3)`,
  no GLFW window. MuJoCo 3.10.0, Python 3.13.12.
- `pytest` → **55 passed** in ~0.5 s.
- `ruff check` → **All checks passed**.
- Stub pipeline → 400 steps headless; two `seed=0` runs produced **byte-identical**
  `run_log.csv` (determinism confirmed via `cmp`).

---

## Exit criteria checklist (from phase0.md)

- [x] Python environment reproducibly installable; MuJoCo binding smoke test passes headless.
- [x] Repository layout from the implementation plan exists with package init files.
- [x] All shared constants live in `config/` with units + `Why` comments — zero magic numbers downstream.
- [x] All inter-layer data contracts (typed messages) and abstract interfaces defined and documented.
- [x] A stub orchestrator runs the full 6-stage loop headless and deterministically, writing a run log + config snapshot.
- [x] `pytest` runs green on contract/infra tests; everything is non-interactive.

---

## Task-by-task

### T0.1 — Environment & dependency setup ✅
- **`pyproject.toml`** — dependency manifest pinning `mujoco==3.10.0`, `numpy==2.5.0`,
  `scipy==1.18.0`, `pyyaml==6.0.3`, `matplotlib==3.11.0`; dev extra pins `pytest==9.1.1`
  and `ruff`. No dependencies beyond the approved list.
- **`scripts/check_env.py`** — env doctor: verifies Python ≥ 3.11, imports all required
  modules, prints the MuJoCo version, confirms the `C:/Dev/Libraries/mujoco/bin`
  resolution, then loads a trivial MJCF, steps it 100×, and renders one frame
  **off-screen**.
- **DoD:** `python scripts/check_env.py` exits 0 and produces a frame buffer headlessly. ✔

### T0.2 — Repository structure ✅
- Created `models/`, `scenarios/`, `results/`, `scripts/`, `tests/{unit,integration}`,
  and `src/interceptor/{config,common,simulation,estimation,guidance,control,pipeline,analysis}`,
  each with a documented `__init__.py`.
- **`README.md`** — setup, headless run/test instructions, layout, determinism notes.
- **`.gitignore`** — ignores `results/`, caches, virtualenvs, generated media.
- **DoD:** all package imports resolve; empty modules importable (covered by the test run). ✔

### T0.3 — Shared constants (single source of truth) ✅
- **`src/interceptor/config/constants.py`** — physics, airframe (placeholders),
  motors, loop rates (`SIM_HZ=400`, `INNER_LOOP_HZ=400`, `OUTER_LOOP_HZ=50`,
  `ESTIMATION_HZ=100`, `GUIDANCE_HZ=50`), guidance (`TILT_DELAY_TIME_CONSTANT_S`,
  `ALTITUDE_PENALTY_B=0.1`, nav-ratio terms), and all KPI thresholds. Every value
  carries explicit units in its name and a `Why` note. Placeholders are labelled.
- **`src/interceptor/config/params.py`** — runtime-tunable params (EKF Q/R, PID gains,
  nav-ratio schedule, limiter bounds) as dataclasses with safe defaults and a
  partial-merge YAML loader (`load_params`) that fails loud on unknown keys.
- **DoD:** no downstream module hard-codes these; values carry units + rationale. ✔
- **Constraint honored:** changing a physical constant / KPI-affecting tuning value
  remains a user-confirmation action (documented in code + this file).

### T0.4 — Coordinate frames & conventions ✅
- **`src/interceptor/common/frames.py`** — documents the **world frame (Z-up;
  altitude = +Z)**, body frame (FLU), quaternion-primary (`[w,x,y,z]`) /
  Euler-secondary (roll φ / pitch θ / yaw ψ) representation, and the **LOS azimuth /
  elevation / LOS-rate** sign conventions used by Estimation/Guidance. Provides
  `quat_to_rotation_matrix`, `world_to_body` / `body_to_world`, `euler_to_quat` /
  `quat_to_euler`, and `los_angles`. The altitude axis is called out explicitly
  (`ALTITUDE_AXIS`) given known overshoot sensitivity.
- **DoD:** documented, tested convention (see `tests/unit/test_frames.py`). ✔

### T0.5 — Pipeline data contracts (typed messages) ✅
- **`src/interceptor/common/types.py`** — one immutable, frozen dataclass per pipeline
  edge: `RawSensorMeasurement`, `TargetStateEstimate` (incl. covariance + quality),
  `AccelerationCommand`, `LimitedAccelerationCommand` (saturation flag/metric),
  `AttitudeReference`, `MotorCommand`. Every field documents its units; array fields
  are stored read-only.
- **`src/interceptor/common/guards.py`** — shared fail-loud guards
  (`ensure_finite`/`ensure_shape`/`ensure_vector`/`ensure_in_range`, `freeze`) and the
  `PipelineError` / `NumericalInstabilityError` / `ContractViolationError` hierarchy.
  Each message's `__post_init__` validates shape + finiteness and **raises on NaN/Inf**.
- **DoD:** exactly one message type per hand-off; no field outside a layer's contract. ✔

### T0.6 — Abstract interfaces ✅
- Narrow ABCs, one per swappable component, each with a docstring stating
  inputs/outputs/units and the owning role:
  - Simulation (Role 1): `TargetTrajectory`, `SensorModel`, `Renderer`, `Plant`
    (`simulation/interfaces.py`).
  - Estimation (Role 2): `Estimator` (`estimation/interfaces.py`).
  - Guidance (Role 3): `GuidanceLaw` with a `name` for KPI tables — PN/APN/OGL will be
    Liskov-substitutable (`guidance/interfaces.py`).
  - Control (Role 4): `CommandLimiter`, `OuterLoopController`, `InnerLoopController`,
    `MotorMixer` — loops kept **separate** so they stay distinct/at-rate
    (`control/interfaces.py`).
- A pass-through **stub** implements each interface (`*/stubs.py`) for T0.8.
- **DoD:** every interface documented + owned; a stub of each exists. ✔

### T0.7 — Determinism, RNG & logging ✅
- **`src/interceptor/common/rng.py`** — `RngFactory` hands out named, independent,
  order-independent RNG streams from one root seed; no global `random`/`np.random`.
- **`src/interceptor/common/logging.py`** — `RunLogger` (deterministic per-step CSV,
  fixed column order + float format) and `write_run_snapshot` (records seed, **git
  hash**, resolved params, metadata to `results/<run_id>/run_config.json`).
- Fail-loud guards (T0.5) are shared across layers.
- **DoD:** identical seed + config ⇒ byte-identical run log (verified by `cmp` and by
  `test_run_is_deterministic_byte_for_byte`). ✔

### T0.8 — Multi-rate scheduler & stub orchestrator ✅
- **`src/interceptor/pipeline/scheduler.py`** — `MultiRateScheduler` emits deterministic
  `Tick`s using **integer sim-step periods** (no float drift); inner 400 Hz, outer
  50 Hz, estimation/guidance cadences, sim step. Fails loud if a rate doesn't divide
  `SIM_HZ` or exceeds it. Loops are **not** collapsed.
- **`src/interceptor/pipeline/orchestrator.py`** — `StubOrchestrator` wires
  `SensorModel → Estimator → GuidanceLaw → CommandLimiter → outer → inner →
  MotorMixer → Plant`, via injected `PipelineComponents` (Dependency Inversion). Runs a
  fixed step count headless, enforces a headless renderer up front, and writes a run
  log + snapshot. The wiring respects the one-directional contract (guidance sees only
  estimates; control sees only the limited acceleration).
- **`scripts/run_stub_pipeline.py`** — CLI entry point.
- **DoD:** full 6-stage loop runs end-to-end on stubs, headless, deterministic, no
  contract violations, no GLFW window. ✔

### T0.9 — Test harness & quality gates ✅
- **`tests/conftest.py`** — fixtures: seeded RNG, tiny MJCF, isolated run dir.
- **Contract/infra tests (`tests/unit/`, `tests/integration/`):**
  - `test_types_contracts.py` — every message rejects NaN/Inf/bad-shape; immutability.
  - `test_frames.py` — rotation round-trips, LOS conventions, fail-loud paths.
  - `test_rng.py` — determinism + stream independence/order-independence.
  - `test_scheduler.py` — per-rate tick counts, drift guards, step-0 firing.
  - `test_interface_stubs.py` — each stub satisfies its ABC and emits valid output.
  - `test_params_logging.py` — YAML override merge, snapshot reproducibility.
  - `test_stub_pipeline.py` — full loop runs/logs; **byte-identical determinism**;
    **headless guarantee** (windowed renderer fails loud); no contract violations.
  - `test_mujoco_headless.py` — off-screen MuJoCo render (marked `mujoco`).
- **Lint/format:** `ruff` config in `pyproject.toml`; `ruff check` is clean.
- **DoD:** `pytest` green (55 passed); all tests non-interactive and headless. ✔

---

## Deliverables produced

- Reproducible env: `pyproject.toml` (pinned) + `scripts/check_env.py`.
- Full repo skeleton with package modules + `README.md`.
- `config/constants.py`, `config/params.py` (single source of truth).
- `common/{frames,types,rng,logging,guards}.py` (contracts, interfaces support infra).
- Per-layer `interfaces.py` + `stubs.py` for Simulation/Estimation/Guidance/Control.
- `pipeline/{scheduler,orchestrator}.py` + `scripts/run_stub_pipeline.py`.
- Passing `pytest` infrastructure suite (`tests/unit`, `tests/integration`).

## Notes, decisions & deviations

- **Loop rates pinned concretely:** `SIM_HZ=400` (= inner loop) so every slower rate
  divides evenly and the scheduler is drift-free; `ESTIMATION_HZ=100`, `GUIDANCE_HZ=50`
  added as named constants (phase0 left exact sensor/guidance rates open — these are
  documented placeholders, easily retuned in Phase 1/2).
- **`Plant` interface added** under Simulation (Role 1) to close the
  `Motor Mixer → Simulation` edge for the stub loop. It is sim-internal (not a new
  cross-layer message) and provides the gyro analogue the inner loop reads.
- **MuJoCo not required for the stub pipeline:** the orchestrator runs on pure-Python
  stubs, keeping the determinism test fast and GL-independent. The MuJoCo headless
  render is exercised separately by `check_env.py` and the `mujoco`-marked test.
- **Airframe/motor constants are explicit placeholders** for Role 1 to finalize in
  Phase 1; they are labelled as such and changing them is a user-confirmation action.

## Ready for Phase 1

The contracts and interfaces are stable slots. Phase 1 (Role 1) can now implement the
MuJoCo world, quad/target MJCF models, noisy/delayed sensor models, trajectory
generators, wind, and the real off-screen renderer **behind the existing interfaces**
without changing the orchestrator or downstream layers.
