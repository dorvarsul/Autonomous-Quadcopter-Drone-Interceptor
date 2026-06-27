# Implementation Plan — Autonomous Quadcopter Drone Interceptor

> **Index document.** This file is the entry point to the phased implementation
> plan. It summarizes scope, the chosen stack, the module/architecture layout, and
> links to the per-phase task breakdowns. It is derived from
> [`Autonomous_Drone_Interceptor_Design_Review.md`](./Autonomous_Drone_Interceptor_Design_Review.md)
> and operationalized under [`../AGENTS.md`](../AGENTS.md). When in doubt, defer to
> the Design Review.

---

## 1. Purpose & Scope

Build a **simulated autonomous quadcopter** that tracks, navigates toward, and
intercepts dynamic, evasive target drones in a 3D **MuJoCo** environment, using the
**Classical Hierarchical Architecture** (6-stage cyclic pipeline). Determinism,
valid physics, and explainability take priority over raw maneuverability. **No DRL /
learned black-box policies.**

The plan is a sequence of tasks for a coding agent. Each phase document lists tasks
(`T<phase>.<n>`) with subtasks, owning role, deliverables, and a Definition of Done
(DoD). Tasks are written so an agent can pick one up, complete it within its role's
boundaries, and verify it against explicit acceptance criteria.

## 2. Chosen Stack

| Concern | Decision |
| :--- | :--- |
| Language | **Python 3.x** |
| Physics | **MuJoCo** via the official `mujoco` Python bindings |
| Math | NumPy / SciPy (EKF, LQ/OGL optimization, PID) |
| Config | YAML/JSON scenario + parameter files |
| Tests | `pytest`, headless / off-screen rendering only |
| Plotting/Reports | `matplotlib` (+ CSV/Parquet run logs) |
| MuJoCo install | `C:/Dev/Libraries/mujoco` (see [`../AGENTS.md`](../AGENTS.md)) |

> Any new third-party dependency beyond this list must be confirmed with the user
> before adoption (per AGENTS.md → Workflow).

## 3. The 6-Stage Pipeline (North Star)

```text
Simulation ──(raw noisy/delayed sensor data)──► Estimation
Estimation ──(clean target pos, range, LOS rate)──► Guidance
Guidance   ──(required acceleration vector)──► Command Limiter
Cmd Limiter──(clamped, physically-safe accel)──► Flight Control (outer→inner)
Flight Ctrl──(roll/pitch/yaw/thrust)──► Motor Mixer
Motor Mixer──(4× rotor RPM)──► Simulation (actuators)
```

Each stage consumes **only** its predecessor's published output. Crossing a boundary
(e.g., Guidance reading raw sensors, or Control reading ground-truth target state) is
a defect.

## 4. Target Module Layout

This layout is the agreed single source of structure. Phase 0 creates it; later
phases fill it in. One module ↔ one pipeline stage / algorithm (Single Responsibility).

```text
Workshop_Autonomous_Systems/
├── docs/                         # design review, thesis summary, this plan
├── models/                       # MJCF (.xml): quad, target, scene
├── scenarios/                    # declarative scenario + tuning configs (yaml)
├── results/                      # logged runs, KPI tables, plots (generated)
├── scripts/                      # entry-point run scripts, env checks
├── src/interceptor/
│   ├── config/                   # constants.py, params.py — single source of truth
│   ├── common/                   # types (data contracts), frames, rng, logging
│   ├── simulation/      (Role 1) # world, sensors/, trajectories/, wind, renderer
│   ├── estimation/      (Role 2) # estimator iface, ekf
│   ├── guidance/        (Role 3) # guidance iface, pn, apn, ogl, time_to_go
│   ├── control/         (Role 4) # command_limiter, outer_loop, inner_loop, motor_mixer
│   ├── pipeline/        (Role 6) # orchestrator, multi-rate scheduler
│   └── analysis/        (Role 5) # kpis, scenarios, reporting
└── tests/                        # unit/ + integration/ (headless)
```

## 5. Phase Overview & Roadmap

| Phase | File | Roadmap Window | Primary Role(s) | Outcome |
| :--- | :--- | :--- | :--- | :--- |
| 0 | [phase0.md](./phase0.md) | (pre-Phase-1 setup) | Role 6 (Integration Architect) | Repo scaffold, env verified, shared constants, data contracts, interfaces, deterministic logging, end-to-end stub pipeline runs headless |
| 1 | [phase1.md](./phase1.md) | Jun 17 – Jun 30 | Role 1 (Simulation) | MuJoCo world, quad + target models, sensor noise/latency, trajectory generators, wind, off-screen rendering, opt-in interactive replay viewer (T1.10) |
| 2 | [phase2.md](./phase2.md) | Jul 1 – Jul 15 | Roles 2, 3, 4 | EKF, OGL (+PN/APN baselines), command limiter, dual-loop control, motor mixer, wired pipeline |
| 3 | [phase3.md](./phase3.md) | Jul 16 – Aug 5 | Role 5 (+ owning roles tune) | KPI + scenario tooling, static & linear trials, parameter tuning, PN/APN/OGL benchmark |
| 4 | [phase4.md](./phase4.md) | Aug 6 – Aug 20 | Role 5 (Test/KPI) | Sinusoidal/varying-speed/windy trials, randomized 3D Monte-Carlo, final KPI dataset |

## 6. KPI Success Criteria (5% Margin)

These are the acceptance bar for Phases 3–4 and the reference for all tuning.

| Metric | Success Target | Named Constant (config) |
| :--- | :--- | :--- |
| Miss Distance `R_miss` | ≤ 1.05 m | `R_MISS_MAX_M` |
| Time-to-Intercept | Static < 10 s; Moving < 20 s | `T_INT_STATIC_MAX_S`, `T_INT_MOVING_MAX_S` |
| Z-Axis Overshoot | ≤ 0.5 m above target | `Z_OVERSHOOT_MAX_M` |
| Command Saturation | ≤ 5% of total flight time | `CMD_SATURATION_MAX_FRAC` |
| Max Target Speed | ≥ 83.6 km/h | `MAX_TARGET_SPEED_MIN_KMH` |
| Mission Success Rate | ≥ 90% over randomized 3D trials | `MISSION_SUCCESS_MIN` |

## 7. Cross-Cutting Rules (apply to every phase/task)

- **Pipeline contract:** never bypass a neighbor; never read ground-truth state in
  Estimation/Guidance/Control.
- **Determinism:** seed all randomness; log full config + seed + git hash with every
  run so any result is reproducible.
- **No magic numbers:** every tuning value lives in `config/` with explicit units
  (m, m/s, rad, rad/s, RPM, Hz, km/h) and a `Why` comment referencing the Design
  Review section/equation.
- **Never assume instantaneous dynamics:** tilt delay `1/(Ts+1)` and the loop rates
  (50 Hz outer / 400 Hz inner / sim step) are first-class.
- **Respect physical limits everywhere:** motor RPM saturation and stability bounds
  hold on every code path; saturation stays measurable.
- **Fail loud:** surface saturation events, divergent EKF estimates, NaN/instability
  — never swallow them.
- **Headless:** every script/test must run with off-screen rendering and be
  non-interactive (no hanging GLFW window). **Sole exception:** the opt-in replay
  viewer (`scripts/replay.py`, Phase 1 T1.10) may open an interactive window, but only
  as a manual consumer of an already-logged run — it never runs in tests/batch/CI,
  never re-steps physics, and cannot affect any result.
- **Stay in role:** make changes within the working role's boundaries; if a fix needs
  another layer, file the finding rather than editing across the contract.
- **Confirm before:** adding dependencies, changing physical constants/tuning that
  affect KPIs, or altering layer interfaces.

## 8. How to Use These Documents (for the coding agent)

1. Complete phases in order; do not start a phase until the previous phase's **Exit
   Criteria** are met.
2. Within a phase, respect stated task dependencies; otherwise tasks may be done in
   any order.
3. For each task: declare the role you are acting as, implement only that task's
   scope, then verify against its **DoD** and add the listed tests.
4. Treat every checkbox as a discrete, reviewable unit of work.
