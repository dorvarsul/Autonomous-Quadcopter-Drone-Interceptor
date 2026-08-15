# Autonomous Quadcopter Drone Interceptor

A simulated autonomous quadcopter that tracks, navigates toward, and intercepts
dynamic, evasive target drones in a 3D **MuJoCo** environment, using a **Classical
Hierarchical Architecture** (a 6-stage cyclic pipeline).

See [`AGENTS.md`](./AGENTS.md) for the full engineering contract and
[`docs/Autonomous_Drone_Interceptor_Design_Review.md`](./docs/Autonomous_Drone_Interceptor_Design_Review.md)
for the design rationale.

## The 6-stage pipeline

```text
Simulation ──(raw noisy/delayed sensor data)──► Estimation
Estimation ──(clean target pos, range, LOS rate)──► Guidance
Guidance   ──(required acceleration vector)──► Command Limiter
Cmd Limiter──(clamped, physically-safe accel)──► Flight Control (outer)
Outer loop ──(target roll/pitch/yaw + thrust)──► Flight Control (inner)
Inner loop ──(body torque + thrust)──► Motor Mixer
Motor Mixer──(4× rotor RPM)──► Simulation (actuators)
```

Each stage consumes **only** its predecessor's published output. Crossing a boundary
is a defect.

## Project layout

```text
src/interceptor/
  config/      # constants.py, params.py — single source of truth (no magic numbers)
  common/      # types (data contracts), frames, rng, logging, guards
  simulation/  # Role 1 — sensors, trajectories, renderer, plant (interfaces + stubs)
  estimation/  # Role 2 — Estimator interface + the EKF
  guidance/    # Role 3 — GuidanceLaw interface; OGL (the sole guidance law)
  control/     # Role 4 — command limiter, dual-loop control, motor mixer
  pipeline/    # Role 6 — multi-rate scheduler + orchestrator
  analysis/    # Role 5 — KPIs, scenarios, Monte-Carlo harness, reporting
scripts/       # check_env.py (env doctor), run_stub_pipeline.py, run_intercept.py, ...
tests/         # unit/ + integration/ — headless, non-interactive
models/ scenarios/ results/   # MJCF, scenario configs, generated run artifacts
```

## Setup (Windows)

MuJoCo is installed at `C:/Dev/Libraries/mujoco`. The pip `mujoco` wheel bundles its
own native libraries; appending `C:/Dev/Libraries/mujoco/bin` to `PATH` is only needed
for the standalone MuJoCo binaries.

```powershell
# From the project root:
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -e ".[dev]"      # installs pinned deps + pytest/ruff
```

## Verify the environment

```powershell
python scripts/check_env.py
```

This checks the Python version, imports, MuJoCo, and renders one frame **off-screen**
(headless smoke test — no GLFW window). Exit code 0 means ready.

> If off-screen rendering fails on a headless host, set a software/EGL GL backend:
> `set MUJOCO_GL=egl` (or `osmesa`) before running.

## Run the stub pipeline

```powershell
python scripts/run_stub_pipeline.py --steps 400 --seed 0
```

Runs the full 6-stage loop on pass-through stubs, headless and deterministically, and
writes a per-step run log + reproducibility snapshot to `results/<run_id>/`. This is the
skeleton that proves the loop closes; the real components inject behind the *same*
interfaces with no orchestrator change.

## Run the guided interception

```powershell
python scripts/run_intercept.py --target 8 3 6 --seconds 9   # stops at intercept (~4.9 s)
python scripts/replay.py results/intercept                   # top isometric view + trails
python scripts/replay.py results/intercept --view interceptor  # chase-cam view
```

Runs the full pipeline with the **real** components — MuJoCo plant, noisy/delayed sensor,
**EKF** estimation, **OGL** guidance (the sole guidance law), command limiter, dual-loop
(50 Hz / 400 Hz) control, and motor mixer — closing on a static target. It prints the
achieved miss distance and writes a replayable run to `results/<run_id>/`. Headless and
deterministic: same seed + config ⇒ byte-identical log.

The run **stops at closest approach** by default (`--seconds` is an upper bound;
`--no-terminate` flies the full duration). The replay viewer offers a `top` isometric
camera framed to keep both drones in view and an `interceptor` chase camera, each drawing
the interceptor (blue) and target (orange) **trajectory trails**; when not looping it
freezes on the intercept frame with the window left open.

## Run the KPI scenarios

```powershell
# Static/linear named scenarios and evasive/high-speed/wind stress probes:
python scripts/run_scenarios.py scenarios/ --report              # -> results/scenarios/
python scripts/run_scenarios.py scenarios/stress --results-dir results/stress

# Randomized 3D Monte-Carlo mission-success batch + final report/plots:
python scripts/run_montecarlo.py --trials 100 --seed 0 --report --results-dir results/montecarlo
```

`run_scenarios.py` flies each declarative scenario through the real closed loop and prints a
pass/fail KPI table; `run_montecarlo.py` samples a **seeded randomized 3D threat envelope**
(family, geometry, speed, wind) and reports the **Mission Success Rate**. Both are headless and
deterministic — a fixed seed reproduces byte-identical logs, and every run/batch writes a
config+seed+git-hash snapshot for reproducibility.

## Performance

Canonical seeded randomized 3D Monte-Carlo batch (100 trials, master seed 0) — reproduce with
the `run_montecarlo.py` command above:

| Metric | Target | Result | |
| :--- | :--- | :--- | :-: |
| Mission Success Rate (interception) | ≥ 90% | **95%** | ✅ |
| Max Target Speed intercepted | ≥ 83.6 km/h | **89.7 km/h** | ✅ |
| Miss Distance `R_miss` | ≤ 1.05 m | 95% compliance | ✅ |
| Z-Axis Overshoot | ≤ 0.5 m | 98% compliance (median ≈ 0.02 m) | ✅ |
| Time-to-Intercept | Static < 10 s / moving < 20 s | 95% compliance | ✅ |
| Command Saturation | ≤ 5% of flight time | 77% compliance | ❌ |

Interception by target family: static 21/21, linear 32/32, sinusoidal 34/35, varying-speed 8/12.
Wind robustness: 94% calm / 100% moderate / 93% gusty.

On the named scenario suites: **11/11** static/linear geometries meet every KPI (miss
0.003–0.037 m), and **9/11** evasive/high-speed/wind stress probes do — both stress failures
are saturation-only breaches (`sinusoidal_fast_juke`, `varying_speed_crossing_84kmh`); every
stress scenario still intercepts inside the 1.05 m miss KPI.

The interceptor tracks, navigates toward, and intercepts static, linear, **evasive (weaving)**,
and **high-speed (to 90 km/h)** targets, and holds up under wind/gust disturbance — all with the
Classical Hierarchical pipeline (no DRL).

**Known limitation — command saturation.** 23% of trials exceed the 5% saturation budget:
85+ km/h crossing targets intercepted from rest in ~2 s, where the engagement ends before the
airframe finishes accelerating. The metric counts the **whole actuator chain** (command limiter
**or** motor mixer) — an earlier limiter-only version reported a flattering 74% while the true
figure was ~40%. Fixing the measurement, then the physics (70° tilt cap, inner-loop
angular-acceleration clamp, attitude-priority mixer), brought it to 77%. Closing the remainder
needs adaptive-authority guidance logic, which is out of scope; see
[`docs/phase4_progress.md`](./docs/phase4_progress.md) for the finding.

## Run the tests (headless, non-interactive)

```powershell
pytest                      # everything
pytest -m "not mujoco"      # skip the off-screen GL render test
```

## Determinism & reproducibility

Identical seed + identical config ⇒ byte-identical run log. All randomness flows
through a single seeded RNG factory (`common/rng.py`); every run writes a
`run_config.json` snapshot recording the seed, resolved params, and git hash.
