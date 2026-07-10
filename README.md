# Autonomous Quadcopter Drone Interceptor

A simulated autonomous quadcopter that tracks, navigates toward, and intercepts
dynamic, evasive target drones in a 3D **MuJoCo** environment, using a **Classical
Hierarchical Architecture** (a 6-stage cyclic pipeline)


See [`AGENTS.md`](./AGENTS.md) for the full engineering contract and
[`docs/implementation_plan.md`](./docs/implementation_plan.md) for the phased plan.

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
  estimation/  # Role 2 — Estimator interface (EKF in Phase 2)
  guidance/    # Role 3 — GuidanceLaw interface; OGL (Phase 2)
  control/     # Role 4 — command limiter, dual-loop control, motor mixer
  pipeline/    # Role 6 — multi-rate scheduler + orchestrator
  analysis/    # Role 5 — KPIs, scenarios, reporting (Phase 3+)
scripts/       # check_env.py (env doctor), run_stub_pipeline.py
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

## Run the stub pipeline (Phase 0)

```powershell
python scripts/run_stub_pipeline.py --steps 400 --seed 0
```

Runs the full 6-stage loop on pass-through stubs, headless and deterministically, and
writes a per-step run log + reproducibility snapshot to `results/<run_id>/`.

## Run the guided interception (Phase 2)

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

> Phase 2 is *"correct, wired, and functioning"* — the loop intercepts static targets to
> well within the 1.05 m KPI. Formal KPI tuning (saturation ≤ 5%, moving/evasive targets)
> is Phase 3–4.

## Run the tests (headless, non-interactive)

```powershell
pytest                      # everything
pytest -m "not mujoco"      # skip the off-screen GL render test
```

## Determinism & reproducibility

Identical seed + identical config ⇒ byte-identical run log. All randomness flows
through a single seeded RNG factory (`common/rng.py`); every run writes a
`run_config.json` snapshot recording the seed, resolved params, and git hash.
