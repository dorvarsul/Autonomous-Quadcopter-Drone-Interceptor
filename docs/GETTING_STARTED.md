# Getting Started

A step-by-step guide for someone who has never seen this project before. Follow it
top-to-bottom and you will have installed the project, verified the environment, watched
an interception, and reproduced every published number — in about 15 minutes, most of
which is the machine running.

---

## 1. What you need

| Requirement | Notes |
| :--- | :--- |
| **Python 3.11+** | Developed and tested on 3.11 – 3.14. |
| **~1 GB disk** | Mostly the MuJoCo + SciPy + Matplotlib wheels. |
| **CPU only** | No GPU, no CUDA. Everything is CPU physics. |
| **No MuJoCo install** | The pip `mujoco` wheel bundles its own native libraries on Linux, macOS, and Windows. |
| **A display — optional** | Only the replay viewer (§5) opens a window. Every other command is headless. |

## 2. Install

```bash
# from the project root
python -m venv .venv

source .venv/bin/activate        # Linux / macOS
# .\.venv\Scripts\Activate.ps1   # Windows PowerShell

pip install -e ".[dev]"          # pinned runtime deps + pytest & ruff
```

`-e` (editable) installs the `interceptor` package from `src/` so `import interceptor`
works from anywhere. `[dev]` adds the test and lint tooling. All versions are pinned in
`pyproject.toml` — pinning is deliberate, because determinism is the project's core
guarantee.

## 3. Verify the environment

```bash
python scripts/check_env.py
```

This is the environment doctor. It checks the Python version, imports every dependency,
reports the MuJoCo version and GL backend, steps a trivial physics model, and renders one
frame **off-screen**. Expected output:

```text
=== Interceptor environment check ===
[ OK ] Python 3.14.5
[ OK ] Imported required modules: mujoco, numpy, scipy, yaml, matplotlib
[ OK ] MuJoCo version 3.10.0
[ OK ] GL backend MUJOCO_GL=(unset — MuJoCo picks a default)
[ OK ] Stepped trivial MJCF 100 times without error
[ OK ] Rendered one frame off-screen, shape=(120, 160, 3) (no GLFW window)
=== All checks passed ===
```

Exit code 0 means you are ready. If the render step fails, see [§8
Troubleshooting](#8-troubleshooting) — it is the only step with a host-specific
dependency, and nothing except the render test needs it.

## 4. Your first interception

```bash
python scripts/run_intercept.py --target 8 3 6 --seconds 9
```

Roughly 5 seconds of wall clock. It flies the complete pipeline — MuJoCo physics, a noisy
delayed sensor, the EKF, the Optimal Guidance Law, the command limiter, the 50 Hz/400 Hz
dual control loop, and the motor mixer — against a target hovering at (8, 3, 6) m, and
prints:

```text
Ran 2048 steps headlessly against target [8.0, 3.0, 6.0].
  min miss distance: 0.037 m at t=5.12 s  [HIT vs R_miss <= 1.05 m]
  run log:  results/intercept/run_log.csv
  snapshot: results/intercept/run_config.json
```

Lines beginning `command saturation: ...` are **not errors** — they are the limiter
honestly reporting that guidance asked for more acceleration than the airframe can
produce. Measuring that is a graded KPI.

Useful flags: `--start X Y Z` (interceptor start, default `0 0 2`), `--seed N`,
`--seconds S` (an upper bound; the run stops at closest approach), `--no-terminate` (fly
the full duration anyway), `--run-id NAME`, `--params FILE.yaml`.

## 5. Watch it (optional, needs a display)

```bash
python scripts/replay.py results/intercept                    # top isometric view
python scripts/replay.py results/intercept --view interceptor # chase camera
python scripts/replay.py results/intercept --speed 0.5 --loop # slow motion, looping
```

The viewer replays the recorded log — it re-renders history and cannot change any result,
which is why it is the one deliberately interactive tool in the project. Both drones draw
trajectory trails (interceptor blue, target orange); without `--loop` it freezes on the
intercept frame. Skip this entirely on a headless machine.

## 6. Reproduce the results

```bash
# 11 static & linear geometries, with a KPI table + per-scenario plots (~21 s)
python scripts/run_scenarios.py scenarios/ --report --results-dir results/scenarios

# 11 stress probes: evasive weaves, 84–90 km/h ramps, wind and gusts (~14 s)
python scripts/run_scenarios.py scenarios/stress --results-dir results/stress

# The headline number: 100 randomized 3D trials, seeded (~1 min 50 s)
python scripts/run_montecarlo.py --trials 100 --seed 0 --report --results-dir results/montecarlo
```

Each prints a table to the terminal and writes artifacts under `--results-dir`. Expected
output is in [RESULTS.md](./RESULTS.md) — mission success **95%**, max intercepted target
speed **89.7 km/h**, 11/11 named scenarios passing.

Note `run_scenarios.py` takes either a single `.yaml` file or a directory, and reads
`*.yaml` in that directory **without recursing** — that is why `scenarios/` and
`scenarios/stress` are two separate commands. (`scenarios/ablation/` holds a two-file pair
for the guidance altitude-penalty study.)

Start with fewer trials if you want a quick look: `--trials 10 --seed 0` takes ~10 s. The
headline rates are only meaningful at the full 100.

## 7. Reading the output

Every run writes a folder under `results/<run_id>/`:

| File | What it holds |
| :--- | :--- |
| `run_log.csv` | One row per simulation step: time, interceptor position + quaternion, target position, estimated range, commanded acceleration magnitude, saturation flags (`saturated` = limiter ∪ mixer, plus per-stage `limiter_saturated` / `mixer_saturated`), and the four rotor RPMs. |
| `run_config.json` | The reproducibility snapshot: root seed, fully resolved parameters, git hash (and, for a scenario, its resolved spec). |

Batch and suite runs add:

| File | What it holds |
| :--- | :--- |
| `kpi_summary.csv` | One row per scenario with each KPI and its pass flag (`run_scenarios.py --report`). |
| `<scenario>.png` | Per-scenario diagnostics: X-Y geometry, altitude vs. time with the overshoot band, range and command effort with saturated frames shaded. |
| `batch_kpis.csv` | One row per Monte-Carlo trial: family, wind preset, seed, and every KPI. |
| `batch_manifest.json` | Master seed, git hash, KPI targets, committed tuning, headline verdicts. |
| `batch_distributions.png` | Miss histogram, miss vs. target speed, interception by family, saturation histogram. |

The `results/` directory is generated output and is git-ignored — delete it freely.

**How to judge a run:** a *hit* is a miss distance ≤ 1.05 m. A scenario row can show ✅ on
the miss and still fail overall — the KPI table grades six metrics, and command saturation
is the strict one (see [RESULTS.md §4](./RESULTS.md#4-known-limitations)).

## 8. Troubleshooting

**Off-screen render fails / `gladLoadGL error` / no GL context.** Only affects the render
check and the `mujoco`-marked tests. Pick a software or headless backend:

```bash
export MUJOCO_GL=egl      # headless GPU
export MUJOCO_GL=osmesa   # pure software (needs the system osmesa/llvmpipe package)
# Windows PowerShell: $env:MUJOCO_GL="egl"
```

Or simply skip those tests: `pytest -m "not mujoco"`. No simulation result depends on
rendering.

**The replay viewer does nothing / crashes on a server.** It needs a real display. Use the
CSV logs and the generated plots instead.

**`ModuleNotFoundError: interceptor`.** The editable install did not take. Re-run
`pip install -e ".[dev]"` with the virtualenv activated.

**`pip` cannot resolve the pinned versions.** Your Python is probably older than 3.11 (or
much newer than the wheels available on your platform). Check `python --version` first.

**A run prints many `command saturation:` warnings.** Expected on aggressive geometries —
that is the honest saturation reporting, not a crash.

**A scenario file errors out immediately.** Intentional: the loader fails loud on unknown
keys, missing keys, a bad vector shape, or any guidance law other than `OGL`. The message
names the offending section.

**Runs feel slow.** Everything is single-process CPU physics: ~2 s per scenario, ~1 s per
Monte-Carlo trial, ~80 s for the full test suite. Reduce `--trials` while exploring.

## 9. Run the tests

```bash
pytest                    # 221 tests, ~80 s
pytest -m "not mujoco"    # skip the tests needing an off-screen GL context
pytest tests/unit         # fast component tests only
ruff check src tests scripts
```

Tests are headless, non-interactive, and seeded — including a test that asserts a
Monte-Carlo batch reproduces exactly.

## 10. Write your own scenario

Drop a YAML file anywhere and point the runner at it:

```yaml
name: my_scenario
seed: 0
target_class: moving        # "static" | "moving" — selects the time KPI bound
guidance_law: OGL           # OGL is the sole law; anything else is rejected
time_limit_s: 20.0
wind_preset: moderate       # optional: calm | moderate | gusty
interceptor:
  start_m: [0.0, 0.0, 2.0]
target:
  type: linear              # static | linear | sinusoidal | varying_speed
  start_position_m: [9.0, -5.0, 5.0]
  velocity_m_s: [0.0, 2.0, 0.0]
```

```bash
python scripts/run_scenarios.py my_scenario.yaml
```

Per-type target keys:

| `type` | Required keys |
| :--- | :--- |
| `static` | `position_m` |
| `linear` | `start_position_m`, `velocity_m_s` |
| `sinusoidal` | `start_position_m`, `drift_velocity_m_s`, `amplitude_m`, `frequency_hz` |
| `varying_speed` | `start_position_m`, `heading`, `initial_speed_m_s`, `peak_speed_m_s`, `ramp_duration_s` |

An optional `params:` block deep-merges onto the defaults, so you can override any tunable
(sensor noise, EKF, guidance, limiter, PID) for one scenario without touching the code:

```yaml
params:
  limiter:
    max_tilt_rad: 1.0472
```

`wind_preset` and an explicit `params.wind` both set the wind profile, so setting both is
rejected rather than silently letting one win.

## 11. Where to go next

- [README](../README.md) — the one-page overview and command reference.
- [RESULTS.md](./RESULTS.md) — measured performance, per-family breakdowns, known limits.
- [PROJECT_EXPLAINED.md](./PROJECT_EXPLAINED.md) — how every stage works, from scratch,
  assuming no control-theory background.
- [Design Review](./Autonomous_Drone_Interceptor_Design_Review.md) — why this architecture
  and this guidance law were chosen.
- [ENGINEERING_STANDARDS.md](./ENGINEERING_STANDARDS.md) — the rules the code is built to.
