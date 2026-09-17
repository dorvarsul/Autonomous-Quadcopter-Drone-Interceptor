# Engineering Standards

The rules the codebase is built to. Source comments cite this file (e.g.
*"fail loud (ENGINEERING_STANDARDS.md)"*); this is the document they mean. It is the
"how we build it" companion to the
[Design Review](./Autonomous_Drone_Interceptor_Design_Review.md) ("what we build") and
[PROJECT_EXPLAINED.md](./PROJECT_EXPLAINED.md) ("how it works, from scratch").

---

## 1. The pipeline contract

The interceptor is a **6-stage cyclic pipeline**. Each stage consumes **only** the
published output of its immediate predecessor:

```text
[1 Simulation] ──SensorMeasurement (noisy, delayed)──►
[2 Estimation] ──TargetEstimate (relative state + covariance)──►
[3 Guidance]   ──AccelerationCommand (ideal)──►
[4 Limiter]    ──LimitedAcceleration (physically safe)──►
[5a Outer ctl] ──AttitudeReference (roll/pitch/yaw + thrust)──►
[5b Inner ctl] ──BodyWrench (torques + thrust)──►
[6 Mixer]      ──MotorCommand (4 × rotor RPM)──► back into [1]
```

Rules:

- **No boundary crossing.** Guidance may not read raw sensor data; control may not read
  ground truth; the estimator may not peek at the simulator's true target state. A stage
  that reaches around its input is a defect, not a shortcut.
- **The messages are the contract.** Every arrow above is a frozen, validated dataclass in
  [`common/types.py`](../src/interceptor/common/types.py). They are immutable, carry
  explicit units, and validate shape/finiteness on construction.
- **Stages are injected, not imported.** Each stage is defined by an interface
  (`*/interfaces.py`); the orchestrator receives implementations through
  `PipelineComponents`. Swapping an implementation must require **zero** orchestrator
  changes — that is what makes the same loop run pass-through test doubles and the real
  MuJoCo/EKF/OGL stack alike.

### Role ownership

Stages map to roles; code comments cite them by number.

| Role | Owns | Lives in |
| :--- | :--- | :--- |
| 1 — Simulation | Physics plant, rotor/actuator model, sensors, target trajectories, wind, rendering | `simulation/` |
| 2 — Estimation / Perception | The EKF and the estimator interface | `estimation/` |
| 3 — Guidance | The Optimal Guidance Law, time-to-go, ZEM | `guidance/` |
| 4 — Flight Control & Actuation | Command limiter, outer/inner loops, motor mixer | `control/` |
| 5 — Test / Validation / KPI | KPI measurement, scenarios, Monte-Carlo harness, reporting | `analysis/`, `tests/` |
| 6 — Integration | Multi-rate scheduler, orchestrator, run logging, scripts | `pipeline/`, `scripts/` |

**Role 5 measures; it never tunes.** The test/KPI layer reports faithfully — it does not
edit guidance/control internals and does not relax a target to manufacture a pass. When it
finds a systemic failure it files a finding against the owning role
(see [RESULTS.md → Known limitations](./RESULTS.md#known-limitations)).

---

## 2. Fail loud, not silent

Bad data must stop the run, not propagate.

- `NaN`/`Inf` anywhere in a pipeline message raises immediately
  ([`common/guards.py`](../src/interceptor/common/guards.py)).
- EKF divergence (covariance blow-up past the configured trace bound) raises rather than
  emitting garbage estimates.
- Unknown scenario keys, unknown trajectory types, or a guidance law other than OGL raise
  instead of silently defaulting.
- A sensor with no noise/latency profile, or an unseeded random source, is rejected — a
  "clean" sensor would quietly invalidate the whole estimation problem.

## 3. Determinism and reproducibility

Identical seed + identical config ⇒ **byte-identical** run log. This is the project's
north star, not a nice-to-have.

- All randomness flows through the seeded RNG factory
  ([`common/rng.py`](../src/interceptor/common/rng.py)), which derives independent named
  streams (`sensor`, `wind`, …) from one root seed. `numpy.random` global state is never
  used.
- Stochastic processes that could depend on call order (e.g. the wind gust series) are
  **precomputed once** from their stream.
- Every run writes `run_config.json` — root seed, fully resolved parameters, and the git
  hash. Every Monte-Carlo batch additionally writes `batch_manifest.json` with the master
  seed, so `(master_seed, num_trials)` regenerates the batch exactly.
- Dependencies are **pinned** in `pyproject.toml`. Adding one is a deliberate, reviewed act.

## 4. Saturation must stay measurable

"Command saturation ≤ 5% of flight time" is a graded KPI, so saturation may never be
hidden inside a stage.

- The **command limiter is the only place** that clamps requested acceleration, and it
  reports both a flag and the magnitude it removed.
- The **motor mixer** reports its own saturation when a rotor would exceed the true
  actuator ceiling.
- The KPI counts **limiter ∪ mixer**; per-stage `limiter_saturated` / `mixer_saturated`
  columns are logged for attribution. Counting only one stage would let the airframe be
  pinned while the metric stayed green — that is the failure mode this rule exists to
  prevent.

## 5. Headless, non-interactive execution

Everything that can run automatically must run without a display or a human.

- No script, test, or library call may open a blocking GLFW window. The orchestrator fails
  loud if an interactive viewer is requested from an automated run.
- Plotting uses matplotlib's **Agg** backend explicitly.
- The one carve-out is [`scripts/replay.py`](../scripts/replay.py): an **opt-in** viewer
  that replays an already-recorded log. It reads no ground truth live, so it cannot affect
  any result.
- Off-screen rendering (`mujoco.Renderer`) is allowed anywhere and is covered by tests
  marked `mujoco`, which can be deselected on hosts with no GL context.

## 6. Configuration discipline

- [`config/constants.py`](../src/interceptor/config/constants.py) holds **physical and
  structural** facts (mass, arm length, rotor coefficients, loop rates, capture radius).
  They describe the airframe and are not tuning knobs.
- [`config/params.py`](../src/interceptor/config/params.py) holds **tunable** values (EKF
  noise, guidance gains, limiter bounds, PID gains, sensor noise profile).
- **No magic numbers downstream.** A module that needs a number reads it from config.
- Changing a KPI-affecting tuning value is a deliberate, documented change: it carries an
  inline rationale, requires the full suite to be re-run, and is reported honestly if it
  moves a metric.

## 7. Physics and numerical integrity

- Units are SI and are stated in every docstring and field name (`_m`, `_m_s2`, `_rad`).
- The world frame is Z-up; the body frame is X-forward / Y-left / Z-up. Rotations are
  quaternions; conversions live in [`common/frames.py`](../src/interceptor/common/frames.py)
  and nowhere else.
- The two control loops run at genuinely different rates (outer 50 Hz, inner 400 Hz) via
  the multi-rate scheduler. Collapsing them into one loop is forbidden — the rate
  separation is what makes the attitude dynamics honest.
- The rotor model maps commanded RPM to thrust/torque with a documented quadratic
  coefficient, and clamps are surfaced as saturation events rather than applied silently.

## 8. Testing standards

- `tests/unit/` covers components in isolation; `tests/integration/` covers the closed
  loop, the scenario suites, the Monte-Carlo batch, and the headless renderer.
- Tests are headless, non-interactive, and seeded. No test may depend on wall-clock timing
  or an open window.
- Tests requiring a GL context are marked `mujoco` (`pytest -m "not mujoco"` skips them).
- Determinism itself is under test: a batch re-run must reproduce its results.
- `ruff check src tests scripts` must be clean (line length 100; rules `E`, `F`, `I`,
  `B`, `UP`).

## 9. Coding standards

- Python ≥ 3.11, `from __future__ import annotations`, full type hints on public
  functions.
- Docstrings explain *why* and state units; physics code favors clarity over cleverness.
- Small, single-purpose modules with one owner role each.
- Public data structures are frozen dataclasses; mutation happens only inside the stage
  that owns the state.
