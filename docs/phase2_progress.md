# Phase 2 — Progress Report

> **Status: COMPLETE.** All exit criteria met. Implemented as Roles 2 (Estimation),
> 3 (Guidance), 4 (Flight Control & Actuation), and 6 (Integration). This phase fills in
> the processing chain behind the Phase 0 interfaces and wires it into the live MuJoCo
> loop from Phase 1: the closed loop now **intercepts a static target** headless and
> deterministically, with **OGL as the sole guidance law**.

> **Scope decision (this phase):** the project was narrowed to **OGL only** — PN and APN
> are **not** implemented. They remain in the Design Review / Thesis Summary only as the
> rejected evaluation baselines. `AGENTS.md`, `README.md`, `implementation_plan.md`, and
> `docs/phase{0,2,3,4}.md` were updated to match.

This report maps every (renumbered) Phase 2 task to what was built, where it lives, and
how its Definition of Done was verified.

---

## How to reproduce the verification

```powershell
.\.venv\Scripts\Activate.ps1
pytest                               # 149 passed (incl. MuJoCo interception + replay tests)
ruff check src tests scripts         # All checks passed

# Fly a real guided interception and (optionally) watch it:
python scripts/run_intercept.py --target 8 3 6 --seconds 9   # stops at intercept (~4.9 s)
python scripts/replay.py results/intercept                   # top isometric, both trails
python scripts/replay.py results/intercept --view interceptor  # chase cam
```

**Observed results (this machine):**

- `pytest` → **149 passed** in ~5.6 s (up from 107; +42 tests since Phase 1). The +7 over
  the original Phase 2 count of 142 are the post-Phase-2 refinements below (engagement
  termination + the two replay views/trails).
- `ruff check` → **All checks passed**.
- The wired pipeline intercepts static targets across varied 3D geometries to
  **0.01–0.04 m** miss distance (well within the `R_miss ≤ 1.05 m` KPI), in 3–6 s.
- A fixed seed + config produces a **byte-identical** `run_log.csv` across runs.

---

## Exit criteria checklist (from phase2.md)

- [x] EKF consumes **only** `RawSensorMeasurement`, compensates latency, and outputs a
      clean `TargetStateEstimate` (incl. LOS rate + covariance/quality). Never reads
      ground truth.
- [x] OGL implements the `GuidanceLaw` interface and is selected via config (sole law).
- [x] OGL incorporates the tilt-delay lag `1/(Ts+1)`, time-varying nav ratio `N'(t_go)`,
      and the altitude penalty `b` (default `0.1`). The augmented ZEM target-acceleration
      term is implemented but **gated off** (see T2.3 note).
- [x] Command Limiter clamps to safe bounds and is the **single** owner of saturation,
      exposing a saturation metric.
- [x] Dual-loop control runs as two distinct loops — outer ~50 Hz (accel→tilt) and inner
      ~400 Hz PID (gyro→torque) — not collapsed.
- [x] Motor Mixer maps body torque + thrust → four RPM within `MOTOR_RPM_MIN/MAX`.
- [x] The wired pipeline intercepts a **static** target headless and deterministically;
      no boundary violations.

---

## Contract & interface changes (approved this phase)

Two Phase-0 message contracts needed extending for real consumers; both were confirmed
with the user before implementing.

- **`TargetStateEstimate.relative_acceleration_m_s2`** — additive, defaulted to zeros so
  the Phase 0 pass-through estimator still satisfies the contract. Populated by the EKF;
  exposed for the (Phase 4) augmented guidance term.
- **`BodyTorqueThrustCommand`** (new message) — the inner-loop→mixer edge. Replaces
  `AttitudeReference` on that one edge so **torque is not smuggled through angle-named
  fields** (Clean Code → meaningful domain names). `InnerLoopController.track` now returns
  it and `MotorMixer.mix` consumes it; `common/types.py` pipeline diagram updated.

A pure `frames.los_rate_from_relative(rel_pos, rel_vel)` helper was added so the
Estimation layer derives LOS rate without importing the Simulation layer.

---

## Task-by-task

### T2.1 — Extended Kalman Filter ✅  (Role 2)
- **`src/interceptor/estimation/ekf.py`** — `ExtendedKalmanFilter`: a 9-state relative
  constant-acceleration model `[pos(3), vel(3), acc(3)]` in the world frame. Nonlinear
  measurement model (range + LOS az/el) with analytic Jacobian; predict/update with `Q`/`R`
  from `config/params.py`; **latency compensation** by keeping the state at measurement
  time and predicting forward by the sample's age for the published estimate; clean LOS
  rate + covariance-derived `quality`; **fails loud** on non-finite state or covariance-
  trace blow-up. Consumes only `RawSensorMeasurement` — never ground truth.
- **Config:** `EkfParams` redesigned for the 9-state model (per-group process noise,
  measurement variances defaulting to the Phase 1 sensor σ², divergence bound).
- **DoD:** on synthetic noisy tracks the position error stays bounded (< 0.5 m mean under
  0.3 m range noise), relative velocity is recovered, latency compensation matches truth-
  at-delivery, LOS rate tracks the analytic truth, and a tight divergence bound trips the
  fail-loud guard (`test_ekf.py`, 6 tests). ✔

### T2.2 — Time-to-go + Zero-Effort-Miss ✅  (Role 3)
- **`src/interceptor/guidance/time_to_go.py`** — `time_to_go_s` = `range / closing_speed`
  when closing, falling back to `range / reference_closing_speed` from rest (so OGL still
  synthesizes a closing command), clamped to `[t_go_min, t_go_max]` to bound the terminal
  `1/t_go²`. Plus `closing_speed_m_s`.
- **`src/interceptor/guidance/zem.py`** — `zero_effort_miss = r + v·t_go (+ ½·a·t_go²)`
  and `perpendicular_component` (the part LOS-nulling acts on).
- **DoD:** `t_go = 5 s` for a 10 m / 2 m·s⁻¹ closing geometry; reference-speed fallback and
  clamping verified; constant-bearing closing → zero perpendicular ZEM; drifting LOS →
  non-zero (`test_guidance.py`). ✔

### T2.3 — Optimal Guidance Law ✅  (Role 3)
- **`src/interceptor/guidance/ogl.py`** — `OptimalGuidanceLaw` behind `GuidanceLaw`:
  `a_cmd = N'(t_go/T) / t_go² · ZEM`. `lag_aware_nav_ratio` is the closed-form OGL gain
  for a first-order tilt lag (→ 3 far from intercept, clamped to `[nav_ratio_min, max]`
  near it, numerically guarded at `t_go/T → 0`). Altitude penalty `b` de-weights the Z
  channel (`1/(1+b)`) to suppress overshoot.
- **Augmented-ZEM note:** the target-acceleration term is provided behind
  `GuidanceParams.use_target_acceleration`, **off by default**. The EKF is a
  *relative-state* filter, so `a_rel = a_target − a_interceptor` mostly reflects the
  interceptor's own maneuver against non-maneuvering targets; feeding it back is positive
  feedback that destabilizes the loop. Correctly isolating the target's absolute
  acceleration is Phase 4 evasive-target work — this replaces the rejected APN baseline.
- **DoD:** name is `"OGL"`; nav ratio → 3 far out; command points toward a static target
  from rest; well-conditioned as `t_go → 0`; `b` de-weights Z monotonically; the augmented
  term is ignored by default (`test_guidance.py`, 12 tests). ✔

### T2.4 — Command Limiter ✅  (Role 4)
- **`src/interceptor/control/command_limiter.py`** — `AccelerationCommandLimiter`: clamps
  the horizontal command to the tilt bound `g·tan(max_tilt)` and the total magnitude to
  `max_acceleration_m_s2`, emitting `LimitedAccelerationCommand` with a `saturated` flag +
  magnitude removed. The **single** source of saturation truth; logs each event loudly.
- **DoD:** within-bounds passes through unchanged; over-magnitude and over-tilt requests
  clamp to the boundary and mark saturation (`test_control.py`). ✔

### T2.5 — Flight Control outer loop (~50 Hz) ✅  (Role 4)
- **`src/interceptor/control/outer_loop.py`** — `DifferentialFlatnessOuterLoop`: maps the
  clamped acceleration to a target attitude + thrust via quad differential flatness
  (`f = a_cmd + g·ẑ`, thrust `= m·|f|`, body +Z aligned with `f`), yaw held at 0.
- **DoD:** hover → level attitude + weight thrust `m·g`; `+X` accel → correct forward pitch
  `atan2(a,g)`; `+Y` accel → negative roll (FLU convention) (`test_control.py`). ✔

### T2.6 — Flight Control inner loop (~400 Hz PID) ✅  (Role 4)
- **`src/interceptor/control/inner_loop.py`** — `AttitudePidInnerLoop`: gyro-only strapdown
  attitude PD. It quaternion-integrates the body rates (seeded level) to maintain its own
  attitude, then commands `torque = inertia · (kp·att_err − kd·rate)`, emitting a
  `BodyTorqueThrustCommand`. Distinct from the outer loop, at its own rate — the resulting
  first-order-lag-like tilt response is exactly what OGL's lag model anticipates.
- **Config:** real inner-loop gains in `ControlParams` (roll/pitch `kp=300, kd=30`; **yaw
  `kp=2, kd=0.5`** — see Notes).
- **DoD:** zero error + zero rate → zero torque; a body rate is damped (D term opposes);
  a closed 1-DOF pitch loop drives the pitch to a step reference and settles without
  oscillation (`test_control.py`). ✔

### T2.7 — Motor Mixer ✅  (Role 4)
- **`src/interceptor/control/motor_mixer.py`** — `QuadMotorMixer`: the exact algebraic
  inverse of the Phase 1 rotor model. Solves the 4×4 allocation from `(thrust, roll,
  pitch, yaw)` to per-rotor thrusts, then `rpm = sqrt(f/kT)`. Reads the same shared
  constants as the forward model (DRY), enforces `MOTOR_RPM_MIN/MAX`, and logs infeasible
  (negative-thrust / over-ceiling) demands loudly.
- **DoD:** round-trips the actuator model to `< 1e-6` RPM; pure hover → four equal
  `hover_rpm`; an impossible thrust demand clamps within `[MIN, MAX]` (`test_control.py`). ✔

### T2.8 — Pipeline integration & timing ✅  (Role 6)
- **`src/interceptor/pipeline/orchestrator.py`** — `PipelineComponents.phase2_intercept(...)`
  wires the real components (MuJoCo plant + noisy/delayed sensor + EKF + OGL + limiter +
  dual-loop control + mixer) behind the *same* interfaces, so the orchestrator loop is
  unchanged (Open/Closed). MuJoCo is **lazy-imported** so the stub path stays native-dep
  free. The loop now passes each layer its own elapsed `dt` (estimation at `1/ESTIMATION_HZ`)
  and drives the target mocap so real runs are replayable.
- **`scripts/run_intercept.py`** — CLI entry point: flies a guided interception, prints the
  achieved miss distance vs the KPI, and writes a replayable run to `results/<run_id>/`.
- **DoD:** the full loop runs headless and deterministically and intercepts a static target
  with the default OGL config (`test_intercept.py`). ✔

### T2.9 — Component & integration tests ✅
- New suites: `tests/unit/test_ekf.py`, `test_guidance.py`, `test_control.py`
  (pure-Python) and `tests/integration/test_intercept.py` (MuJoCo-marked: 3 parametrized
  geometries + determinism + config). Existing `test_interface_stubs.py` updated for the
  new inner-loop/mixer signatures.
- **DoD:** all pass headlessly — **142 passed** total; ruff clean. ✔

---

## Deliverables produced

- `estimation/ekf.py`; `guidance/{ogl,time_to_go,zem}.py`;
  `control/{command_limiter,outer_loop,inner_loop,motor_mixer}.py`.
- `common/types.py`: `BodyTorqueThrustCommand` + `relative_acceleration_m_s2` field;
  `common/frames.py`: `los_rate_from_relative`.
- `config/params.py`: redesigned `EkfParams`, extended `GuidanceParams`
  (`use_target_acceleration`, t_go conditioning), real inner-loop `ControlParams` gains.
- `pipeline/orchestrator.py`: `phase2_intercept` factory + timing/mocap wiring;
  `scripts/run_intercept.py`.
- Four new test modules; updated `test_interface_stubs.py`.
- Doc updates for the OGL-only scope across `AGENTS.md`, `README.md`,
  `implementation_plan.md`, and `docs/phase{0,2,3,4}.md`.

## Notes, decisions & deviations

- **OGL only.** PN/APN dropped per user decision; the `GuidanceLaw` interface is retained
  (Open/Closed) so a future law could still be swapped in.
- **Augmented ZEM gated off.** Discovered that feeding the relative-state EKF's
  acceleration into the ZEM is positive feedback (it conflates the interceptor's own
  maneuver) and diverged every diagonal/steep intercept. With the classic `r + v·t_go` ZEM,
  all geometries hit dead-on. Correct target-acceleration isolation is Phase 4 work.
- **Yaw gains ~100× smaller than roll/pitch.** Yaw torque comes from rotor-drag
  differential (`kQ`), ~100× weaker than the arm-lever roll/pitch torque (`kT·arm`), so a
  normal-sized yaw gain demands impossible rotor differentials and saturates all four
  motors → tumble. This was the root cause of an early instability; fixing it (`kp_yaw=2`)
  unlocked clean interception. Yaw does not affect interception (the quad translates by
  tilting).
- **Interception is "functioning," not yet KPI-tuned.** Miss distance is already well under
  1.05 m. Command **saturation over the real engagement is ~7.5 %** (target `[8,3,6]`, seed 0)
  — modestly over the 5 % KPI, concentrated in the last few frames where the ZEM `1/t_go²`
  term peaks near intercept. (An earlier report of ~49.6 % was an artifact of measuring over
  a fixed-duration run: it counted ~4 s of physically meaningless *post-intercept flyby*,
  now removed by engagement termination — see the addendum.) Driving that terminal peak
  under 5 %, plus moving/evasive/windy targets, is Phase 3–4.
- **No physical constants changed.** Only tuning *params* (EKF `Q`/`R`, PID gains, guidance
  conditioning) were set; airframe/motor constants are untouched. The one added constant is
  `INTERCEPT_CAPTURE_RADIUS_M` (a run/termination bound, not an airframe property).

## Addendum — post-Phase-2 refinements (engagement termination + replay views)

Two follow-ups after the initial Phase 2 sign-off, prompted by watching a replay where the
interceptor hit the target cleanly and then flew on, lost stability, and diverged:

- **Engagement termination at closest approach (Role 5/6).** `StubOrchestrator.run` gained
  `terminate_on_intercept` (default **off**, so Phase 0/2 determinism tests are unchanged);
  `run_intercept.py` turns it **on** by default (`--no-terminate` to opt out). Once the true
  range enters `INTERCEPT_CAPTURE_RADIUS_M` (2.0 m) and starts increasing again, the loop
  stops with the **closest-approach frame as the last logged row**. Rationale: past intercept
  the target is *behind* the interceptor, OGL's geometry inverts (`t_go` collapses,
  `1/t_go²` saturates), and the drone thrashes — none of which is interception behavior.
  Removing that tail is what corrects the saturation figure (49.6 % → 7.5 %) and the reported
  time-to-intercept. `RunResult.num_steps` now reports the *actual* steps executed.
- **Replay viewer: two framed views + trajectory trails.** `scripts/replay.py` gained
  `--view {top,interceptor}`: `top` is a fixed isometric camera auto-framed to the whole
  engagement's bounding box (so both drones stay in frame — previously the free camera lost
  them), `interceptor` is a chase camera tracking the interceptor body. Both overlay the
  interceptor (blue) and target (orange) **trajectory trails**, drawn as decimated line
  segments that grow with playback. When not looping, playback now **freezes on the intercept
  frame with the window left open** for inspection instead of closing.
- **Tests:** `+7` headless tests (camera framing, trail decimation/append, intercept-frame
  detection; orchestrator termination stops at closest approach and trims the flyby tail).

## Ready for Phase 3

The full pipeline is wired and functioning behind the stable interfaces. Phase 3 (Role 5
driving; owning roles tuning) can now build the KPI-measurement + scenario tooling, run
seeded static/linear trials, tune EKF `Q`/`R`, the OGL `N'`/`b`/`T` schedule, and the PID
gains toward the KPI targets (miss distance, time-to-intercept, Z-overshoot, and getting
command saturation under 5 %), and lock passing scenarios as regression tests.
