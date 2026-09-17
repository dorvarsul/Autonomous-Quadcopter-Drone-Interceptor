# Phase 2 — Estimation, Guidance, Limiter, Control & Motor Mixer

> **Roadmap window:** Jul 1 – Jul 15.
> **Primary roles:** Role 2 (Estimation), Role 3 (Guidance), Role 4 (Flight Control &
> Actuation); Role 6 wires the pipeline.
> **Pipeline stages owned:** Estimation → Guidance → Command Limiter → Flight Control
> (outer→inner) → Motor Mixer.
> **Goal:** Implement the full processing chain behind the Phase 0 interfaces and wire
> it into the live MuJoCo loop from Phase 1. By phase end, the closed loop intercepts a
> static target headless, with **OGL** as the operational default and **PN/APN** as
> interchangeable baselines. Parameter *tuning* and KPI *measurement* are Phase 3 — here
> the bar is "correct, wired, and functioning," not "tuned to spec."

---

## Entry Criteria

- Phase 1 complete: stable physics, motor model with RPM saturation, sensor models
  emitting noisy/delayed `RawSensorMeasurement`, trajectory generators, wind, headless
  rendering.
- Phase 0 interfaces (`Estimator`, `GuidanceLaw`, `CommandLimiter`, `FlightController`,
  `MotorMixer`) and message types available.

## Exit Criteria (Definition of Done for the phase)

- [ ] EKF consumes **only** `RawSensorMeasurement`, compensates latency, and outputs a
      clean `TargetStateEstimate` (incl. **LOS rate** and covariance/quality). It never
      reads ground truth.
- [ ] PN, APN, and OGL all implement `GuidanceLaw` and are **swappable by config** with
      no caller change (Liskov) — enabling apples-to-apples comparison later.
- [ ] OGL incorporates the tilt-delay lag `1/(Ts+1)`, time-varying nav ratio `N'(t_go)`,
      and altitude penalty `b` (default `0.1`).
- [ ] Command Limiter clamps to physically safe bounds and is the **single** owner of
      saturation handling, exposing a saturation metric.
- [ ] Dual-loop control runs as two distinct loops — **outer ~50 Hz** (accel→tilt) and
      **inner ~400 Hz** PID (gyro→torque) — not collapsed into one.
- [ ] Motor Mixer maps roll/pitch/yaw/thrust → four RPM within `MOTOR_RPM_MIN/MAX`.
- [ ] The wired pipeline intercepts a **static** target headless and deterministically;
      no boundary violations.

---

## Tasks

### Role 2 — Estimation

#### T2.1 — Extended Kalman Filter (EKF)
**Role:** 2 · **Depends on:** Phase 1 sensors

- [ ] Define the EKF state vector (e.g., relative position & velocity, and target
      acceleration as needed by APN/OGL) and the continuous/discrete process model.
- [ ] Implement the nonlinear measurement model mapping state → expected sensor
      readings (range, LOS angles) and derive the required Jacobians.
- [ ] Implement predict/update with `Q` (process) and `R` (measurement) covariances
      sourced from `config/params.py` (defaults now; tuning in Phase 3).
- [ ] **Latency compensation:** use each measurement's timestamp/age (Phase 1 latency)
      to fuse delayed data correctly (e.g., predict state forward to current time).
- [ ] Derive and expose a clean **LOS rate** and angular rates for Guidance, plus an
      estimate **quality/covariance** field.
- [ ] **Fail loud** on divergence (innovation/covariance blow-up, NaN) rather than
      emitting silent garbage.
- [ ] **DoD:** on synthetic noisy/delayed tracks (static, linear, sinusoidal) the EKF
      converges and its estimate error stays bounded; LOS rate tracks truth within
      tolerance. Never references ground-truth state.

### Role 3 — Guidance

#### T2.2 — Guidance interface conformance + Time-to-Go + PN baseline
**Role:** 3 · **Depends on:** T2.1 (consumes `TargetStateEstimate`)

- [ ] Implement a `time_to_go` estimator from range and closing velocity.
- [ ] Implement **Proportional Navigation (PN)**, partitioned into the three 2D
      sub-problems `Sxy`, `Sxz`, `Syz`, recombined into a 3D `AccelerationCommand`.
- [ ] Conform strictly to `GuidanceLaw`; consume only filtered estimates.
- [ ] **DoD:** for a constant-bearing closing geometry PN commands ≈ zero lateral
      acceleration; for a drifting LOS it commands acceleration to null the LOS rate.

#### T2.3 — Augmented Proportional Navigation (APN)
**Role:** 3 · **Depends on:** T2.2

- [ ] Implement APN: PN plus a feed-forward term on the **Zero-Effort-Miss** using the
      estimated target acceleration.
- [ ] **DoD:** with estimated target acceleration ≈ 0, APN reduces to PN (documented
      and tested); with maneuvering targets it adds the expected feed-forward.

#### T2.4 — Optimal Guidance Law (OGL) — operational default
**Role:** 3 · **Depends on:** T2.2

- [ ] Implement OGL as the LQ optimization minimizing `J = y(t_f)² + ∫ u(t)² dt`.
- [ ] Model the quadcopter's mechanical **tilt delay** via the first-order lag
      `1/(Ts+1)` using `TILT_DELAY_TIME_CONSTANT_S` — never assume instantaneous turns.
- [ ] Implement the **time-varying Navigation Ratio `N'`** driven by Time-to-Go.
- [ ] Implement the **altitude penalty `b`** (default `ALTITUDE_PENALTY_B = 0.1`) that
      suppresses Z-axis overshoot.
- [ ] Keep OGL interchangeable behind `GuidanceLaw`; select via config.
- [ ] **DoD:** OGL produces a finite, smooth `AccelerationCommand` over a nominal
      engagement; as `t_go → 0` behavior is well-conditioned (no singular blow-up); the
      Z channel shows no overshoot tendency in an open-loop check.

> **Boundary:** Guidance requests an *ideal* acceleration only. It must **not** clamp
> to physical limits (that is the Command Limiter) nor translate to tilt/motor commands.

### Role 4 — Command Limiter, Flight Control, Motor Mixer

#### T2.5 — Command Limiter (Safety)
**Role:** 4 · **Depends on:** T2.4 (consumes `AccelerationCommand`)

- [ ] Clamp the requested acceleration to physically safe bounds (max lateral/vertical
      accel, implied max tilt) so the drone stays stable and rotors are protected.
- [ ] Output `LimitedAccelerationCommand` with an explicit **saturation flag/metric**;
      this is the single source of saturation truth (KPI ≤ 5%).
- [ ] **Fail loud / log** each saturation event.
- [ ] **DoD:** requests within bounds pass through unchanged; out-of-bounds requests
      clamp to the boundary and mark saturation; no other layer duplicates clamping.

#### T2.6 — Flight Control outer loop (~50 Hz)
**Role:** 4 · **Depends on:** T2.5

- [ ] At `OUTER_LOOP_HZ = 50`, translate the clamped acceleration command into target
      roll/pitch tilt angles plus a thrust setpoint, accounting for gravity and the
      current yaw.
- [ ] Output an `AttitudeReference`.
- [ ] **DoD:** a commanded horizontal acceleration maps to the correct tilt direction
      and magnitude; hover command yields zero tilt and weight-compensating thrust.

#### T2.7 — Flight Control inner loop (~400 Hz PID)
**Role:** 4 · **Depends on:** T2.6

- [ ] At `INNER_LOOP_HZ = 400`, run a PID attitude controller using gyroscope feedback
      to drive actual roll/pitch/yaw toward the `AttitudeReference`, emitting
      roll/pitch/yaw torque + thrust commands.
- [ ] Keep this loop **distinct** from the outer loop and running at its own rate; the
      real tilt response emerges from these dynamics (do not shortcut the tilt delay).
- [ ] Provide default PID gains in `config/params.py` (tuned in Phase 3).
- [ ] **DoD:** the inner loop tracks step attitude references with stable, non-divergent
      response; rate separation from the outer loop is preserved.

#### T2.8 — Motor Mixer
**Role:** 4 · **Depends on:** T2.7, Phase 1 motor model

- [ ] Convert roll/pitch/yaw torque + total thrust into four individual rotor RPM
      values (inverse of the Phase 1 thrust/torque mapping).
- [ ] Enforce `MOTOR_RPM_MIN/MAX` saturation; on saturation, log and (per chosen
      policy) preserve attitude authority — never exceed physical motor boundaries.
- [ ] Output `MotorCommand` consumed by the simulation actuators.
- [ ] **DoD:** mixer is the numerical inverse of the motor model within tolerance for
      feasible commands; infeasible commands saturate loudly.

### Role 6 — Integration

#### T2.9 — Pipeline integration & timing
**Role:** 6 · **Depends on:** T2.1–T2.8

- [ ] Replace the Phase 0 stubs with the real components in `pipeline/orchestrator.py`,
      driven by the multi-rate scheduler: sim step / EKF & guidance cadence / outer
      50 Hz / inner 400 Hz.
- [ ] Make the active guidance law selectable by config to guarantee swap-without-edit
      (Liskov) for the Phase 3 benchmark.
- [ ] Verify no layer reads outside its contract (no ground-truth in Est/Guid/Ctrl, no
      raw sensors in Guidance).
- [ ] **DoD:** the full loop runs headless and deterministically and **intercepts a
      static target** with the default OGL configuration.

### Tests

#### T2.10 — Component & integration unit tests
**Role:** owning role per component · **Depends on:** respective tasks

- [ ] EKF: convergence + bounded error + latency-compensation tests on synthetic tracks.
- [ ] PN/APN/OGL: analytic-behavior tests (PN nulls LOS rate; APN→PN at zero target
      accel; OGL well-conditioned near `t_go→0`, no Z overshoot tendency).
- [ ] Limiter: pass-through vs clamp + saturation-metric tests.
- [ ] Outer/inner loops: tilt-mapping and step-tracking tests at their rates.
- [ ] Mixer: inversion + RPM saturation tests.
- [ ] Integration: static-target interception smoke test (deterministic).
- [ ] **DoD:** all pass headlessly; guidance-law swap leaves the harness unchanged.

---

## Deliverables

- `estimation/ekf.py`; `guidance/{pn,apn,ogl,time_to_go}.py`;
  `control/{command_limiter,outer_loop,inner_loop,motor_mixer}.py`; updated
  `pipeline/orchestrator.py`.
- Phase 2 unit + integration tests.

## KPIs Touched (functionally exercised; formally measured in Phase 3)

- Enables `R_miss`, `Time-to-Intercept`, `Z-overshoot` (via OGL `b`), and
  `Command Saturation` (via the Limiter's metric). Targets are *met* through Phase 3
  tuning.

## Risks & Mitigations

- **EKF divergence under noise/latency** → fail-loud guards (T2.1); defer aggressive
  tuning to Phase 3 but keep defaults stable.
- **Singularities near intercept (`t_go→0`)** → condition OGL/PN as in T2.4/T2.2 DoD.
- **Collapsing the two control loops** → explicit rate separation enforced in T2.6/T2.7
  and checked in T2.9.
- **Saturation handled in multiple places** → Limiter is the sole owner (T2.5).

## Boundaries (do not cross)

- Estimation never reads ground truth; Guidance never reads raw sensors; Guidance never
  clamps or converts to motor commands; saturation lives only in the Limiter/Mixer.

## References

- Design Review §5 (stages 2–6), §6 (PN/APN/OGL), AGENTS.md → Roles 2/3/4/6, Pipeline
  Contract, Physics & Numerical Integrity. Thesis summary Ch. 3–4 (control + guidance).
