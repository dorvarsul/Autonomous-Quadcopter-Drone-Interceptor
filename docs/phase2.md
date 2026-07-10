# Phase 2 — Estimation, Guidance, Limiter, Control & Motor Mixer

> **Roadmap window:** Jul 1 – Jul 15.
> **Primary roles:** Role 2 (Estimation), Role 3 (Guidance), Role 4 (Flight Control &
> Actuation); Role 6 wires the pipeline.
> **Pipeline stages owned:** Estimation → Guidance → Command Limiter → Flight Control
> (outer→inner) → Motor Mixer.
> **Goal:** Implement the full processing chain behind the Phase 0 interfaces and wire
> it into the live MuJoCo loop from Phase 1. By phase end, the closed loop intercepts a
> static target headless, with **OGL** as the **sole** guidance law. Parameter *tuning*
> and KPI *measurement* are Phase 3 — here the bar is "correct, wired, and functioning,"
> not "tuned to spec."
>
> **Scope decision (this project):** OGL is the only guidance law. PN and APN were
> evaluated and rejected in the Design Review (§6) and are **not** implemented. OGL uses
> the Zero-Effort-Miss (ZEM) formulation and folds in the estimated target acceleration,
> which subsumes the maneuvering-target capability APN would have provided.

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
- [ ] OGL implements the `GuidanceLaw` interface and is selected via config, staying
      decoupled from orchestration (a future law could be swapped in without caller edits).
- [ ] OGL incorporates the tilt-delay lag `1/(Ts+1)`, time-varying nav ratio `N'(t_go)`,
      and the altitude penalty `b` (default `0.1`). The augmented ZEM target-acceleration
      term is implemented but **gated off by default** (see T2.3): a relative-state EKF
      conflates the interceptor's own maneuver into `a_rel`, so it is deferred to Phase 4.
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

- [ ] Define the EKF state vector (relative position, relative velocity, and relative
      acceleration — the acceleration state feeds OGL's augmented ZEM term) and the
      continuous/discrete process model.
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

#### T2.2 — Guidance interface conformance + Time-to-Go + ZEM
**Role:** 3 · **Depends on:** T2.1 (consumes `TargetStateEstimate`)

- [ ] Implement a `time_to_go` estimator from range and closing velocity, well-conditioned
      when closing speed is ~0 (target static / interceptor at rest) — floor/cap `t_go`.
- [ ] Implement the **Zero-Effort-Miss** helper: `ZEM = R + V·t_go (+ ½·a·t_go²)` from the
      filtered relative position/velocity/acceleration.
- [ ] Conform strictly to `GuidanceLaw`; consume only filtered estimates.
- [ ] **DoD:** for a constant-bearing closing geometry ZEM ⟂ LOS ≈ 0; for a drifting LOS
      the perpendicular ZEM is non-zero and drives the command.

#### T2.3 — Optimal Guidance Law (OGL) — the sole guidance law
**Role:** 3 · **Depends on:** T2.2

- [ ] Implement OGL as the LQ optimization minimizing `J = y(t_f)² + ∫ u(t)² dt`, using
      the ZEM formulation `a_cmd = N'(t_go)/t_go² · ZEM`.
- [ ] Model the quadcopter's mechanical **tilt delay** via the first-order lag
      `1/(Ts+1)` using `TILT_DELAY_TIME_CONSTANT_S` — never assume instantaneous turns.
      The lag enters through the closed-form optimal gain `N'(t_go/T)`.
- [ ] Implement the **time-varying Navigation Ratio `N'`** driven by Time-to-Go (the
      lag-aware OGL gain schedule), numerically stable as `t_go/T → 0`.
- [ ] Implement the **altitude penalty `b`** (default `ALTITUDE_PENALTY_B = 0.1`) that
      suppresses Z-axis overshoot by de-weighting the Z-channel command.
- [ ] Provide the augmented **target-acceleration** ZEM term (`+ ½·a·t_go²`) behind a
      config switch (`use_target_acceleration`), **off by default**. Rationale: the
      Phase 2 EKF estimates *relative* acceleration `a_rel = a_target − a_interceptor`, so
      against non-maneuvering targets it mostly reflects the interceptor's own command;
      feeding that back is positive feedback and destabilizes the loop. Correctly isolating
      the target's absolute acceleration (using the known interceptor acceleration) is
      Phase 4 work for evasive targets — this is what replaces the rejected APN baseline.
- [ ] Keep OGL behind `GuidanceLaw`; select via config.
- [ ] **DoD:** OGL produces a finite, smooth `AccelerationCommand` over a nominal
      engagement; as `t_go → 0` behavior is well-conditioned (no singular blow-up); the
      Z channel shows no overshoot tendency in an open-loop check; from rest against a
      static target it generates a closing command (ZEM trajectory-shaping).

> **Boundary:** Guidance requests an *ideal* acceleration only. It must **not** clamp
> to physical limits (that is the Command Limiter) nor translate to tilt/motor commands.

### Role 4 — Command Limiter, Flight Control, Motor Mixer

#### T2.4 — Command Limiter (Safety)
**Role:** 4 · **Depends on:** T2.3 (consumes `AccelerationCommand`)

- [ ] Clamp the requested acceleration to physically safe bounds (max lateral/vertical
      accel, implied max tilt) so the drone stays stable and rotors are protected.
- [ ] Output `LimitedAccelerationCommand` with an explicit **saturation flag/metric**;
      this is the single source of saturation truth (KPI ≤ 5%).
- [ ] **Fail loud / log** each saturation event.
- [ ] **DoD:** requests within bounds pass through unchanged; out-of-bounds requests
      clamp to the boundary and mark saturation; no other layer duplicates clamping.

#### T2.5 — Flight Control outer loop (~50 Hz)
**Role:** 4 · **Depends on:** T2.4

- [ ] At `OUTER_LOOP_HZ = 50`, translate the clamped acceleration command into target
      roll/pitch tilt angles plus a thrust setpoint, accounting for gravity and the
      current yaw.
- [ ] Output an `AttitudeReference`.
- [ ] **DoD:** a commanded horizontal acceleration maps to the correct tilt direction
      and magnitude; hover command yields zero tilt and weight-compensating thrust.

#### T2.6 — Flight Control inner loop (~400 Hz PID)
**Role:** 4 · **Depends on:** T2.5

- [ ] At `INNER_LOOP_HZ = 400`, run a PID attitude controller using gyroscope feedback
      to drive actual roll/pitch/yaw toward the `AttitudeReference`, emitting a
      **`BodyTorqueThrustCommand`** (roll/pitch/yaw torque + thrust). This new message
      type replaces `AttitudeReference` on the inner-loop→mixer edge so torque is not
      smuggled through angle-named fields (contract change approved this phase).
- [ ] Keep this loop **distinct** from the outer loop and running at its own rate; the
      real tilt response emerges from these dynamics (do not shortcut the tilt delay).
- [ ] Provide default PID gains in `config/params.py` (tuned in Phase 3).
- [ ] **DoD:** the inner loop tracks step attitude references with stable, non-divergent
      response; rate separation from the outer loop is preserved.

#### T2.7 — Motor Mixer
**Role:** 4 · **Depends on:** T2.6, Phase 1 motor model

- [ ] Convert the `BodyTorqueThrustCommand` (roll/pitch/yaw torque + total thrust) into
      four individual rotor RPM values (inverse of the Phase 1 thrust/torque mapping).
- [ ] Enforce `MOTOR_RPM_MIN/MAX` saturation; on saturation, log and (per chosen
      policy) preserve attitude authority — never exceed physical motor boundaries.
- [ ] Output `MotorCommand` consumed by the simulation actuators.
- [ ] **DoD:** mixer is the numerical inverse of the motor model within tolerance for
      feasible commands; infeasible commands saturate loudly.

### Role 6 — Integration

#### T2.8 — Pipeline integration & timing
**Role:** 6 · **Depends on:** T2.1–T2.7

- [ ] Replace the Phase 0 stubs with the real components in `pipeline/orchestrator.py`,
      driven by the multi-rate scheduler: sim step / EKF & guidance cadence / outer
      50 Hz / inner 400 Hz. Pass each layer the correct elapsed `dt` for its own rate.
- [ ] Keep the guidance law selectable by config (OGL is the default and only law), so
      orchestration stays decoupled from the concrete law.
- [ ] Verify no layer reads outside its contract (no ground-truth in Est/Guid/Ctrl, no
      raw sensors in Guidance).
- [ ] **DoD:** the full loop runs headless and deterministically and **intercepts a
      static target** with the default OGL configuration.

### Tests

#### T2.9 — Component & integration unit tests
**Role:** owning role per component · **Depends on:** respective tasks

- [ ] EKF: convergence + bounded error + latency-compensation tests on synthetic tracks.
- [ ] OGL: analytic-behavior tests (well-conditioned near `t_go→0`, no Z overshoot
      tendency, ZEM ⟂ LOS ≈ 0 on constant-bearing closing, closing command from rest).
- [ ] Limiter: pass-through vs clamp + saturation-metric tests.
- [ ] Outer/inner loops: tilt-mapping and step-tracking tests at their rates.
- [ ] Mixer: inversion + RPM saturation tests.
- [ ] Integration: static-target interception smoke test (deterministic).
- [ ] **DoD:** all pass headlessly.

---

## Deliverables

- `estimation/ekf.py`; `guidance/{ogl,time_to_go,zem}.py`;
  `control/{command_limiter,outer_loop,inner_loop,motor_mixer}.py`; updated
  `pipeline/orchestrator.py`; new `BodyTorqueThrustCommand` +
  `relative_acceleration_m_s2` estimate field in `common/types.py`.
- Phase 2 unit + integration tests.

## KPIs Touched (functionally exercised; formally measured in Phase 3)

- Enables `R_miss`, `Time-to-Intercept`, `Z-overshoot` (via OGL `b`), and
  `Command Saturation` (via the Limiter's metric). Targets are *met* through Phase 3
  tuning.

## Risks & Mitigations

- **EKF divergence under noise/latency** → fail-loud guards (T2.1); defer aggressive
  tuning to Phase 3 but keep defaults stable.
- **Singularities near intercept (`t_go→0`)** → condition OGL/time-to-go per T2.2/T2.3 DoD.
- **Collapsing the two control loops** → explicit rate separation enforced in T2.5/T2.6
  and checked in T2.8.
- **Saturation handled in multiple places** → Limiter is the sole owner (T2.4).

## Boundaries (do not cross)

- Estimation never reads ground truth; Guidance never reads raw sensors; Guidance never
  clamps or converts to motor commands; saturation lives only in the Limiter/Mixer.

## References

- Design Review §5 (stages 2–6), §6 (OGL; PN/APN retained only as the rejected
  baselines), AGENTS.md → Roles 2/3/4/6, Pipeline Contract, Physics & Numerical
  Integrity. Thesis summary Ch. 3–4 (control + guidance).
