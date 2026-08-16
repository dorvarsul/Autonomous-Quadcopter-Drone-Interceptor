# Autonomous Quadcopter Drone Interceptor
## Complete Project Report — Architecture, Implementation, and Results

**Author:** Dor Varsulker
**Course:** Workshop in Autonomous Systems Simulation
**Simulator:** MuJoCo 3.10.0 (Python 3.13)
**Architecture:** Classical Hierarchical (6-stage cyclic pipeline) — explicitly *not* Deep RL
**Status:** All four roadmap phases complete; final measured results included.

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Problem Statement, Requirements and Constraints](#2-problem-statement-requirements-and-constraints)
3. [Architecture Decision: Classical vs. Deep Reinforcement Learning](#3-architecture-decision-classical-vs-deep-reinforcement-learning)
4. [Guidance Algorithm Selection: PN vs. APN vs. OGL](#4-guidance-algorithm-selection-pn-vs-apn-vs-ogl)
5. [System Architecture — The 6-Stage Pipeline](#5-system-architecture--the-6-stage-pipeline)
6. [Engineering Governance: Roles, Contracts and Boundaries](#6-engineering-governance-roles-contracts-and-boundaries)
7. [Stage 1 — Simulation Environment](#7-stage-1--simulation-environment)
8. [Stage 2 — Estimation (Extended Kalman Filter)](#8-stage-2--estimation-extended-kalman-filter)
9. [Stage 3 — Guidance (Optimal Guidance Law)](#9-stage-3--guidance-optimal-guidance-law)
10. [Stage 4 — Command Limiter](#10-stage-4--command-limiter)
11. [Stage 5 — Flight Control (Dual Loop)](#11-stage-5--flight-control-dual-loop)
12. [Stage 6 — Motor Mixer](#12-stage-6--motor-mixer)
13. [Integration: Scheduling, Orchestration, Termination](#13-integration-scheduling-orchestration-termination)
14. [Determinism, Configuration and Reproducibility](#14-determinism-configuration-and-reproducibility)
15. [Validation Methodology: KPIs, Scenarios, Monte-Carlo](#15-validation-methodology-kpis-scenarios-monte-carlo)
16. [Implementation Phases — What Was Built and What Was Learned](#16-implementation-phases--what-was-built-and-what-was-learned)
17. [Final Results](#17-final-results)
18. [Engineering Findings, Limitations and Future Work](#18-engineering-findings-limitations-and-future-work)
19. [Reproduction Guide](#19-reproduction-guide)
20. [Project Map](#20-project-map)
21. [Glossary](#21-glossary)

---

## 1. Executive Summary

This project delivers a **simulated autonomous quadcopter interceptor**: a counter-UAS
("counter-drone") system that autonomously detects, tracks, navigates toward, and
physically intercepts dynamic and evasive target drones inside a 3D MuJoCo physics
environment.

The interceptor receives **only noisy, delayed sensor measurements** — never ground
truth. From those it must estimate the target's motion, decide how to accelerate, and
translate that decision all the way down to four individual rotor RPM values, while
respecting real physical limits: finite motor speed, finite tilt authority, and the
fact that a quadcopter cannot change its attitude instantaneously.

The system is built as a **Classical Hierarchical Architecture** — six mathematically
explainable stages chained into a cyclic pipeline — rather than a single end-to-end
learned policy. This was a deliberate architectural decision in favour of
**determinism, valid physics, and explainability**.

### Headline result

Canonical **seeded randomized 3D Monte-Carlo batch** — 100 trials, master seed 0,
reproducible byte-for-byte from the committed code:

| KPI | Success Target | Measured | Verdict |
| :--- | :--- | :--- | :---: |
| Mission Success Rate (interception) | ≥ 90 % | **95 %** | ✅ |
| Max Target Speed intercepted | ≥ 83.6 km/h | **89.7 km/h** | ✅ |
| Miss Distance `R_miss` | ≤ 1.05 m | 95 % of trials | ✅ |
| Z-Axis Overshoot | ≤ 0.5 m | 98 % of trials (median ≈ 0.02 m) | ✅ |
| Time-to-Intercept | Static < 10 s / Moving < 20 s | 95 % of trials | ✅ |
| Command Saturation | ≤ 5 % of flight time | **77 %** of trials | ❌ (documented) |

Five of six KPIs pass. The sixth — command saturation — is reported as failing
**deliberately and honestly**: the metric was found to be undercounting, was corrected
so that the reported number got *worse*, and the residual tail was characterized and
filed rather than hidden. This is discussed in full in §16.5 and §18.

Supporting evidence: **221 automated tests** passing (headless, seeded); **11/11**
named static/linear scenarios meeting every KPI; **9/11** evasive/high-speed/wind
stress probes meeting every KPI (both exceptions are saturation-only breaches that
still intercept).

---

## 2. Problem Statement, Requirements and Constraints

### 2.1 Context

The rapid proliferation of commercial UAVs has created a genuine security problem, and
existing countermeasures are poorly matched to it. Missile-defence systems such as Iron
Dome are effective but cost on the order of $50 M per battery — vastly oversized and
overpriced for neutralizing a hobbyist quadcopter. A **cheap, expendable interceptor
quadcopter** that physically collides with the threat is a far better cost exchange.

The algorithmic inspiration comes from the missile-guidance literature (see
`docs/Andrea_Tini_Thesis_Summary.md`, Andrea Tini, University of Bologna, 2022/23),
which showed that classical interception algorithms *can* be translated to multi-rotor
drones — provided the missile-centric assumptions (non-zero constant velocity,
acceleration strictly perpendicular to the line of sight) are discarded.

### 2.2 Functional requirements

* **Target detection & tracking** — continuous real-time estimation of the target's
  relative position, range, and **Line-of-Sight (LOS) rate**.
* **Autonomous interception** — real-time guidance commands producing a reliable
  collision trajectory.
* **Autonomous flight control** — automatic translation of required acceleration
  vectors into motor speeds, maintaining stability even during aggressive maneuvers.

### 2.3 Technical constraints

* **Actuator saturation.** Motor RPM limits mean thrust vectoring must stay inside the
  physical motor envelope; exceeding it risks stall and loss of control.
* **Sensor noise & latency.** Real Radar/LiDAR/camera sensors are noisy and delayed.
  Raw data must be aggressively filtered *before* reaching the guidance loops.
* **Non-instantaneous attitude.** A quadcopter accelerates horizontally only by
  tilting, and tilting takes time. Every layer must respect this lag.

### 2.4 Success criteria (KPIs)

Six graded metrics, each carrying a 5 % engineering margin. These are the acceptance
bar for the whole project and every threshold lives as a named constant in
`config/constants.py` — no KPI number is hard-coded anywhere else.

| Metric | Description | Success Target | Named Constant |
| :--- | :--- | :--- | :--- |
| **Miss Distance** `R_miss` | Proximity required for neutralization | ≤ 1.05 m | `R_MISS_MAX_M` |
| **Time-to-Intercept** `t_int` | Time efficiency of the trajectory | Static < 10 s; Moving < 20 s | `T_INT_STATIC_MAX_S`, `T_INT_MOVING_MAX_S` |
| **Z-Axis Overshoot** | Altitude-leveling precision | ≤ 0.5 m above target | `Z_OVERSHOOT_MAX_M` |
| **Command Saturation** | Time pushed against physical limits | ≤ 5 % of flight time | `CMD_SATURATION_MAX_FRAC` |
| **Max Target Speed** | Fastest threat still defeated | ≥ 83.6 km/h | `MAX_TARGET_SPEED_MIN_KMH` |
| **Mission Success Rate** | Robustness over randomized 3D trials | ≥ 90 % interception | `MISSION_SUCCESS_MIN` |

### 2.5 Test scenario spectrum

| Scenario family | Purpose |
| :--- | :--- |
| **Static targets** | Baseline validation of algorithms and stability |
| **Linear moving targets** | Constant-velocity tracking and closing speeds |
| **Sinusoidal trajectories** | Evasive weaving — stress-tests EKF tracking and OGL responsiveness |
| **Varying target speeds** | Ramps up to 90 km/h — tests maximum physical constraints |
| **Wind & gusts** | Environmental disturbance — tests control-loop robustness |

---

## 3. Architecture Decision: Classical vs. Deep Reinforcement Learning

Two fundamentally different ways of mapping sensor data to motor commands were
evaluated.

### Architecture A — Classical Hierarchical *(selected)*

Decompose interception into distinct, specialized, mathematically explainable layers:

* **Estimation** — Extended Kalman Filter (EKF) processes noisy sensor data.
* **Guidance** — geometric guidance law computes required acceleration.
* **Control** — translates acceleration into roll/pitch targets and stabilizes.
* **Actuation** — motor mixer converts commands into four rotor RPMs within limits.

### Architecture B — Deep Reinforcement Learning *(rejected)*

Feed state information (interceptor orientation, relative target vectors) directly into
a deep neural network trained via PPO or SAC, which outputs low-level actuator commands
end-to-end.

### The verdict and its consequences

DRL can discover spectacular, "super-maneuverable" flight profiles. It was nevertheless
rejected for four concrete reasons:

| Concern | Why it disqualifies DRL here |
| :--- | :--- |
| **Simulation-quirk exploitation** | RL agents routinely find physically impossible tricks that work in the simulator but would stall a real motor. |
| **Black-box opacity** | You cannot explain *why* the policy acted — unacceptable for a safety-critical interceptor. |
| **Sensor-noise sensitivity** | Learned policies overreact to noise, producing erratic "twitching". |
| **Non-determinism** | Hard to reproduce, hard to certify, hard to regression-test. |

The classical approach trades a little raw agility for **determinism, valid physics and
explainability** — the project's stated architectural north star.

This decision is not merely documentation; it is *enforced*. Throughout the codebase it
manifests as: seeded randomness only, no magic numbers, physical limits respected on
every code path, and fail-loud behaviour on instability. Introducing a learned
black-box policy into any layer is defined as a forbidden change.

---

## 4. Guidance Algorithm Selection: PN vs. APN vs. OGL

The guidance law is the "brain" of interception. Three candidates were evaluated in the
design review.

### 4.1 Proportional Navigation (PN)

**Concept.** The foundational insight of all classical interception: *if the line of
sight to the target is not rotating, you are on a collision course.* PN therefore
commands acceleration proportional to the LOS rotation rate, driving that rotation to
zero and creating a "collision triangle" — aiming where the target *will be*, not where
it *is*.

**Limitations.** Assumes instantaneous turns (false for a quadcopter) and struggles
against maneuvering targets. For 3D drone flight it must be partitioned into three 2D
sub-problems (Sxy, Sxz, Syz).

### 4.2 Augmented Proportional Navigation (APN)

**Concept.** Adds a feed-forward term to the Zero-Effort-Miss calculation that
explicitly accounts for the target's evasive *acceleration*.

**Limitations.** If the target cruises at constant velocity (acceleration = 0), APN
degenerates to plain PN. It suffers severe Z-axis (altitude) overshooting and fails
against targets exceeding ~55–60 km/h.

### 4.3 Optimal Guidance Law (OGL) — **the winner**

**Concept.** Formulated as a **Linear Quadratic (LQ) optimization problem**
minimizing

```
J = y(t_f)² + ∫ u(t)² dt
```

i.e. simultaneously minimizing terminal miss distance *and* total control effort. Its
decisive advantage: it explicitly models the quadcopter's **mechanical tilt delay** as a
first-order lag transfer function `1/(T·s + 1)`, rather than pretending the drone can
turn instantly.

**Mathematical advantages.**
* Minimizes miss distance *and* control effort — saving battery and preventing violent
  maneuvers.
* Dynamically adjusts steering aggressiveness through a **time-varying navigation
  ratio** `N'` driven by time-to-go.
* An altitude penalty parameter `b` (default 0.1) de-weights the vertical channel to
  suppress the Z-overshoot that plagued PN/APN.

**Reported performance (design review / source thesis).** Tracks targets up to
**90 km/h**; reaches static targets roughly **12× faster** than PN/APN; eliminates
altitude overshooting.

### 4.4 Scope decision

**OGL is the sole guidance law implemented.** PN and APN were evaluated in the design
review and rejected; they exist in this project only as the historical evaluation that
selected OGL. The guidance interface (`GuidanceLaw`) is nevertheless kept clean and
narrow so a future law could be substituted without editing any caller — Open/Closed and
Liskov substitution are preserved even though only one implementation ships.

The augmented (target-acceleration) term that APN would have provided is *subsumed*
into OGL's ZEM formulation as an optional, gated feature (see §9.6).

---

## 5. System Architecture — The 6-Stage Pipeline

The interceptor's brain is a chain of six specialized stages, each consuming the
previous stage's output and producing the next stage's input, cycling many times per
second.

```
   ┌──────────────────────────────────────────────────────────────────────┐
   │                                                                      │
   ▼                                                                      │
[1 SIMULATION] ──raw noisy, delayed sensor measurement──►                 │
[2 ESTIMATION] ──clean relative position, range, LOS rate──►              │
[3 GUIDANCE]   ──ideal required acceleration vector──►                    │
[4 LIMITER]    ──clamped, physically safe acceleration──►                 │
[5a OUTER CTRL]──target roll/pitch/yaw + thrust──►                        │
[5b INNER CTRL]──body torques + collective thrust──►                      │
[6 MOTOR MIXER]──four rotor RPM values───────────────────────────────────┘
                     (fed back into the Simulation as actuator commands)
```

### 5.1 The pipeline contract (the golden rule)

**Each stage may read only its immediate predecessor's published output.**

* Guidance is *forbidden* from peeking at raw sensor data.
* Control is *forbidden* from reading the true target position.
* The estimator is *forbidden* from "cheating" with the simulator's ground truth.

Crossing a boundary is treated as a **defect**, not a shortcut. This is what makes every
stage independently unit-testable and the whole system explainable, and it is enforced
structurally by the type system (§6.2), not just by convention.

### 5.2 Loop rates

| Loop | Rate | Rationale |
| :--- | :--- | :--- |
| Physics integration | 400 Hz | MuJoCo timestep = 0.0025 s, RK4 integrator |
| Inner control loop (attitude PID) | 400 Hz | Fast stabilization from gyro feedback |
| Estimation (EKF / sensor cadence) | 100 Hz | The EKF runs at sensor rate, not control rate |
| Outer control loop + guidance | 50 Hz | Acceleration → tilt reference |

The two control loops are deliberately **distinct** and must never be collapsed into
one: the fast inner loop stabilizes the aircraft while the slower outer loop steers it.
This mirrors how real flight controllers are built.

---

## 6. Engineering Governance: Roles, Contracts and Boundaries

### 6.1 The six roles

Ownership mirrors the pipeline. Every change declares which role it acts as and stays
inside that role's boundaries.

| Role | Owns | Key boundary |
| :--- | :--- | :--- |
| **1 — Simulation Environment Engineer** | Physics, MJCF models, sensors, trajectories, wind | Must **not** clean up sensor noise for downstream convenience, nor alter physical constants to flatter KPIs |
| **2 — Estimation / Perception Engineer** | The EKF | Consumes only raw sensor output; never reads ground truth; does not compute guidance commands |
| **3 — Guidance Engineer** | OGL, ZEM, time-to-go | Consumes only filtered estimates; does **not** enforce physical limits (that is Role 4) |
| **4 — Flight Control & Actuation Engineer** | Command limiter, dual-loop control, motor mixer | Consumes only the acceleration vector; does not second-guess guidance strategy; owns all saturation handling |
| **5 — Test, Validation & KPI Engineer** | Scenarios, KPI measurement, Monte-Carlo | Does **not** modify algorithm logic to fix a failing test — files a finding to the owning role instead |
| **6 — Integration Architect** | Pipeline wiring, interfaces, loop timing, roadmap | Defines contracts; does not own any single layer's internal math |

Role 5's discipline is the one that most shaped the results: **measure faithfully, report
the ugly baseline, never tune a scenario to manufacture a pass.**

### 6.2 The data contracts

Each arrow in the pipeline is one specific, **immutable, self-validating** message
(`common/types.py`):

| Message | From → To | Carries |
| :--- | :--- | :--- |
| `RawSensorMeasurement` | Sim → Estimation | range, LOS azimuth/elevation, timestamp, latency |
| `TargetStateEstimate` | Estimation → Guidance | relative pos/vel/accel, range, LOS rate, covariance, quality |
| `AccelerationCommand` | Guidance → Limiter | the ideal (unclamped) acceleration |
| `LimitedAccelerationCommand` | Limiter → Control | clamped acceleration + saturation flag/magnitude |
| `AttitudeReference` | Outer → Inner | target roll/pitch/yaw + thrust |
| `BodyTorqueThrustCommand` | Inner → Mixer | three body torques + collective thrust |
| `MotorCommand` | Mixer → Sim | four rotor RPMs (+ saturation flag) |

Every message is a **frozen dataclass** whose arrays are made read-only, whose fields
document their units, and whose `__post_init__` validates shapes and **rejects NaN/Inf on
construction**. A stage literally cannot receive data across a boundary it isn't supposed
to — the architecture is enforced by the type system.

### 6.3 Coding standards

**SOLID.** One module ↔ one pipeline stage or algorithm (SRP). Guidance/estimation/
trajectory families sit behind narrow ABCs so new implementations plug in without
editing callers (OCP, LSP, ISP). The orchestrator depends on injected abstractions, not
concrete classes (DIP) — which is exactly why the *same* orchestrator loop runs the
Phase-0 stubs and the Phase-2+ real components with zero changes.

**Clean Code.** Domain vocabulary throughout (`los_rate`, `time_to_go`, `nav_ratio`,
`zero_effort_miss`, `tilt_delay`, `miss_distance`, `command_saturation`). Small focused
functions. **No magic numbers** — every tuning value lives in `config/` with explicit
units in its name and a `Why` comment referencing the design review. Comments explain
the *physics*, not the syntax.

**Fail loud, never silent.** Saturation events, diverging EKF estimates, NaN, and
instability are raised or logged loudly — never swallowed.

---

## 7. Stage 1 — Simulation Environment

**Role 1.** Files: `src/interceptor/simulation/`, `models/`.

This stage *is* the world: physics, the airframe, target motion, sensors, and wind.

### 7.1 MuJoCo model files

| File | Contents |
| :--- | :--- |
| `models/scene.xml` | World root: `timestep = 0.0025 s` (= 1/400 s), **RK4** integrator, ground plane, lighting. MuJoCo's built-in aerodynamics are turned **off** (`density=0 viscosity=0`) because wind is modeled explicitly and reproducibly in Python instead. |
| `models/quadcopter.xml` | The interceptor: "+"-configuration airframe, mass 1.0 kg, arm length 0.15 m, rotor sites in the fixed order `[front, right, back, left]`. Mass/inertia are written to match `constants.py` exactly — and the plant **asserts** the match on load, so XML and code can never silently drift. |
| `models/target.xml` | The target: a **mocap body** whose position is prescribed (teleported) each step by the trajectory generator rather than physically simulated — it is a threat to be tracked, not a body the interceptor's wake could disturb. |

### 7.2 The plant (`mujoco_plant.py`)

Each step the plant: takes four rotor RPMs → converts them to a body force+torque wrench
→ rotates into world coordinates → optionally adds a wind force → applies it and calls
`mujoco.mj_step` → exposes exactly what sensors and controllers are allowed to read
(world position, body angular rates as the gyro analogue, orientation quaternion, and
world velocity by finite difference).

It asserts the caller's `dt` equals the model timestep and checks for NaN after every
step.

> **Design note.** Forces are applied via `xfrc_applied`, not XML `<actuator>` elements.
> The nonlinear `rpm²` thrust/drag mapping is computed explicitly in Python so the
> actuator physics stays auditable in one place and matches the `kT`/`kQ` constants
> exactly.

### 7.3 Rotor actuator model (`actuators.py`)

A rotor spinning at `rpm` produces:

```
thrust_i = kT · rpm²      [N]   — lift along the body's +Z axis
drag_i   = kQ · rpm²      [N·m] — reaction torque about the vertical
```

with `kT = 1.0e-7 N/RPM²` and `kQ = 1.0e-9 N·m/RPM²`. Thrust grows with the **square** of
RPM. From four rotor thrusts the body wrench follows:

```
total thrust  =  f_front + f_right + f_back + f_left
roll torque   =  arm · (f_left  − f_right)
pitch torque  =  arm · (f_back  − f_front)
yaw torque    = −(kQ/kT) · (f_front − f_right + f_back − f_left)
```

Yaw authority is therefore ~**100× weaker** than roll/pitch, a fact that later turned out
to be critical for control tuning (§16.3).

This is the **physical actuator boundary**: incoming RPMs are clamped to
`[0, 25 000] RPM`, and any clamp is reported as a saturation event, never silently
swallowed. Hover sits at ≈ 4 952 RPM — comfortably inside the ceiling.

### 7.4 Sensor model (`sensors/noisy_sensor.py`)

Deliberately **corrupts** the perfect geometry to imitate a real radar/LiDAR, producing a
`RawSensorMeasurement` of `(range, azimuth, elevation)`:

| Corruption | Default |
| :--- | :--- |
| Gaussian noise, range | σ = 0.30 m |
| Gaussian noise, angles | σ = 0.0035 rad (≈ 0.2°) |
| Constant bias | 0.0 (configurable; systematic offset the filter cannot average away) |
| Quantization | disabled (configurable) |
| Finite update rate | 100 Hz (sensor slower than the 400 Hz sim) |
| Transport latency | 0.02 s, delivered via a delay buffer, each sample stamped with its true age |

The noise is **intentional and must remain configurable** — it is the entire reason the
estimator exists. Constructing a noisy sensor without a random seed is a fatal error,
because that would make the run irreproducible.

### 7.5 Target trajectory generators (`trajectories/generators.py`)

Five families behind the `TargetTrajectory` interface, each providing both position and
an *analytically exact* velocity:

| Generator | Motion |
| :--- | :--- |
| `StaticTrajectory` | Fixed point — the baseline test |
| `LinearTrajectory` | Constant velocity, `p(t) = p₀ + v·t` |
| `SinusoidalTrajectory` | Drift plus a 3D sine weave — the **evasive** stress case |
| `VaryingSpeedTrajectory` | Ramps from a start speed to a peak (25 m/s = 90 km/h), exact distance integral |
| `WindAffectedTrajectory` | Any base path pushed by the integral of a seeded wind field |

### 7.6 Wind model (`wind.py`)

Steady breeze plus random gusts, where the gusts use an **Ornstein–Uhlenbeck process** —
the standard way to generate *smooth, temporally correlated* randomness, because real
gusts drift and swirl rather than jitter like white noise. The whole gust series is
**precomputed once** from a seeded RNG, so `velocity_at(t)` is a pure, reproducible
function of time. Force on the airframe is `F = k·(v_wind − v_body)` with a documented
lumped drag coefficient.

Three presets: `calm`, `moderate`, `gusty`. **The calm preset produces exactly zero
wind** — bit-for-bit identical to undisturbed physics, which is verified by test.

### 7.7 Ground-truth kinematics (`kinematics.py`)

Computes the *true* engagement geometry (relative position/velocity, range, LOS
azimuth/elevation, analytic LOS rate, closing speed). **This is truth and it lives
strictly inside the simulation layer** — deliberately typed as a non-pipeline object so
it cannot leave. It is used only to *feed the sensors* (which then corrupt it) and to
*verify* results in tests.

### 7.8 Frames and conventions (`common/frames.py`)

* **World frame:** right-handed, **Z up** = altitude. The Z axis receives first-class
  attention everywhere because the design review flags it as overshoot-sensitive.
* **Body frame:** X forward, Y left, Z up (FLU).
* **Rotation:** **quaternions** `[w,x,y,z]` are primary (no gimbal lock, clean
  composition); Euler angles are secondary, used only for human-readable attitude targets.

LOS geometry, shared consistently by the kinematics, the sensor, and the estimator:

```
range          = |r|                              where r = target − interceptor
azimuth        = atan2(r_y, r_x)
elevation      = atan2(r_z, hypot(r_x, r_y))
azimuth_rate   = (r_x·v_y − r_y·v_x) / (r_x² + r_y²)
elevation_rate = (h·v_z − r_z·(dh/dt)) / range²,   h = hypot(r_x, r_y)
```

Critically, `frames.los_rate_from_relative` lets the **estimator** compute LOS rate from
its own *filtered* estimate without ever importing the simulation layer.

---

## 8. Stage 2 — Estimation (Extended Kalman Filter)

**Role 2.** File: `src/interceptor/estimation/ekf.py`.

**Job:** given only the noisy, delayed sensor stream, produce a clean, *current* estimate
of the target's relative position, velocity and acceleration, plus range and LOS rate —
together with a measure of confidence.

### 8.1 Why a Kalman filter

Two imperfect information sources exist: a **physics-based prediction** ("it was here
moving this fast, so now it is probably there") and a **noisy measurement**. The
prediction drifts; the measurement jitters. A Kalman filter is the mathematically optimal
recipe for blending them — it maintains both a running estimate *and* a running
uncertainty, and weighs prediction against measurement in proportion to how much it
trusts each. The blend weight is the **Kalman gain**.

The cycle is two steps forever: **predict** (advance the state, uncertainty grows) and
**correct** (fold in a measurement, uncertainty shrinks).

### 8.2 Why *Extended*

The sensor is **nonlinear**: it reports range and angles, which relate to Cartesian
position through square roots and `atan2`. The Extended Kalman Filter handles this by
**linearizing** — at each step computing the **Jacobian** (matrix of analytic partial
derivatives of `(range, azimuth, elevation)` with respect to position) around the current
estimate, and using that local linear approximation.

### 8.3 State and process model

The filter tracks the **relative** target state as 9 numbers in the world frame:

```
x = [ position(3),  velocity(3),  acceleration(3) ]
```

with a **constant-acceleration** process model (acceleration is a random walk driven by a
jerk PSD):

```
position     += velocity·dt + ½·acceleration·dt²
velocity     += acceleration·dt
acceleration += random walk
```

The interceptor's own maneuvers are unknown at this layer, so their effect is absorbed
into the process noise — a deliberate, documented, honest simplification.

Tuned covariances (`EkfParams`): process noise PSDs
`position 1e-3 / velocity 1.0 / acceleration 5.0`; measurement variances default to the
Phase-1 sensor σ² (`range 0.09 m²`, `angle 1.225e-5 rad²`).

### 8.4 The measurement update

1. **Predict** the state forward by the elapsed time, inflating the covariance by process
   noise.
2. Compute the expected measurement `h(x)` and the **innovation** = actual − expected.
   Angle innovations are **wrapped to (−π, π]** so a reading crossing the ±180° line does
   not manufacture a fake giant error.
3. Compute the **Kalman gain** from the Jacobian and the two covariances, and correct the
   state by `gain · innovation`.
4. Update the covariance with the **Joseph form** — a numerically stable rearrangement
   that keeps the covariance symmetric and positive-definite under floating-point
   rounding.

### 8.5 Latency compensation

The measurement is *old* (0.02 s by default). Acting on a 0.02-s-stale target position
would systematically aim behind the target. So the filter keeps its state at
*measurement time* and, before publishing, **predicts forward by the sample's stamped
age** — handing guidance an estimate valid *now*.

### 8.6 Fail-loud divergence guard

Kalman filters can diverge catastrophically. If any state becomes NaN/infinite, or the
covariance trace exceeds `divergence_covariance_trace_max` (1e9), the filter **raises**
rather than emitting nonsense. It also publishes a **quality score in [0,1]** derived
from position covariance, so guidance can reason about how much to trust the estimate.

**Measured performance:** against the 0.30 m range-noise / 0.02 s latency profile, mean
position error stays below 0.5 m, relative velocity is recovered, LOS rate tracks the
analytic truth, and estimation was verified **not** to be the accuracy bottleneck on the
static/linear suite (Phase 3, T3.5 — no retune was required).

---

## 9. Stage 3 — Guidance (Optimal Guidance Law)

**Role 3.** Files: `guidance/ogl.py`, `guidance/zem.py`, `guidance/time_to_go.py`.

**Job:** answer *"given where the target is and how it is moving, in which direction and
how hard should I accelerate to hit it?"* It outputs an **ideal** acceleration vector and
deliberately does **not** worry about physical limits — that is the next stage's job.

### 9.1 Don't chase — intercept

A naïve controller points straight at the target and chases it ("pure pursuit"). Against
a moving target this is terrible: you always aim where it *was*, curving in behind and
arriving late. Classical guidance instead nulls the LOS rotation: **constant bearing +
decreasing range = collision**.

### 9.2 Zero-Effort-Miss (ZEM)

ZEM answers: *"if I applied no further acceleration from now on, by how much would I
miss?"* It is the predicted miss vector:

```
ZEM = r + v·t_go + ½·a·t_go²
```

where `r`, `v`, `a` are the estimated relative position, velocity and acceleration, and
`t_go` is the time-to-go. The three terms are exactly the kinematic "predict future
position" formula. Guidance then drives ZEM toward zero:

```
a_cmd = N' / t_go² · ZEM
```

The `½·a·t_go²` term is the "augmented" part that anticipates a maneuvering target — it
is implemented but **gated off** (see §9.6).

### 9.3 Time-to-go conditioning

```
closing_speed = −(r·v)/|r|      (positive when approaching)
t_go          = range / closing_speed
```

Two singularities must be handled:

* **At launch the interceptor is at rest**, so closing speed ≈ 0 and `range/0 = ∞`.
* **Near intercept `t_go → 0`**, and the `1/t_go²` gain explodes.

Therefore `time_to_go` is carefully conditioned:

* If genuinely closing → `range / closing_speed`.
* If closing speed ≈ 0 (static target or from-rest launch) → synthesize `t_go` from a
  **reference closing speed** (tuned to **4.25 m/s**) so guidance still produces a
  sensible "start accelerating toward it" command.
* **Clamp** the result to `[0.05 s, 30 s]` to bound the terminal blow-up.

The reference closing speed is a documented trade-off: too high and the launch command
saturates the motors in the first ~20 % of flight; too low and the farthest static target
misses its 10 s budget. Its tuning history (5.0 → 3.5 → 4.25) is one of the project's
clearest examples of measurement-driven engineering (§16.6).

### 9.4 The lag-aware navigation ratio — what makes it *Optimal*

Plain PN ignores the fact that a real quadcopter cannot change tilt instantly; it turns
too gently near intercept and then cannot catch up. OGL solves the LQ optimization
*including* the first-order lag `1/(T·s+1)` (with `T = 0.2 s`), yielding the classic
closed-form time-varying navigation ratio in `x = t_go / T`:

```
N'(x) = 6·x²·(e^(−x) − 1 + x)
        ────────────────────────────────────────────────────
        2x³ − 6x² + 6x + 3 − 12·x·e^(−x) − 3·e^(−2x)
```

Behaviour:
* **Far from intercept** (`x` large) → `N' → 3`, i.e. it degenerates gracefully to
  classic PN.
* **Near intercept** (`x` small) → `N'` **grows**, steering harder *in advance* to
  pre-empt the tilt lag.

The result is clamped to `[3, 5]` for numerical safety. This is, in essence, a navigation
gain that *knows the airframe is sluggish and compensates ahead of time*.

### 9.5 The altitude penalty `b`

The design review flags altitude overshoot as a recurring interception failure mode. OGL
carries a tuning weight `b` (default **0.1**) that de-weights the vertical command —
formally arising from penalizing vertical control effort in the LQ cost, and appearing in
code as `a_cmd_z *= 1/(1 + b)`.

> **Honest measured finding (Phase 3 ablation).** With *this* project's
> differential-flatness outer loop, the airframe barely overshoots altitude at all, so
> `b`'s measured effect is near-zero: `b = 0.1` vs `b = 0` differ by < 0.005 m, and a
> purpose-built near-vertical climb overshoots < 0.002 m regardless of `b` swept 0 → 1.0.
> The only measurable effect of a larger `b` is a slight slowing of vertical closing.
> Rather than manufacture an effect, the null result was reported as such; `b` is retained
> at 0.1 as cheap insurance and for faithfulness to the design-review cost function.

### 9.6 The full law

```
r, v   = estimated relative position, velocity
a      = estimated relative acceleration        (gated off — see below)
t_go   = time_to_go(r, v)                        # conditioned & clamped
N'     = clamp(lag_aware_nav_ratio(t_go, T), 3, 5)
ZEM    = r + v·t_go + ½·a·t_go²
a_cmd  = N' / t_go² · ZEM
a_cmd[Z] *= 1/(1 + b)                            # altitude penalty
```

**Why the augmented term is gated off.** The EKF is a *relative-state* filter, so its
estimated acceleration is `a_target − a_interceptor`. Against a non-maneuvering target
that quantity mostly reflects the **interceptor's own maneuver** — feeding it back through
`½·a·t_go²` is positive feedback, and in Phase 2 it diverged on every diagonal and steep
intercept. With the classic `ZEM = r + v·t_go`, all geometries hit dead-on. Correctly
isolating the target's *absolute* acceleration remains the candidate next step for the
fast-crossing tail (finding F4-2), not a shipped feature.

---

## 10. Stage 4 — Command Limiter

**Role 4.** File: `control/command_limiter.py`.

Guidance may request accelerations the airframe physically cannot produce. The limiter is
the single place that clamps the *acceleration request*:

| Bound | Value | Physics |
| :--- | :--- | :--- |
| **Tilt limit** | max tilt 70° (1.2217 rad) → `g·tan(70°) ≈ 27.0 m/s²` | Horizontal acceleration comes *only* from tilting, so max horizontal accel = `g·tan(max_tilt)` |
| **Total magnitude limit** | 40 m/s² | Protects the rotors; well inside the airframe's ~250 m/s² collective-thrust capacity |

Every clamp is **reported** — a `saturated` flag plus the magnitude removed — and logged
with a warning, because *"command saturation ≤ 5 % of flight time"* is a graded KPI: being
pinned at a limit means control authority has been lost.

### 10.1 Tilt-limit tuning history

| Change | Phase | Reason |
| :--- | :--- | :--- |
| 35° → **45°** | 3 | At 35° horizontal authority was only 6.87 m/s²; cross-range dashes kept clamping |
| 45° → **60°** | 4 | At 45° (= exactly 1 g) chasing weaving and 90 km/h targets still clamped for 15–30 % of a short engagement — the dominant saturation miss. 60° gives 17.0 m/s² and lifted randomized mission success from ~57 % to 93 % |
| 60° → **70°** | F4-1 | Once thrust projection (§11.2) removed the tilt-lag Z-overshoot artifact, the extra authority became free. 70° gives 27.0 m/s², roughly halving honest saturation on the hardest crossers |

Going past 60° had originally been rejected because it *overshot* easy static targets —
but that overshoot turned out to be a **tilt-lag artifact in the thrust sizing**, not a
consequence of the tilt cap. Once the artifact was removed, static Z-overshoot at 70° was
verified unchanged (≤ 0.013 m). `max_acceleration_m_s2` was raised 30 → 40 along the way.

### 10.2 The saturation KPI covers the whole actuator chain

> Clamping an acceleration request here is one way to run out of authority; the **motor
> mixer** hitting a real rotor RPM ceiling is another, and it is just as much a loss of
> control. An earlier version of the KPI counted *only* the limiter, which let the airframe
> be saturated while the metric stayed green — precisely the hidden saturation the project
> forbids. The KPI now counts **limiter OR mixer**, with per-stage columns in the run log
> for attribution. The honest number is worse and correct. (Full story: §16.5.)

---

## 11. Stage 5 — Flight Control (Dual Loop)

**Role 4.** Files: `control/outer_loop.py`, `control/inner_loop.py`.

We have a safe *acceleration*. The drone does not accept "acceleration" as an input — it
accepts tilt and thrust, and ultimately motor speeds. Two nested loops bridge the gap:

* **Outer loop (~50 Hz):** "what tilt and thrust achieve this acceleration?"
* **Inner loop (~400 Hz):** "spin the body toward that tilt, fast, using the gyroscope."

### 11.1 Outer loop — differential flatness

A quadcopter is a **differentially flat** system: given a desired acceleration there is a
direct *algebraic* formula for the required tilt and thrust — no feedback gains needed.

The rotors push only along the body's +Z axis, so to produce a desired world-frame
acceleration `a_cmd`, the specific force the rotors must generate is:

```
f      = a_cmd + [0, 0, g]        (the vector the thrust must point along)
thrust = mass · |f|
roll   = atan2(−f_y, hypot(f_x, f_z))
pitch  = atan2(f_x, f_z)
yaw    = 0                        (held; yaw does not help interception)
```

For hover (`a_cmd = 0`) this gives `f = [0,0,g]` → zero tilt and thrust `= m·g`, exactly
cancelling weight. This clean, gain-free map is one reason the system overshoots altitude
so little (and hence why the `b` ablation returned a null result).

### 11.2 Inner loop — the attitude PD, plus two lag-driven refinements

A rate gyro measures *angular velocity*, not orientation. The controller therefore
maintains **its own attitude estimate by quaternion-integrating the gyro** (a strapdown
integration, seeded level at start), then runs a PD law:

```
attitude_error = desired_attitude − current_attitude
angular_accel  = kp · attitude_error − kd · body_rate
torque         = inertia · angular_accel
```

*(P pushes proportionally to error; D opposes the current rate to damp oscillation; the I
term is reserved and currently zero.)*

Gains: roll/pitch `kp = 300, kd = 30` (natural frequency ≈ 17 rad/s, well inside the
400 Hz loop). **Yaw gains are ~100× smaller** (`kp = 2, kd = 0.5`) on purpose: yaw torque
comes from the weak rotor-drag differential, so a normal-sized yaw gain demands impossible
rotor imbalances and saturates all four motors — this was the root cause of an early
tumbling instability, and fixing it unlocked clean interception.

**This is where the tilt delay becomes real.** Nothing fakes instant tilting; a real fast
controller runs and the attitude responds at its natural speed — which is precisely the
lag that OGL was designed to anticipate.

Two refinements sit on top of the PD law, both consequences of taking that lag seriously:

**(a) Angular-acceleration clamp.** A large attitude error (a hard reversal against a
weaving target) makes the P term demand ~360 rad/s² — far more torque than the rotors can
allocate around the collective thrust, so the mixer would clamp a rotor (actuator
saturation). The commanded angular acceleration is therefore capped at
`max_angular_accel_rad_s2 = 70 rad/s²` — the *hover-feasible ceiling* for this airframe,
derived as `arm · 2·(m·g/4) / I_xx = 0.15 · 4.9 / 0.01 ≈ 73 rad/s²` — scaling the whole
vector so the **slew axis is preserved**. A big slew becomes a rate-limited turn instead of
an infeasible demand. Small tracking errors stay far under the cap, so steady tracking and
OGL's lag model are unchanged.

**(b) Thrust projection (tilt-lag compensation).** The outer loop sizes collective thrust
as `m·|f|` assuming the body is *already* aligned with `f`. It is not — the tilt lags, and
during that lag the full thrust magnitude still points mostly *up*, so the surplus briefly
lifts the airframe. On a same-altitude lateral dash this produced ~0.48 m of Z-overshoot,
right at the 0.5 m KPI edge — and the altitude penalty `b` **cannot** fix it, because the
guidance Z-command is ≈ 0 there. It is a *control artifact*, not a guidance error. The fix
is standard geometric control — scale the collective by the projection of the desired
thrust axis onto the actual body +Z:

```
thrust = m·|f| · clip( dot(desired_axis, actual_body_z), 0, 1 )
```

so the *realized vertical* force stays ≈ the commanded one through the lag, and the term
becomes a no-op (projection → 1) once the tilt catches up. The inner loop already holds
the actual attitude, so this needs no new data and crosses no contract boundary. It cut
`static_lateral` Z-overshoot **0.476 → 0.373 m** — and, less obviously, it is what made the
70° tilt cap safe and the faster 4.25 m/s launch affordable.

Output: a `BodyTorqueThrustCommand` — three body torques plus the projected collective
thrust.

---

## 12. Stage 6 — Motor Mixer

**Role 4.** File: `control/motor_mixer.py`.

The final translation: "this much total thrust and these three torques" → **four rotor
RPMs**. It is the exact algebraic **inverse** of the rotor model in §7.3, and it reads the
*same* shared constants, so the forward model (physics) and inverse model (mixer) can
never disagree (DRY).

With `f_i = kT·rpm_i²`, the four forward equations of §7.3 form a 4×4 linear system in the
four rotor thrusts. The mixer solves it, then inverts `rpm = √(f / kT)`.

### 12.1 The two failure modes

**Over-ceiling.** A solution exceeding `MOTOR_RPM_MAX` is the true actuator ceiling: the
mixer clamps, **logs a saturation warning**, and flags the step as saturated. It never
silently exceeds actuator bounds.

**Negative thrust — solved by attitude-priority allocation.** A solution demanding
*negative* thrust from a rotor is physically impossible (a rotor cannot push down). This
happens at **low collective thrust**: when the drone is barely holding altitude but wants a
hard roll, the required torque differential exceeds the average thrust available to
distribute, driving one rotor below zero. Clipping it there silently destroys the torque
differential — the attitude command is quietly *not executed* — and this proved to be a
dominant source of real saturation on aggressive slews.

The key observation: in that regime the rotors are running at only ~5–7 k of 25 k RPM.
There is enormous headroom *above* and none *below*. So instead of clipping, the mixer
**raises the collective uniformly** until the minimum rotor reaches zero:

```
if min(thrusts) < 0:   thrusts += |min(thrusts)|
```

Adding the same amount to all four rotors leaves **every torque differential exactly
unchanged** (roll, pitch and yaw are all *differences* of rotor thrusts), so the attitude
command survives intact. The cost is a small transient thrust surplus that the outer loop's
next correction absorbs. Attitude authority is prioritized over instantaneous thrust
accuracy, spending RPM headroom that was going unused.

Because this path no longer loses authority, it is **no longer flagged as saturation** —
only the genuine ceiling is. The four RPMs return to the plant, closing the loop.

---

## 13. Integration: Scheduling, Orchestration, Termination

**Role 6.** Files: `pipeline/scheduler.py`, `pipeline/orchestrator.py`.

### 13.1 The multi-rate scheduler

Loop rates are converted into **integer periods in sim-steps** (the 50 Hz outer loop fires
every 8th step), so loops fire on exact step boundaries with **no floating-point drift**.
The scheduler refuses to construct if any rate fails to divide `SIM_HZ` evenly (fail loud).
Each `Tick` carries booleans stating which loops are due. This is a foundation of
bit-for-bit reproducibility.

### 13.2 The orchestrator

Each step it:

1. Reads interceptor position (plant) and target position (trajectory).
2. If estimation is due → take a sensor measurement, update the EKF.
3. If guidance is due → run OGL, then the command limiter.
4. If the outer loop is due → compute the attitude reference.
5. If the inner loop is due → run the attitude PD (gyro-fed), then the mixer.
6. Step the physics with the latest motor command.
7. Log one row: positions, orientation quaternion, estimate, per-stage saturation flags,
   RPMs.

Slower loops **reuse** their most recent output on steps where they do not fire.

Components are **injected** via `PipelineComponents` (dependency inversion), which is why
the *same* orchestrator runs the Phase-0 pass-through stubs and the Phase-2+ real stack
(MuJoCo, EKF, OGL, limiter, dual loop, mixer) with **zero changes** — only the injected
implementations differ.

### 13.3 Engagement termination at closest approach

By default a run stops at **closest approach**: once the interceptor comes within the
capture radius (2.0 m) and the range then starts *growing*, the intercept moment has
passed. Flying on is physically meaningless thrashing — the target is now behind the
interceptor, OGL's geometry inverts, `t_go` collapses, and the `1/t_go²` term saturates.

This was discovered by watching a replay in which the interceptor hit cleanly and then
flew on, lost stability and diverged. Removing that tail also **corrected the KPI
measurements**: reported saturation for a representative run fell from a meaningless
49.6 % (dominated by ~4 s of post-intercept flyby) to a genuine **7.5 %** over the real
engagement.

---

## 14. Determinism, Configuration and Reproducibility

**The guarantee: identical seed + identical config ⇒ byte-identical run log, on any
machine.** Three mechanisms enforce it.

| File | Role |
| :--- | :--- |
| `common/rng.py` | **All** randomness flows through one seeded `RngFactory`, which hands each stochastic component (sensor, wind) its own **named, independent, order-independent** stream. Adding a new random component never disturbs existing ones. Calling global `random`/`np.random` is forbidden. |
| `common/logging.py` | One CSV row per timestep with fixed column order, fixed float formatting and `\n` newlines; plus a `run_config.json` snapshot recording **seed, fully resolved parameters, and git commit hash** for every run. |
| `common/guards.py` | Fail-loud validators (`ensure_finite`, `ensure_vector`, `ensure_in_range`, `freeze`) used everywhere to reject bad data at the boundary. |

### 14.1 Constants vs. parameters — a deliberate split

**`config/constants.py` — physical & structural.** Gravity, air density, mass/inertia,
arm length, motor RPM limits, `kT`/`kQ`, loop rates, the tilt-delay time constant, and
every KPI threshold. Units are explicit *in the name* (`MOTOR_RPM_MAX`,
`TILT_DELAY_TIME_CONSTANT_S`) and each carries a `Why` comment. Changing one of these
would mean *a different drone*.

**`config/params.py` — tunable knobs.** EKF covariances, PID gains, navigation-ratio
bounds, reference closing speed, limiter bounds, sensor noise profile, wind profile. Plain
dataclasses that serialize cleanly into the reproducibility snapshot. A **YAML scenario
file can override any of them** via deep merge — and a typo'd key **fails loud** instead of
silently doing nothing.

The two are governed differently: changing a KPI-affecting default requires explicit user
sign-off, and every such change in this project carries an inline rationale referencing the
report that justified it.

### 14.2 Key committed values

| Parameter | Value | Note |
| :--- | :--- | :--- |
| `QUAD_MASS_KG` | 1.0 | Hover ≈ 4 952 RPM of 25 000 |
| `ARM_LENGTH_M` | 0.15 | |
| `MOTOR_RPM_MAX` | 25 000 | Hard actuator ceiling |
| `THRUST_COEFF_KT` / `TORQUE_COEFF_KQ` | 1.0e-7 / 1.0e-9 | Yaw ~100× weaker than roll/pitch |
| `SIM_HZ` / `INNER_LOOP_HZ` | 400 / 400 | |
| `OUTER_LOOP_HZ` / `GUIDANCE_HZ` / `ESTIMATION_HZ` | 50 / 50 / 100 | |
| `TILT_DELAY_TIME_CONSTANT_S` | 0.2 | The `T` in OGL's `1/(Ts+1)` |
| `ALTITUDE_PENALTY_B` | 0.1 | Design-review default |
| `NAV_RATIO_MIN` / `MAX` | 3.0 / 5.0 | |
| `INTERCEPT_CAPTURE_RADIUS_M` | 2.0 | Termination arming radius |
| `reference_closing_speed_m_s` | 4.25 | Tuned 5.0 → 3.5 → 4.25 |
| `max_tilt_rad` | 1.2217 (70°) | Tuned 35° → 45° → 60° → 70° |
| `max_acceleration_m_s2` | 40 | Tuned 30 → 40 |
| `max_angular_accel_rad_s2` | 70 | Added in F4-1 |
| Sensor σ (range / angle) | 0.30 m / 0.0035 rad | Latency 0.02 s @ 100 Hz |

---

## 15. Validation Methodology: KPIs, Scenarios, Monte-Carlo

**Role 5.** Files: `analysis/kpis.py`, `analysis/scenarios.py`, `analysis/montecarlo.py`,
`analysis/reporting.py`, `tests/`, `scenarios/`.

### 15.1 KPI measurement (`kpis.py`)

The single source of truth for turning a raw `run_log.csv` into graded metrics. It **never
re-runs physics** — it only *measures* a recorded run. Every threshold is pulled from
`config/constants.py`. Two subtleties matter:

* **Z-overshoot is measured sign-aware.** For a climbing intercept it is how far the
  interceptor rose *above* the target; for a descending one, how far it sank *below*. This
  fixed a metric artifact where a descending approach counted its benign initial altitude
  gap as 3.4 m of "overshoot".
* **Command saturation counts the whole actuator chain.** A frame is saturated if the
  *limiter* clamped **or** the *mixer* hit a rotor RPM limit. The run log carries
  `limiter_saturated` and `mixer_saturated` columns alongside the combined flag so any
  breach can be attributed to a stage.

### 15.2 Declarative scenarios (`scenarios.py` + `scenarios/*.yaml`)

A scenario is a small YAML file fully specifying one trial — seed, target trajectory,
interceptor start, time limit, optional parameter overrides, optional `wind_preset`, and
(always) OGL:

```yaml
name: static_diagonal
seed: 0
target_class: static
guidance_law: OGL
time_limit_s: 12.0
interceptor:
  start_m: [0.0, 0.0, 2.0]
target:
  type: static
  position_m: [8.0, 3.0, 6.0]
```

The runner reuses the existing trajectory generators and the exact same closed loop — it
only *declares and drives*, containing no physics logic of its own. It fails loud on
unknown trajectory types, missing keys, a malformed vector, or a non-OGL law.

**The scenario library:**

| Group | Count | Contents |
| :--- | :--- | :--- |
| Static | 6 | near-level, high, lateral, diagonal, far, descending |
| Linear | 5 | crossing, receding (tail-chase), approaching (head-on), climbing, diagonal |
| Ablation | 2 | `b = 0` controls for the altitude-penalty study |
| Stress — sinusoidal | 4 | lateral weave, vertical bob, fast juke, 3D spiral |
| Stress — varying speed | 3 | head-on 90 km/h, quartering 86 km/h, beam-crossing 84 km/h |
| Stress — wind | 4 | static/linear/evasive under moderate and gusty presets |

### 15.3 Randomized Monte-Carlo harness (`montecarlo.py`)

Named scenarios probe *specific* geometries; the Monte-Carlo harness samples the *whole
threat envelope*. From one `master_seed` it draws a seeded batch of randomized 3D
engagements — geometry within a frontal cone, a weighted trajectory family, family
parameters, and a weighted wind preset — converts each draw into a validated `Scenario`
through the same parser the YAML files use (single source of truth), and flies it through
the ordinary closed loop.

* **Mission Success Rate = interception fraction** (`R_miss ≤ 1.05 m`), matching the design
  review's *"≥ 90 % interception over randomized 3D trials"*. The other KPIs are reported as
  **separate compliance rates**, so a very short high-speed intercept that transiently
  exceeds the saturation budget is a *mission success with a filed finding*, not a mission
  failure.
* Results are broken down **per trajectory family and per wind preset**, so weak regimes are
  exposed rather than averaged away.
* Each trial's run seed is its index, so `(master_seed, num_trials)` reproduces the whole
  batch byte-for-byte; a batch manifest records the master seed, git hash, committed tuning,
  KPI targets and every headline verdict.

### 15.4 Reporting (`reporting.py`)

Produces the KPI summary table (CSV + Markdown), per-scenario diagnostic plots (X-Y
geometry; altitude-vs-time with the overshoot band; range + command effort with saturated
frames shaded), the b-ablation altitude overlay, and the batch artifacts
(`batch_kpis.csv`, `batch_manifest.json`, `batch_distributions.png`). It forces
matplotlib's headless **Agg** backend so it never opens a window.

### 15.5 The test suite

**221 passing tests**, split into `tests/unit/` (per component: EKF, guidance, control,
sensors, trajectories, kinematics, frames, RNG, scheduler, contracts, KPIs, scenarios,
Monte-Carlo sampling/aggregation, wind wiring, batch reporting, orchestrator logging) and
`tests/integration/` (whole-pipeline: stub loop determinism, real interception, the
scenario suite, the stress suite, a reproducible Monte-Carlo batch, MuJoCo headless
render, replay).

All tests are **headless, non-interactive and seeded**. Tests requiring a GL context are
marked `mujoco` so they can be skipped where there is no display. Failure modes are covered
explicitly — saturation, EKF divergence, NaN/instability, and boundary conditions — not just
the happy path.

Test growth by phase: **55 → 107 → 149 → 178 → 208 → 221**.

---

## 16. Implementation Phases — What Was Built and What Was Learned

### 16.0 Phase 0 — Foundations *(pre-Phase-1 setup; Role 6)* ✅

Built structure only — no algorithm logic — exactly as scoped.

* Reproducible environment (`pyproject.toml` pinning mujoco 3.10.0, numpy 2.5.0,
  scipy 1.18.0, pyyaml, matplotlib) and `scripts/check_env.py`, an environment doctor that
  imports everything, loads a trivial MJCF, steps it, and renders one frame **off-screen**.
* Full repository layout; `config/constants.py` + `config/params.py` as the single source of
  truth; `common/frames.py` conventions.
* All 6+ **data contracts** and every **abstract interface**, each with a pass-through stub.
* `common/rng.py`, `common/logging.py`, `common/guards.py` — determinism infrastructure.
* The **multi-rate scheduler** and the **stub orchestrator**, proving the full 6-stage loop
  closes headlessly and deterministically.

**Verification:** 55 tests green; two seed-0 runs produced byte-identical `run_log.csv`.

### 16.1 Phase 1 — The Simulated World *(Jun 17 – Jun 30; Role 1)* ✅

* `models/{scene,quadcopter,target}.xml`; MuJoCo plant with load-time consistency
  assertions.
* `RotorActuatorModel` — quadratic thrust/drag, "+"-config torque mapping, RPM saturation
  reported as an event.
* All five trajectory generators with analytic velocities.
* `NoisyDelayedSensorModel` — noise, bias, quantization, finite update rate, latency buffer.
* `WindField` — steady + Ornstein-Uhlenbeck gusts, precomputed and seeded.
* Ground-truth kinematics (sim-only) and the off-screen renderer.
* `scripts/replay.py` — the *only* sanctioned interactive window: an opt-in replay viewer
  that drives bodies to logged poses (`mj_forward` only — no physics re-step, no ground
  truth) and therefore cannot affect any result.

**Verification:** 107 tests green. The airframe hovers with < 5 cm drift over 30 s;
free-fall matches `½gt²`; sensor residual statistics match the configured profile over
20 000 samples; the calm wind preset is byte-identical to no wind.

### 16.2 Phase 2 — Closing the Loop *(Jul 1 – Jul 15; Roles 2/3/4/6)* ✅

The real algorithms replaced the stubs behind the *same* interfaces: EKF, OGL (+ ZEM,
time-to-go), command limiter, differential-flatness outer loop, attitude-PD inner loop,
motor mixer, and the `phase2_intercept` wiring.

**Result:** the closed loop intercepts static targets across varied 3D geometries to
**0.01–0.04 m** miss distance, in 3–6 s, headless and deterministically.

**Three decisions that shaped everything downstream:**

1. **Scope narrowed to OGL only.** PN and APN were dropped; the `GuidanceLaw` interface was
   kept for Open/Closed.
2. **Augmented ZEM gated off.** Feeding the relative-state EKF's acceleration into the ZEM
   proved to be positive feedback and diverged every diagonal/steep intercept.
3. **Yaw gains cut ~100×.** Root cause of an early tumbling instability: yaw torque comes
   from the weak drag differential, so normal-sized yaw gains demanded impossible rotor
   imbalances and saturated all four motors. `kp_yaw = 2` unlocked clean interception.

**Post-phase refinement:** engagement termination at closest approach (§13.3) and the
two-view replay camera with trajectory trails.

### 16.3 Phase 3 — Meeting Spec on Static and Linear Targets *(Jul 16 – Aug 5; Role 5 driving)* ✅

Built the KPI + declarative-scenario tooling, then used it to move the system from
"functioning" to **"meets spec"**.

**Result: 11/11 scenarios met every KPI** (Phase-3-era measurements, seed 0):

| Scenario | Class | R_miss (m) | t_int (s) | Z-over (m) | Sat % | Pass |
| :--- | :--- | ---: | ---: | ---: | ---: | :---: |
| static_near_level | static | 0.018 | 2.96 | 0.014 | 4.2 | ✅ |
| static_high | static | 0.007 | 3.95 | 0.005 | 0.2 | ✅ |
| static_lateral | static | 0.013 | 2.75 | 0.441 | 4.9 | ✅ |
| static_diagonal | static | 0.011 | 5.53 | 0.008 | 0.8 | ✅ |
| static_far | static | 0.028 | 8.35 | 0.018 | 0.6 | ✅ |
| static_descend | static | 0.009 | 3.60 | 0.000 | 0.7 | ✅ |
| linear_approaching | moving | 0.059 | 4.74 | 0.041 | 1.5 | ✅ |
| linear_crossing | moving | 0.013 | 5.56 | 0.008 | 0.7 | ✅ |
| linear_receding | moving | 0.019 | 5.82 | 0.015 | 0.7 | ✅ |
| linear_climbing | moving | 0.063 | 4.82 | 0.024 | 1.3 | ✅ |
| linear_diagonal | moving | 0.005 | 5.68 | 0.003 | 0.0 | ✅ |

**Key findings:**

* **Saturation was a launch transient, not a terminal spike.** Per-third analysis showed
  saturation concentrated in the *first* ~20 % of frames with 0 % mid-flight. This
  redirected the fix away from the terminal `1/t_go²` term (raising the `t_go` floor made it
  *worse*) toward from-rest launch shaping — hence `reference_closing_speed 5.0 → 3.5`.
* **The far-range / soft-launch trade-off is real and was documented rather than papered
  over.** No single reference closing speed both kept the near geometries under 5 %
  saturation *and* the 12.4 m static target under 10 s. Raising tilt authority to 45°
  dissolved the tension.
* **EKF needed no retune** — estimation was verified not to be the bottleneck.
* **The `b`-penalty ablation returned a null result and was reported as such** (§9.5).

### 16.4 Phase 4 — Randomized 3D Trials *(Aug 6 – Aug 20; Role 5)* ✅

Added the **evasive / high-speed / wind stress suites** and the **seeded randomized 3D
Monte-Carlo harness**, then certified the acceptance table.

**Measured first, tuned second — and reported the ugly baseline.** The initial randomized
batch scored **50 % mission success**. Triage attributed the gap to (a) an over-broad
sampler generating physically unwinnable trials, and (b) a genuine command-authority
shortfall.

* **(a) was scenario design, not rigging.** The envelope was tightened so targets stay
  airborne and move *through* the engagement zone — a target diving underground or simply
  fleeing a from-rest interceptor is not a valid intercept trial. The hard evasive and
  90 km/h tail was **kept**, and the per-family breakdown reports exactly where it hurts.
* **(b) was fixed params-only, with user sign-off:** `max_tilt_rad` 45° → 60° (authority
  9.81 → 17.0 m/s²), lifting randomized mission success ~57 % → **93 %**; and
  `max_acceleration_m_s2` 30 → 40. No estimation/guidance/control *logic* was edited and no
  physical constant was touched.

Also aligned in this phase: **mission success is measured as interception**, per the design
review, with the other KPIs reported as separate compliance rates. Wind was finally wired
through to the plant (the model existed since Phase 1 but had never been fed to it), with
`calm → None` so undisturbed runs stay byte-identical to Phase 2/3.

### 16.5 F4-1 — The Honest Saturation Fix *(post-Phase-4 follow-up)* ✅

This is the project's most instructive episode, and worth presenting in full.

**Finding 1 — the KPI itself was undercounting.** The `saturated` flag written to the run
log counted **only the command limiter**. Motor-mixer (actuator) saturation was logged
loudly but never counted, and not even stored as a column. The airframe could therefore be
saturated while the metric stayed green — precisely the hidden saturation the engineering
contract forbids. Concretely: `varying_speed_headon_90kmh` **passed at 4.6 %** on the old
metric while being truly **15.7 %** saturated, almost entirely in the mixer.

**Finding 2 — the saturation was not a launch transient after all.** On hard short
engagements it is (a) **mid-course horizontal pinning** against the tilt cap, and
(b) **mixer min-clamping** — at low collective thrust the allocation drove a rotor below
0 RPM to produce roll/pitch torque, even though rotors sat at only ~5–7 k of 25 k RPM.

**The three-part fix** (all verified to cause no miss-distance regression):

1. **Honest measurement** — saturation KPI = limiter ∪ mixer, with per-stage attribution
   columns.
2. **`max_tilt_rad` 60° → 70°** plus the new inner-loop **angular-acceleration clamp**
   (70 rad/s²), enabled by the thrust-projection fix that had removed the Z-overshoot which
   originally blocked > 60°.
3. **Attitude-priority motor mixer** — raise the collective uniformly instead of clipping a
   rotor, preserving every torque differential (§12.1).

**Result:**

| Metric | Old Phase 4 (limiter-only) | Honest baseline | **After F4-1** |
| :--- | :--- | :--- | :--- |
| Mission success (interception) | 93 % | ~92 % | **95 %** |
| Command-saturation compliance | 74 % *(undercounted)* | ~40 % *(honest)* | **77 % (honest)** |
| Z-overshoot compliance | 95 % | — | **98 %** |
| Named static/linear suite | 11/11 | — | **11/11 (honest)** |

The hardest single case, `varying_speed_crossing_84kmh`, fell from **40.2 % → 5.8 %**
saturation.

> **The presentable lesson:** the reported number was allowed to get *worse* (74 % → ~40 %)
> in order to become *true*, and only then was the physics fixed (→ 77 %). Fixing the
> measurement before fixing the metric is the difference between engineering and
> KPI-polishing.

### 16.6 Committed tuning history

Every change below was params-only, user-approved, and carries an inline rationale in
`config/params.py` referencing the report that justified it. **No physical constant in
`config/constants.py` was ever changed**, and the airframe/motor model is untouched.

| Parameter | History | Driving reason |
| :--- | :--- | :--- |
| `reference_closing_speed_m_s` | 5.0 → **3.5** (P3) → **4.25** (F4-1) | P3: soften the from-rest launch transient that was saturating the first ~20 % of frames. F4-1: once thrust projection removed the tilt-lag Z-overshoot, the binding constraint reverted to saturation and 4.25 became affordable — worth ~15 % off mean time-to-intercept on slow static/linear geometries. |
| `max_tilt_rad` | 35° → **45°** (P3) → **60°** (P4) → **70°** (F4-1) | Horizontal authority is `g·tan(tilt)`: 6.87 → 9.81 → 17.0 → 27.0 m/s². Each raise targeted the dominant saturation source of its phase. |
| `max_acceleration_m_s2` | 30 → **40** (P4) | The magnitude cap bound only on aggressive climbing dashes; 40 stays far inside the ~250 m/s² thrust capacity so the mixer never saturates in the limiter's place. |
| `max_angular_accel_rad_s2` | **70** (new, F4-1) | The hover-feasible angular-acceleration ceiling; turns an infeasible torque spike into a rate-limited slew. |
| Inner-loop yaw gains | `kp 300 → 2`, `kd 30 → 0.5` (P2) | Yaw torque is ~100× weaker than roll/pitch; normal gains saturated all four motors and tumbled the airframe. |
| EKF `Q`/`R`, `b`, `N'` bounds, `T` | **unchanged** | Measured and found not to be the bottleneck; ablation results reported faithfully. |

---

## 17. Final Results

### 17.1 Canonical randomized batch — 100 trials, master seed 0

| KPI | Success Target | Measured | Verdict |
| :--- | :--- | :--- | :---: |
| Mission Success Rate (interception) | ≥ 90 % | **95 %** | ✅ |
| Max Target Speed intercepted | ≥ 83.6 km/h | **89.7 km/h** | ✅ |
| Miss Distance `R_miss` ≤ 1.05 m | — | 95 % of trials | ✅ |
| Z-Axis Overshoot ≤ 0.5 m | — | 98 % of trials (median ≈ 0.02 m) | ✅ |
| Time-to-Intercept (static < 10 s / moving < 20 s) | — | 95 % of trials | ✅ |
| Command Saturation ≤ 5 % of flight time | — | **77 %** of trials | ❌ |

### 17.2 Interception breakdown

| Target family | Interception | | Wind preset | Interception |
| :--- | ---: | :--- | :--- | ---: |
| static | 21/21 (**100 %**) | | calm | **94 %** |
| linear | 32/32 (**100 %**) | | moderate | **100 %** |
| sinusoidal (evasive) | 34/35 (**97 %**) | | gusty | **93 %** |
| varying_speed (to 90 km/h) | 8/12 (**67 %**) | | | |

Static and linear geometries are solved outright. Evasive weaving targets are intercepted
97 % of the time. **Wind robustness is confirmed** — interception is essentially flat across
calm / moderate / gusty, meaning the dual-loop controller absorbs the disturbance as a
gentle bias. The entire residual failure mass sits in `varying_speed`: fast, accelerating,
off-axis targets — the genuine hard tail.

### 17.3 Named scenario suites

* **Static + linear (11 scenarios): 11/11 meet every KPI**, miss distances **0.003–0.037 m**.
* **Stress — evasive / high-speed / wind (11 scenarios): 9/11 meet every KPI.** Both
  exceptions (`sinusoidal_fast_juke`, `varying_speed_crossing_84kmh`) are **saturation-only**
  breaches — every stress scenario still intercepts inside the 1.05 m miss KPI.

### 17.4 Interpretation

The system **tracks, navigates toward, and intercepts static, linear, evasive (weaving) and
high-speed (to 90 km/h) targets, and holds up under wind and gust disturbance** — entirely
with the Classical Hierarchical pipeline, no DRL, fully deterministic, and with every result
reproducible from a seed and a config file.

---

## 18. Engineering Findings, Limitations and Future Work

Three findings remain open **by design**, each triaged to an owning role with reproduction
seeds, rather than hidden or tuned away.

### F4-1 — Command saturation on short high-speed intercepts *(worked; residual accepted)*

**Regime.** 85+ km/h crossing/quartering targets intercepted from rest in ~2 s.
**Status.** Measurement corrected and physics substantially improved (honest compliance
~40 % → **77 %**). The residual **23 %** is a genuine physical tail: the engagement is over
before the airframe can finish accelerating, so a large fraction of a very short flight is
spent legitimately at maximum authority.
**What closing it would require.** Adaptive-authority or launch-shaping guidance logic —
explicitly scoped out of this project.

### F4-2 — Fast off-axis misses

**Regime.** `varying_speed` above ~85 km/h with a strong crossing/quartering component.
**Nature.** A from-rest interceptor physically cannot lead the fastest strongly-crossing
targets. This is the documented **degradation edge**, not a defect.
**Candidate improvement.** The **target-acceleration feed-forward (augmented ZEM)**, which
requires isolating the target's *absolute* acceleration from the relative-state EKF using
the known interceptor acceleration. Deliberately deferred.

### F4-3 — Far-static time budget

**Regime.** Static targets beyond ~12 m.
**Nature.** The 10 s static time-to-intercept KPI is tight from rest at long range.
**Impact.** Interception still succeeds — only the *time* metric slips.

### Other honest limitations worth stating

* Several airframe/motor figures (mass, inertia, `kT`/`kQ`, `MOTOR_RPM_MAX`, wind drag
  coefficient) are **documented placeholders**, labelled as such in `constants.py`. They are
  mutually consistent and physically plausible, but they are not measurements of a specific
  real aircraft.
* Wind uses a **single lumped drag coefficient** rather than a full aerodynamic surface
  model — explicit and reproducible, but simplified.
* The target is a **mocap body** whose motion is prescribed; it does not react to the
  interceptor. The evasion modelled is pre-programmed weaving, not closed-loop adversarial
  evasion.
* The EKF absorbs the interceptor's own maneuvers into process noise rather than modelling
  them — the deliberate simplification that also forces the augmented-ZEM term to stay off.

### Design decisions carried to the end

* **OGL is the sole guidance law.** PN/APN were evaluated and rejected; not implemented.
* **The augmented-ZEM term remains gated off.** It is the candidate next step for the
  fast-crossing tail, not a shipped feature.
* **Mission success is interception**, per the design review; other KPIs are reported as
  separate compliance rates.
* **No DRL entered any layer at any point.**

---

## 19. Reproduction Guide

From the project root on Windows (PowerShell). MuJoCo is installed at
`C:/Dev/Libraries/mujoco`; the pip `mujoco` wheel bundles its own native libraries.

```powershell
# One-time setup
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -e ".[dev]"

# Verify the environment (Python, imports, MuJoCo, one off-screen frame)
python scripts/check_env.py

# Phase 0 — the whole 6-stage loop on trivial stubs, deterministic
python scripts/run_stub_pipeline.py --steps 400 --seed 0

# Phase 2 — a real guided interception against a static target
python scripts/run_intercept.py --target 8 3 6 --seconds 9
python scripts/replay.py results/intercept                     # top isometric view + trails
python scripts/replay.py results/intercept --view interceptor  # chase cam

# Phase 3 — declarative scenarios with a KPI report
python scripts/run_scenarios.py scenarios/linear_crossing.yaml
python scripts/run_scenarios.py scenarios/ --report            # -> results/scenarios/

# Phase 4 — evasive/high-speed/wind stress probes, and the randomized batch
python scripts/run_scenarios.py scenarios/stress --results-dir results/stress
python scripts/run_montecarlo.py --trials 100 --seed 0 --report --results-dir results/montecarlo

# The tests
pytest                    # everything (221 tests)
pytest -m "not mujoco"    # skip the off-screen GL render tests
```

Every run writes `results/<run_id>/` containing `run_log.csv` (per-step data) and
`run_config.json` (seed + resolved parameters + git hash) — enough to replay or reproduce it
exactly. Every batch additionally writes `batch_kpis.csv`, `batch_manifest.json` and
`batch_distributions.png`.

---

## 20. Project Map

```
Workshop_Autonomous_Systems/
├── AGENTS.md                  # The engineering contract — roles, boundaries, standards
├── CLAUDE.md / GEMINI.md      # Point AI assistants at AGENTS.md
├── README.md                  # Quick-start + headline results
├── pyproject.toml             # Pinned dependencies
│
├── docs/
│   ├── Autonomous_Drone_Interceptor_Design_Review.md   # THE authoritative design
│   ├── Andrea_Tini_Thesis_Summary.md                   # Source-literature summary
│   ├── implementation_plan.md, phase0..4.md            # Phased roadmap + task DoDs
│   ├── phase*_progress.md                              # What was actually built/verified
│   ├── PROJECT_EXPLAINED.md                            # From-scratch tutorial explainer
│   └── PROJECT_REPORT.md                               # (this document)
│
├── models/                    # MuJoCo world (MJCF XML)
│   ├── scene.xml              # Solver, timestep, floor, lights; includes both bodies
│   ├── quadcopter.xml         # The interceptor airframe
│   └── target.xml             # The target (kinematic mocap body)
│
├── scenarios/                 # Declarative trial configs (YAML)
│   ├── static_*.yaml          # 6 static geometries
│   ├── linear_*.yaml          # 5 constant-velocity geometries
│   ├── ablation/*.yaml        # b = 0 controls for the altitude-penalty study
│   └── stress/*.yaml          # 4 sinusoidal + 3 varying-speed + 4 wind
│
├── scripts/                   # Entry points
│   ├── check_env.py           # Environment doctor (verifies MuJoCo, off-screen render)
│   ├── run_stub_pipeline.py   # Phase 0 — loop on pass-through stubs
│   ├── run_intercept.py       # Phase 2 — real guided interception
│   ├── run_scenarios.py       # Phase 3 — scenario(s) + KPI table + optional report
│   ├── run_montecarlo.py      # Phase 4 — randomized 3D mission-success batch
│   ├── run_sim_demo.py        # Simulation-only demo
│   └── replay.py              # Interactive replay viewer (opt-in, never in CI)
│
├── src/interceptor/
│   ├── config/
│   │   ├── constants.py       # Physical constants + KPI thresholds (one source of truth)
│   │   └── params.py          # Tunable parameters (EKF/PID/guidance/limiter/sensor/wind)
│   ├── common/
│   │   ├── types.py           # The immutable pipeline messages (data contracts)
│   │   ├── frames.py          # Coordinate frames, quaternions, LOS math
│   │   ├── rng.py             # Seeded, named RNG streams (determinism)
│   │   ├── logging.py         # Per-step CSV + reproducibility snapshot
│   │   └── guards.py          # Fail-loud validators
│   ├── simulation/            # STAGE 1 (Role 1)
│   │   ├── mujoco_plant.py    # MuJoCo wrapper — the plant
│   │   ├── actuators.py       # Rotor thrust/torque model + RPM saturation
│   │   ├── sensors/noisy_sensor.py    # Noisy, biased, delayed sensor
│   │   ├── trajectories/generators.py # Five target-motion families
│   │   ├── kinematics.py      # Ground-truth geometry (sim-only; never leaks)
│   │   ├── wind.py            # Steady wind + OU-process gusts
│   │   ├── rendering.py       # Off-screen renderer
│   │   ├── interfaces.py      # Plant / SensorModel / TargetTrajectory / Renderer ABCs
│   │   └── stubs.py           # Pass-through implementations (Phase 0 / tests)
│   ├── estimation/            # STAGE 2 (Role 2) — ekf.py, interfaces.py, stubs.py
│   ├── guidance/              # STAGE 3 (Role 3) — ogl.py, zem.py, time_to_go.py, ...
│   ├── control/               # STAGES 4 & 5 (Role 4)
│   │   ├── command_limiter.py # Tilt & magnitude clamps + saturation reporting
│   │   ├── outer_loop.py      # Differential-flatness accel → attitude
│   │   ├── inner_loop.py      # Gyro-fed attitude PD → body torque (+ clamp, projection)
│   │   └── motor_mixer.py     # Torque + thrust → four rotor RPMs (attitude-priority)
│   ├── pipeline/              # Wiring (Role 6) — orchestrator.py, scheduler.py
│   └── analysis/              # Validation (Role 5)
│       ├── kpis.py            # Measure metrics from a run log
│       ├── scenarios.py       # Declarative scenario runner
│       ├── montecarlo.py      # Randomized 3D harness + aggregation
│       └── reporting.py       # Summary tables, batch manifest, diagnostic plots
│
├── tests/                     # unit/ + integration/ — headless, seeded (221 tests)
└── results/                   # Generated run logs, snapshots, reports (per run_id)
```

---

## 21. Glossary

| Term | Meaning |
| :--- | :--- |
| **Attitude** | The drone's orientation (roll / pitch / yaw) |
| **Azimuth / Elevation** | Horizontal / vertical angles of the line of sight |
| **Body frame** | Coordinates fixed to the drone (X forward, Y left, Z up) |
| **Closing speed** | How fast the interceptor–target range is shrinking |
| **Covariance** | Matrix expressing the Kalman filter's uncertainty about its estimate |
| **Determinism** | Same inputs always produce byte-identical outputs |
| **Differential flatness** | Property letting a desired acceleration map algebraically to tilt + thrust |
| **EKF** | Extended Kalman Filter — a Kalman filter linearized for the nonlinear range/angle sensor |
| **Fail loud** | Raise on bad data (NaN, divergence, out-of-range) instead of continuing silently |
| **Innovation** | Difference between the actual and expected measurement |
| **Jacobian** | Matrix of partial derivatives used to linearize the sensor equations |
| **Joseph form** | Numerically stable covariance-update formulation |
| **Kalman gain** | The weight blending prediction vs. measurement each update |
| **KPI** | Key Performance Indicator — a graded success metric |
| **LOS (Line of Sight)** | The straight line from interceptor to target; its *rate* is central to guidance |
| **LQ optimization** | Linear-Quadratic optimal control — minimizes a quadratic cost, here `J = y(t_f)² + ∫u²dt` |
| **MJCF** | MuJoCo's XML model format |
| **Mocap body** | A body whose pose is prescribed (teleported), not physically simulated |
| **Motor mixer** | Converts desired thrust + torques into four rotor speeds |
| **Navigation ratio `N'`** | The proportionality gain in the guidance law (here 3–5, time-varying) |
| **OGL** | Optimal Guidance Law — the sole guidance law; a lag-aware, altitude-penalized ZEM law |
| **Ornstein–Uhlenbeck process** | Generates smooth, temporally correlated random gusts |
| **PID / PD controller** | Proportional-Integral-Derivative feedback controller; the inner loop uses PD |
| **Plant** | Control-theory term for the system being controlled (here, the MuJoCo drone) |
| **Process noise** | The Kalman filter's assumed uncertainty in its motion model |
| **Quaternion** | Four-number encoding of a 3D rotation; robust and gimbal-lock-free |
| **Saturation** | A command hitting a physical limit (clamped); tracked as a KPI |
| **Tilt delay** | The drone's inability to change attitude instantly, modeled as `1/(T·s+1)` |
| **Time-to-go `t_go`** | Estimated time remaining until intercept |
| **Wrench** | A combined force + torque acting on a body |
| **World frame** | The fixed ground frame (Z up = altitude) |
| **ZEM** | Zero-Effort-Miss — the predicted miss vector if no further acceleration were applied |

---

*Sources: `docs/Autonomous_Drone_Interceptor_Design_Review.md` (authoritative design),
`docs/Andrea_Tini_Thesis_Summary.md`, `docs/implementation_plan.md`,
`docs/phase0–4_progress.md`, `docs/PROJECT_EXPLAINED.md`, `AGENTS.md`, `README.md`, and the
committed source in `src/interceptor/`.*
