# The Autonomous Drone Interceptor, Explained From Scratch

This document explains **everything** in this project — what it does, how every piece
works, and the mathematics and physics underneath each algorithm — assuming you have
**no prior knowledge** of control theory, estimation, or drone dynamics. Read it
top-to-bottom and you should understand not just *what* the code does but *why*.

It is long on purpose. Sections are self-contained; skim the table of contents and jump
where you like, but the "Background you need" section (Part 2) makes everything after it
much easier.

---

## Table of contents

1. [What this project is, in one paragraph](#1-what-this-project-is-in-one-paragraph)
2. [Background you need (no assumptions)](#2-background-you-need-no-assumptions)
3. [The big idea: a 6-stage pipeline](#3-the-big-idea-a-6-stage-pipeline)
4. [Why "classical" and not AI/deep learning](#4-why-classical-and-not-aideep-learning)
5. [Coordinate frames and conventions](#5-coordinate-frames-and-conventions)
6. [Stage 1 — Simulation (the physical world)](#6-stage-1--simulation-the-physical-world)
7. [Stage 2 — Estimation (the Kalman filter)](#7-stage-2--estimation-the-kalman-filter)
8. [Stage 3 — Guidance (the Optimal Guidance Law)](#8-stage-3--guidance-the-optimal-guidance-law)
9. [Stage 4 — Command Limiter (safety)](#9-stage-4--command-limiter-safety)
10. [Stage 5 — Flight Control (the dual loop)](#10-stage-5--flight-control-the-dual-loop)
11. [Stage 6 — Motor Mixer (down to four numbers)](#11-stage-6--motor-mixer-down-to-four-numbers)
12. [How the stages are wired together](#12-how-the-stages-are-wired-together)
13. [Configuration: constants vs. tunable parameters](#13-configuration-constants-vs-tunable-parameters)
14. [Determinism and reproducibility](#14-determinism-and-reproducibility)
15. [Testing, KPIs, and scenarios](#15-testing-kpis-and-scenarios)
16. [The project map, file by file](#16-the-project-map-file-by-file)
17. [How to run everything](#17-how-to-run-everything)
18. [Current status and roadmap](#18-current-status-and-roadmap)
19. [Glossary](#19-glossary)

---

## 1. What this project is, in one paragraph

This is a **simulation** of a small four-rotor drone (a *quadcopter*) whose job is to
autonomously hunt down and physically collide with another, possibly evasive, flying
drone — a "counter-drone" interceptor. Nothing physical is built; everything runs inside
a physics engine called **MuJoCo** on a computer. The interceptor gets only *noisy,
delayed* sensor readings of where the target is (like a real radar), has to *estimate*
the target's true motion from those imperfect readings, *decide* how to accelerate to
hit it, and *translate* that decision all the way down to four individual motor speeds —
all while respecting real physical limits (motors can only spin so fast, the drone can't
change its tilt instantly). The whole thing is built out of separate, mathematically
explainable stages rather than a single "AI brain," on purpose.

It is a university workshop project ("Workshop in Autonomous Systems Simulation") by Dor
Varsulker, developed in phases. The authoritative design document is
[`docs/Autonomous_Drone_Interceptor_Design_Review.md`](./Autonomous_Drone_Interceptor_Design_Review.md);
the engineering rules every contributor (human or AI) follows are in
[`AGENTS.md`](../AGENTS.md).

---

## 2. Background you need (no assumptions)

Here are the concepts the rest of the document leans on, each in plain terms.

### 2.1 Vectors, position, velocity, acceleration

A **vector** is just a list of numbers with a direction — here almost always three
numbers `[x, y, z]` describing a point or a direction in 3D space.

- **Position** `[x, y, z]` — where something is, in metres (m).
- **Velocity** — how fast the position changes, in metres per second (m/s). It's the
  *derivative* (rate of change) of position.
- **Acceleration** — how fast the velocity changes, in metres per second squared (m/s²).
  It's the derivative of velocity. A force applied to a mass produces acceleration:
  Newton's law **F = m·a**, so **a = F/m**.

If you know an object's acceleration and its starting position and velocity, you can
predict where it will be later. Over a short time step `dt`:

```
new_position ≈ position + velocity·dt + ½·acceleration·dt²
new_velocity ≈ velocity + acceleration·dt
```

These two lines are the heart of both the physics engine and the Kalman filter.

### 2.2 A quadcopter cannot fly sideways directly

A quadcopter has four upward-pointing propellers. **All the thrust points "up" out of
the drone's body.** So how does it move sideways? It **tilts**. If it tilts forward, a
portion of that thrust now points forward, pushing it forward while the rest still fights
gravity. This single fact drives a huge amount of the design:

- To accelerate horizontally, the drone must first **change its tilt angle**.
- It cannot tilt *instantly* — tilting takes time (the "tilt delay"). The guidance math
  explicitly accounts for this lag instead of pretending turns are instantaneous.
- The maximum horizontal acceleration is limited by the maximum safe tilt angle.

### 2.3 Angles: roll, pitch, yaw

An aircraft's orientation ("attitude") is described by three angles:

- **Roll** (φ, "phi") — tipping left/right (rotating about the forward axis).
- **Pitch** (θ, "theta") — nose up/down (rotating about the sideways axis).
- **Yaw** (ψ, "psi") — turning left/right like a car steering (rotating about the
  vertical axis).

A quadcopter translates by rolling and pitching; **yaw doesn't help it chase a target**
(it can move any direction without turning to face it), which is why the code keeps yaw
gentle and fixed at zero.

### 2.4 Line of Sight (LOS)

The **Line of Sight** is the imaginary straight line from the interceptor to the target.
Two angles describe its direction:

- **Azimuth** — the compass-like left/right angle in the horizontal plane.
- **Elevation** — the up/down angle above the horizontal.

The **LOS rate** is how fast that line is *rotating* (in radians per second). This is the
single most important quantity in classical guidance, explained in Part 8. Intuition: if
you're driving toward another car and it stays at a *constant bearing* — always at the
same angle out your window, getting bigger — **you are on a collision course**. A
non-rotating line of sight means impact. Guidance works by driving the LOS rotation
toward zero.

### 2.5 What "control loop" means

A **control loop** is the endless cycle: *measure the current state → compute an error
(how far from where we want to be) → command an actuator to reduce that error → repeat.*
Doing this fast and stably is the whole field of *control theory*. A **PID controller**
(Part 10) is the classic recipe.

### 2.6 Noise, and why estimation is hard

Real sensors lie a little. Every reading is the true value plus random **noise** (jitter)
and often a fixed **bias** (a consistent offset), and it arrives **late** (latency).
Feeding raw noisy data straight into guidance produces violent, twitchy commands. The
job of the **estimator** (the Kalman filter) is to see through the noise and delay and
recover a clean, trustworthy picture of the target's motion.

### 2.7 Radians

Angles here are in **radians**, not degrees. π radians = 180°. So 60° ≈ 1.047 radians,
which you'll see as `1.0472` in the code (the maximum tilt, raised from 45° in Phase 4 to
give the interceptor more authority against fast, evasive targets).

---

## 3. The big idea: a 6-stage pipeline

The interceptor's "brain" is not one program — it's a **chain of six specialized stages**,
each taking the previous stage's output and producing the next stage's input. The chain
runs over and over, many times per second. This is the **Classical Hierarchical
Architecture**.

```
   ┌─────────────────────────────────────────────────────────────────────┐
   │                                                                       │
   ▼                                                                       │
[1 SIMULATION] ──raw noisy, delayed sensor reading──►                     │
[2 ESTIMATION] ──clean target position, range, LOS rate──►                │
[3 GUIDANCE]   ──"ideal" acceleration I want──►                           │
[4 LIMITER]    ──clamped, physically safe acceleration──►                 │
[5a OUTER CTRL]──target tilt angles + thrust──►                           │
[5b INNER CTRL]──body torques + thrust──►                                 │
[6 MOTOR MIXER]──four rotor RPM values────────────────────────────────────┘
                     (back into the Simulation as motor commands)
```

The **golden rule** (enforced throughout `AGENTS.md`): **each stage may only read its
immediate predecessor's output.** Guidance is *forbidden* from peeking at the raw sensors.
Control is *forbidden* from reading the true target position. Crossing a boundary — for
example, letting the estimator "cheat" by reading the simulator's ground-truth target
position — is treated as a bug. This keeps every stage independently testable and the
whole system explainable.

Each stage maps to a **role** (Simulation Engineer, Estimation Engineer, Guidance
Engineer, Flight Control Engineer, Test/KPI Engineer, Integration Architect). The roles
are a way of assigning ownership and boundaries; you'll see them referenced in the code
comments (e.g. "Role 3, Phase 2").

The messages passed between stages are strict, validated data structures (Part 12). They
are **immutable** and **fail loud** — if any number is `NaN` (not-a-number) or infinity,
the program raises an error immediately rather than quietly corrupting the run.

---

## 4. Why "classical" and not AI/deep learning

The design review explicitly considered two ways to build the brain:

- **Architecture A — Classical Hierarchical (chosen):** the 6 explainable math stages
  above.
- **Architecture B — Deep Reinforcement Learning (rejected):** feed sensor data into a
  neural network trained to output motor commands directly (a single learned "black box").

Deep RL *can* discover spectacular maneuvers, but it was rejected because:

- **It exploits simulator quirks.** RL agents often find physically impossible tricks
  that work in simulation but would stall a real motor.
- **It's a black box.** You can't explain *why* it did something, which is unacceptable
  for a safety-critical interceptor.
- **It's twitchy under noise.** Learned policies tend to overreact to sensor noise.
- **It's non-deterministic.** Hard to reproduce and certify.

The classical approach trades a little raw agility for **determinism, valid physics, and
explainability** — the project's stated "north star." This is why you'll see, throughout
the code, an insistence on: seeded randomness, no magic numbers, respecting physical
limits, and failing loudly on instability. It's also why deep-learning guidance is a
*forbidden* change.

> Historical note: three guidance algorithms were evaluated — **PN** (Proportional
> Navigation), **APN** (Augmented PN), and **OGL** (Optimal Guidance Law). OGL won and is
> the **only** one implemented. PN/APN survive only as background in the design review.

---

## 5. Coordinate frames and conventions

Before any math, you must agree on *which way is up* and *how rotations are described*.
This lives in [`src/interceptor/common/frames.py`](../src/interceptor/common/frames.py),
the single authority for conventions.

### 5.1 The two frames

- **World frame** (fixed to the ground): right-handed, **Z is up** (altitude). X is a
  forward/"north" reference, Y completes the set. Because the design review warns that the
  altitude (Z) axis is prone to overshoot, Z gets special attention everywhere.
- **Body frame** (glued to the drone): **X forward, Y left, Z up** ("FLU"). When the drone
  is level, its body frame lines up with the world frame.

### 5.2 Representing rotation: quaternions

How do you store "the drone is tilted 20° forward and rolled 5° left"? Two common ways:

- **Euler angles** `[roll, pitch, yaw]` — human-readable, but they have a nasty failure
  ("gimbal lock") at straight-up/down and are awkward to combine.
- **Quaternions** — a set of four numbers `[w, x, y, z]` that encode any 3D rotation
  without gimbal lock and combine cleanly. This project uses **quaternions as the primary
  representation** and converts to Euler angles only for human-readable attitude targets.

You don't need to understand quaternion algebra to follow the project; just know:

- `quat_to_rotation_matrix(q)` turns a quaternion into a 3×3 matrix `R` that rotates a
  vector from body coordinates into world coordinates: `v_world = R · v_body`.
- `quat_to_euler` / `euler_to_quat` convert between the two representations.
- The inner control loop integrates the gyroscope using quaternions because it's
  numerically robust.

### 5.3 LOS angle math (the geometry the sensors and estimator share)

Given the relative position `r = target − interceptor` in world coordinates:

```
range      = |r|                          (straight-line distance)
azimuth    = atan2(r_y, r_x)              (horizontal angle)
elevation  = atan2(r_z, hypot(r_x, r_y))  (angle above horizontal)
```

And the **LOS rate** (how fast those angles change) is a bit of calculus on `r` and the
relative velocity `v`:

```
azimuth_rate   = (r_x·v_y − r_y·v_x) / (r_x² + r_y²)
elevation_rate = (h·v_z − r_z·(dh/dt)) / range²,   where h = hypot(r_x, r_y)
```

These exact formulas appear in three places, deliberately kept consistent: the
ground-truth kinematics (simulation side), the sensor model, and — importantly — in
`frames.los_rate_from_relative`, which lets the **estimator** compute LOS rate from its
own *filtered* estimate without ever touching the simulator's truth.

---

## 6. Stage 1 — Simulation (the physical world)

**Role:** Simulation Environment Engineer. **Files:**
[`simulation/`](../src/interceptor/simulation/), [`models/`](../models/).

This stage *is* the world: the physics, the drone's body, the target's motion, the
sensors, and the wind. Everything else perceives and acts *through* it.

### 6.1 MuJoCo and the model files

**MuJoCo** ("Multi-Joint dynamics with Contact") is a professional physics engine. You
describe your world in an XML file (an "MJCF" model) and MuJoCo integrates the equations
of motion for you.

The world is described across three XML files in [`models/`](../models/):

- [`scene.xml`](../models/scene.xml) — the top-level world. Sets the **timestep**
  (`0.0025 s` = 1/400 s, matching the 400 Hz sim rate), the **integrator** (RK4, a
  high-accuracy method), a ground plane, and lighting. It turns MuJoCo's built-in
  aerodynamics **off** (`density=0 viscosity=0`) because wind is modeled explicitly and
  reproducibly in Python instead. It `<include>`s the two bodies:
- [`quadcopter.xml`](../models/quadcopter.xml) — the interceptor: a "+"-shaped body with
  mass 1.0 kg, four arms of length 0.15 m, and rotor attachment points in a fixed order
  `[front, right, back, left]`. Its mass and inertia are written to match
  `constants.py` exactly — and the plant **asserts** they match on load, so the XML and
  the code can never silently drift apart.
- [`target.xml`](../models/target.xml) — the target drone. It's a **mocap body**, meaning
  its position is *dictated* each step (teleported to wherever the trajectory generator
  says), not physically simulated. The target is a threat to be tracked, not something the
  interceptor's wake could disturb.

### 6.2 The plant: [`mujoco_plant.py`](../src/interceptor/simulation/mujoco_plant.py)

The "plant" (control-theory jargon for *the thing being controlled*) wraps MuJoCo. Each
step it:

1. Takes the four rotor RPMs from the motor mixer.
2. Converts them to a **force and torque** on the body (via the rotor model, below).
3. Rotates that body-frame force/torque into world coordinates.
4. Optionally adds a **wind force**.
5. Applies it and advances the physics one step (`mujoco.mj_step`).
6. Exposes what sensors and controllers are allowed to read: world position, body angular
   rates (the gyroscope), orientation, velocity (finite-differenced from position).

It also asserts the caller's `dt` equals the model's timestep, and checks for `NaN` after
every step (fail loud).

### 6.3 The rotor model: [`actuators.py`](../src/interceptor/simulation/actuators.py)

This is the physics of a spinning propeller. A rotor spinning at `rpm` produces:

```
thrust_i = kT · rpm²          (lift, Newtons — always along the body's up axis)
drag_i   = kQ · rpm²          (a twisting reaction torque about the vertical, from air resistance)
```

`kT` and `kQ` are the thrust and torque coefficients. Thrust grows with the **square** of
RPM — doubling the speed quadruples the lift. From the four rotor thrusts, the model
computes the total body **wrench** (force + torque):

- **Total thrust** = sum of the four (straight up in the body frame).
- **Roll torque** = arm length × (left rotor − right rotor). Spin the left rotor faster,
  the left side rises, the drone rolls right.
- **Pitch torque** = arm length × (back rotor − front rotor).
- **Yaw torque** = the *reaction* to the propellers' drag. Diagonal rotors spin the same
  direction; imbalance them and the whole body counter-rotates. This yaw authority is
  ~100× weaker than roll/pitch — a fact that matters for control tuning.

Crucially, this is the **physical actuator boundary**: incoming RPMs are **clamped** to
`[0, 25000]` RPM (a real motor has limits), and any clamp is *reported* as a saturation
event, never silently swallowed.

### 6.4 The sensor model: [`noisy_sensor.py`](../src/interceptor/simulation/sensors/noisy_sensor.py)

This deliberately **corrupts** the perfect geometry to imitate a real radar/LiDAR. It
takes the true interceptor→target vector and produces a `RawSensorMeasurement` of
`(range, azimuth, elevation)` with:

- **Gaussian noise** — random jitter added to each channel (default: 0.30 m on range,
  ~0.2° on the angles).
- **Bias** — an optional constant offset the filter can't average away.
- **Quantization** — optional rounding to a finite resolution.
- **Finite update rate** — the sensor samples slower (100 Hz) than the sim runs (400 Hz).
- **Latency** — each sample is delivered *late* (default 0.02 s) via an internal delay
  buffer, and stamped with its true age so the estimator can compensate.

A key design principle: the sensor **must not** be "cleaned up" for downstream
convenience. The noise is intentional — it's the entire reason the estimator exists.
Constructing a noisy sensor without a random seed is a *fatal error*, because that would
make the run irreproducible.

### 6.5 Target trajectories: [`trajectories/generators.py`](../src/interceptor/simulation/trajectories/generators.py)

The target's motion is **prescribed** as a formula of time. Each generator gives both
position and (analytically exact) velocity:

- **Static** — sits at a fixed point. The baseline test.
- **Linear** — constant velocity straight line: `p(t) = p₀ + v·t`.
- **Sinusoidal** — a weaving, *evasive* path: straight drift plus a 3D sine wave. This is
  the stress test for the estimator and guidance (Phase 4).
- **VaryingSpeed** — accelerates from a starting speed up to a peak (e.g. 90 km/h) to test
  the maximum-speed KPI.
- **WindAffected** — takes any base trajectory and pushes it around with the integral of a
  wind field.

### 6.6 Wind: [`wind.py`](../src/interceptor/simulation/wind.py)

Wind is modeled as a **steady breeze plus random gusts**. The gusts use an
**Ornstein–Uhlenbeck process** — a mathematically standard way to generate *smooth,
correlated* randomness (real gusts drift and swirl; they aren't jittery white noise). The
whole gust time-series is **precomputed once** from a seeded random generator, so
`velocity_at(t)` is a pure, reproducible function of time. There are three presets:
`calm`, `moderate`, `gusty`. The calm preset produces exactly zero wind — bit-for-bit
identical to the undisturbed physics.

### 6.7 Ground-truth kinematics: [`kinematics.py`](../src/interceptor/simulation/kinematics.py)

This computes the *true* engagement geometry (relative position, velocity, range, LOS
angles, LOS rate, closing speed) from both bodies' actual states. **This is truth, and it
lives strictly inside the simulation layer.** Estimation, guidance, and control are
forbidden from reading it — that would be "cheating with ground truth," the exact defect
the architecture forbids. It's used only to *feed the sensors* (which then corrupt it) and
to *verify* results in tests.

---

## 7. Stage 2 — Estimation (the Kalman filter)

**Role:** Estimation / Perception Engineer. **File:**
[`estimation/ekf.py`](../src/interceptor/estimation/ekf.py).

The estimator's job: given only the **noisy, delayed** sensor stream, produce a **clean,
current** estimate of the target's relative position, velocity, acceleration, range, and
LOS rate — with a measure of how confident it is. It uses an **Extended Kalman Filter
(EKF)**.

### 7.1 What a Kalman filter is (from scratch)

Imagine you're tracking something and you have two imperfect sources of information:

1. A **prediction** of where it should be, based on physics ("it was here, moving this
   fast, so now it's probably there").
2. A **measurement** of where it is, from a noisy sensor.

Neither is perfect. The prediction drifts; the measurement jitters. A **Kalman filter**
is the mathematically optimal recipe for **blending** them: it keeps a running estimate
*and a running measure of its own uncertainty*, and at each step it weighs prediction
against measurement **in proportion to how much it trusts each**. When the sensor is
noisy, it leans on prediction; when the prediction has drifted, it leans on the
measurement. The blend weight is called the **Kalman gain**.

The cycle is two steps, forever:

- **Predict** — advance the estimate forward using a motion model; uncertainty grows.
- **Correct** ("update") — fold in a new measurement; uncertainty shrinks.

### 7.2 Why *Extended*

A plain Kalman filter assumes everything is linear (straight-line relationships). But our
sensor is **nonlinear**: it reports range and angles, which relate to position through
square roots and `atan2`. The **Extended** Kalman filter handles this by
**linearizing** — at each step it computes the *slope* (the derivative, called the
**Jacobian**) of the measurement equations around the current estimate, and uses that
local straight-line approximation. The `_measurement_jacobian` method is exactly this:
the analytic partial derivatives of `(range, azimuth, elevation)` with respect to
position.

### 7.3 The state model (9 numbers)

The filter tracks the **relative** target motion (`target − interceptor`) as 9 numbers:

```
x = [ position(3),  velocity(3),  acceleration(3) ]
```

Its motion model is **constant acceleration**: assume acceleration stays roughly constant
over a step (with a bit of random "jerk" allowed), and propagate:

```
position     += velocity·dt + ½·acceleration·dt²
velocity     += acceleration·dt
acceleration += (random walk)
```

The interceptor's *own* maneuvers aren't known to this layer, so their effect is absorbed
into the model's "process noise." This is a deliberate, honest simplification.

### 7.4 The measurement update

Each cycle, the filter:

1. **Predicts** the state forward by the elapsed time (`_predict`), growing the
   uncertainty (covariance) by the process noise.
2. Computes what the sensor *should* read given the predicted position (`h(x)` =
   range/azimuth/elevation), and the **innovation** = actual reading − expected reading.
   Angle innovations are **wrapped** to (−π, π] so a reading that crosses the ±180° line
   doesn't produce a fake giant error.
3. Computes the **Kalman gain** from the Jacobian and the two uncertainties (process vs.
   measurement noise) and nudges the state by `gain · innovation`.
4. Updates the covariance using the **Joseph form** — a numerically stable rearrangement
   that keeps the uncertainty matrix valid (symmetric, positive) under floating-point
   rounding.

### 7.5 Latency compensation

The measurement is *old* (it was generated 0.02 s ago). If guidance acted on a 0.02-s-old
target position, it would always aim slightly behind. So before publishing, the filter
**predicts its own state forward by the measurement's age**, handing guidance an estimate
valid *now*, not when the photon left the target.

### 7.6 Failing loud on divergence

Kalman filters can **diverge** — if something goes wrong, the uncertainty can blow up to
infinity and the estimate becomes garbage. The filter guards against this: if any state is
`NaN`/infinite, or the total uncertainty (`covariance trace`) exceeds a huge bound, it
**raises an error** instead of emitting nonsense. It also reports a **quality** score in
[0, 1] (high when position uncertainty is low) so guidance can reason about how much to
trust it.

---

## 8. Stage 3 — Guidance (the Optimal Guidance Law)

**Role:** Guidance Engineer. **Files:** [`guidance/ogl.py`](../src/interceptor/guidance/ogl.py),
[`guidance/zem.py`](../src/interceptor/guidance/zem.py),
[`guidance/time_to_go.py`](../src/interceptor/guidance/time_to_go.py).

Guidance answers one question: **"Given where the target is and how it's moving, in which
direction and how hard should I accelerate to hit it?"** It outputs an *ideal*
acceleration vector — it does **not** worry about physical limits (that's the next stage).

### 8.1 The intuition: don't chase, intercept

A naïve drone would always point straight at the target and chase it — "pure pursuit."
Against a moving target this is terrible: you're always aiming where it *was*, curving in
behind it, arriving late. Real interceptors (and missiles, and interceptor insects) use
**Proportional Navigation (PN)**: aim at where the target *will be* by nulling the LOS
rotation.

Recall from Part 2.4: **if the line of sight to the target isn't rotating, you're on a
collision course.** PN turns this into a control law: command acceleration proportional to
the LOS rotation rate, so as to drive that rotation to zero. The proportionality constant
is the **navigation ratio** `N'` (typically 3–5).

### 8.2 Zero-Effort-Miss (ZEM)

This project uses a cleaner, more modern formulation built on **Zero-Effort-Miss**. ZEM
answers: *"If I did nothing more from now on — no further acceleration — by how much would
I miss?"* It's the predicted miss vector:

```
ZEM = r + v·t_go + ½·a·t_go²
```

where `r`, `v`, `a` are the relative position, velocity, acceleration, and `t_go` is the
**time-to-go** (estimated time until intercept). The three terms are exactly the
"predict future position" formula from Part 2.1: where the gap will be if you coast.

Guidance then commands acceleration to **drive ZEM toward zero**:

```
a_cmd = N' / t_go² · ZEM
```

The `½·a·t_go²` term is what makes this "augmented" — it folds in the target's estimated
*acceleration*, letting the law anticipate an evasive, maneuvering target. (This term is
currently **switched off** — see 8.6 — because against non-maneuvering targets it can
cause instability; it's Phase 4 work.)

### 8.3 Time-to-go: [`time_to_go.py`](../src/interceptor/guidance/time_to_go.py)

`t_go` = distance / closing speed. **Closing speed** is how fast the range is shrinking:
`−(r·v)/|r|`, positive when approaching. But there's a problem: at the very start, the
interceptor is *at rest* — closing speed is ~0, and distance/0 = infinity. And near
intercept, `t_go → 0`, and the `1/t_go²` term explodes.

So `time_to_go` is carefully conditioned:

- If genuinely closing, use `range / closing_speed`.
- If closing speed is ~0 (static target or a from-rest launch), synthesize a `t_go` from a
  **reference closing speed** (default 3.5 m/s) so guidance still produces a sensible
  "start accelerating toward it" command.
- **Clamp** the result to `[0.05 s, 30 s]` to bound the terminal blow-up.

That 3.5 m/s reference was **tuned in Phase 3** (down from 5.0): too high and the launch
command was so aggressive it saturated the motors in the first 20% of flight; too low and
the farthest target wasn't reached in time. It's a documented trade-off.

### 8.4 The lag-aware navigation ratio

Here's what makes it the *Optimal* Guidance Law rather than plain PN. A real quadcopter
**can't change its tilt instantly** — its attitude follows a first-order lag, modeled by
the transfer function `1/(T·s + 1)` with time constant `T` (0.2 s). Plain PN ignores this
and turns out too gently near intercept, then can't catch up.

OGL solves the underlying optimization *including* the lag, which yields a **time-varying
navigation ratio** `N'(x)` where `x = t_go / T` (this is the classic closed-form Zarchan
solution):

```
N'(x) = 6·x²·(e^(−x) − 1 + x) / (2x³ − 6x² + 6x + 3 − 12·x·e^(−x) − 3·e^(−2x))
```

Behaviour:
- **Far from intercept** (`x` large): `N' → 3`, i.e. it behaves like ordinary PN.
- **Near intercept** (`x` small): `N'` **grows**, steering harder *in advance* to
  pre-empt the tilt lag.

The result is clamped to `[3, 5]` for numerical safety. This is a "smart" navigation
gain that knows the drone is sluggish and compensates ahead of time.

### 8.5 The altitude penalty `b`

The design review flags **altitude (Z) overshoot** as a recurring problem: interceptors
tend to shoot up past the target's height and oscillate back down. OGL includes a tuning
knob `b` (default **0.1**) that **de-weights the vertical command** — mathematically,
from an optimization that penalizes vertical control effort. In code it appears as an
attenuation of the Z channel: `a_cmd_z ·= 1/(1 + b)`. Changing `b` affects a KPI, so it
requires explicit sign-off.

(Interesting Phase-3 finding: with this project's *differential-flatness* controller, the
drone barely overshoots altitude anyway, so `b`'s measured effect was near-zero. It's kept
as cheap insurance and to stay faithful to the design review, to be re-evaluated against
fast evasive targets in Phase 4.)

### 8.6 Putting it together (`OptimalGuidanceLaw.compute`)

```
r, v   = estimated relative position, velocity
a      = estimated relative acceleration  (currently zeroed — see below)
t_go   = time_to_go(r, v)                         # conditioned & clamped
N'     = clamp(lag_aware_nav_ratio(t_go, T), 3, 5)
ZEM    = r + v·t_go + ½·a·t_go²
a_cmd  = N' / t_go² · ZEM
a_cmd[Z] *= 1/(1 + b)                             # altitude penalty
```

The **augmented term is gated off** (`use_target_acceleration = False`) for now: the EKF
estimates *relative* acceleration, which against a non-maneuvering target mostly reflects
the interceptor's *own* maneuvering — feeding that back would be positive feedback and
destabilize the loop. Isolating the target's *absolute* acceleration is deferred to Phase
4 (evasive targets).

OGL sits behind a clean `GuidanceLaw` interface so a future law could be swapped in
without touching any caller — but per project scope, **OGL is the sole law**.

---

## 9. Stage 4 — Command Limiter (safety)

**Role:** Flight Control & Actuation Engineer. **File:**
[`control/command_limiter.py`](../src/interceptor/control/command_limiter.py).

Guidance can ask for anything — including accelerations the drone physically can't
produce. The limiter is the **single place** that clamps the request to something safe,
and the single place that **measures saturation**. Two bounds:

- **Tilt limit.** Since horizontal acceleration comes only from tilting, the maximum
  horizontal acceleration is `g · tan(max_tilt)`. With the Phase-4-tuned max tilt of 60°
  (1.0472 rad), that's `9.81 · tan(60°) ≈ 17.0 m/s²`. A larger horizontal request is
  scaled back to this cap.
- **Total magnitude limit.** The overall acceleration magnitude is capped at
  `max_acceleration` (40 m/s²) to protect the rotors.

Every clamp is **reported**: a `saturated` flag plus how much acceleration was removed,
and it's logged with a warning. Why so careful? Because **"command saturation ≤ 5% of
flight time" is a graded KPI** — being pinned at the limit means you've lost control
authority, and the project measures exactly how often that happens. Concentrating all
clamping here (and nowhere else) means saturation is counted in exactly one place.

That tilt limit has been raised twice, each with sign-off: **35° → 45° in Phase 3** (at 35°
the horizontal authority was only ~6.87 m/s² and cross-range dashes kept hitting it), then
**45° → 60° in Phase 4** (at 45° = a full `g`, chasing weaving and 90 km/h targets still
clamped for 15–30% of a short engagement — the dominant saturation-KPI miss on the randomized
batch). 60° raises horizontal authority to ~17 m/s², which lifted randomized-batch mission
success from ~57% to **93%**; the total-magnitude cap was raised 30 → 40 m/s² at the same
time. 60° is the aggressive-but-physical end for an interceptor — going further (65°) began to
*overshoot* easy static targets. The airframe's ~250 m/s² thrust capacity means these limits
are real authority, not saturation hidden in the motors.

---

## 10. Stage 5 — Flight Control (the dual loop)

**Role:** Flight Control & Actuation Engineer. **Files:**
[`control/outer_loop.py`](../src/interceptor/control/outer_loop.py),
[`control/inner_loop.py`](../src/interceptor/control/inner_loop.py).

Now we have a safe *acceleration* we want. But the drone doesn't take "acceleration" as an
input — it takes tilt and thrust, and ultimately motor speeds. Flight control bridges this
gap with **two nested loops running at different speeds**:

- **Outer loop (~50 Hz):** "what tilt and thrust achieve this acceleration?"
- **Inner loop (~400 Hz):** "spin the body toward that tilt, fast, using the gyroscope."

Running them at different rates is deliberate and **must not be collapsed** — the fast
inner loop stabilizes the aircraft while the slower outer loop steers it. This mirrors how
real flight controllers are built.

### 10.1 Outer loop — differential flatness

**File:** `outer_loop.py`. A quadcopter is what's called a **differentially flat** system,
which (in plain terms) means: *given a desired acceleration, there is a direct algebraic
formula for the required tilt and thrust — no feedback tuning needed.* Here's the physics:

The rotors can only push along the body's up-axis. To produce a desired world-frame
acceleration `a_cmd`, the total "specific force" the rotors must generate is the commanded
acceleration **plus** the force to hold up against gravity:

```
f = a_cmd + [0, 0, g]        (the vector the thrust must point along)
```

From this single vector everything follows:

- **Thrust magnitude** = `mass · |f|`.
- **Tilt direction:** the body's up-axis must point along `f`. That fixes the required
  roll and pitch (yaw is held at 0). The formulas are:
  ```
  roll  = atan2(−f_y, hypot(f_x, f_z))
  pitch = atan2(f_x, f_z)
  ```

So if `a_cmd = 0` (just hover), `f = [0,0,g]`, giving zero tilt and thrust = `m·g` (exactly
counteracting weight). Tilt more, and part of the thrust vectors sideways. This is a clean,
gain-free map from acceleration to attitude — one reason the system overshoots altitude so
little.

### 10.2 Inner loop — the attitude PID

**File:** `inner_loop.py`. The outer loop said "I want *this* tilt." The inner loop makes
it happen, fast, using the **gyroscope** (which measures body rotation rates). This is
where the tilt delay becomes *real* — we don't fake instant tilting; we run an actual fast
controller and let the attitude respond at its natural speed, which is exactly the lag OGL
was designed to anticipate.

A subtlety: a rate gyro measures *angular velocity*, not the current orientation. So the
controller maintains its **own** attitude estimate by **integrating** the gyro over time
using quaternion math (a "strapdown" integration), seeded level at start. Then it runs a
**PD controller**:

```
attitude_error   = desired_attitude − current_attitude
angular_accel    = kp · attitude_error − kd · body_rate
torque           = inertia · angular_accel
```

**What's a PID/PD controller?** The workhorse of control engineering:
- **P (Proportional):** push in proportion to the error — the bigger the tilt error, the
  harder you correct. (`kp` term)
- **D (Derivative):** push *against* the current rate of change to damp out oscillation and
  avoid overshoot. (`−kd · body_rate` term)
- **I (Integral):** accumulate small persistent errors to eliminate steady offset (reserved
  here for later tuning, currently zero).

The gains are tuned so roll/pitch respond crisply (natural frequency ~17 rad/s, well
inside the 400 Hz loop). **Yaw gains are ~100× smaller** on purpose: yaw torque comes from
weak rotor-drag differential, so a big yaw gain would demand impossible rotor imbalances
and saturate all four motors — and since yaw doesn't help interception anyway, a gentle
yaw hold is all that's needed.

The output is a `BodyTorqueThrustCommand`: the three torques (roll/pitch/yaw) the body
should generate, plus the collective thrust passed through from the outer loop.

---

## 11. Stage 6 — Motor Mixer (down to four numbers)

**Role:** Flight Control & Actuation Engineer. **File:**
[`control/motor_mixer.py`](../src/interceptor/control/motor_mixer.py).

The final translation: turn "I want this much total thrust and these three torques" into
**four individual rotor RPMs**. This is the exact algebraic **inverse** of the rotor model
from Part 6.3 — and it deliberately reads the *same* physical constants, so the forward
model (physics) and inverse model (mixer) can never disagree.

Recall the forward relationships (with `f_i = kT·rpm_i²` the thrust of rotor `i`):

```
total thrust = f_front + f_right + f_back + f_left
roll torque  = arm·(f_left − f_right)
pitch torque = arm·(f_back − f_front)
yaw torque   = −(kQ/kT)·(f_front − f_right + f_back − f_left)
```

That's four equations in four unknowns (`f_front, f_right, f_back, f_left`). The mixer
solves this linear system for the four thrusts, then inverts `thrust = kT·rpm²` to get
`rpm = √(f / kT)`.

Then the **hard physical limit**: RPMs are clamped to `[0, 25000]`. If a solution demands
a *negative* thrust (impossible — a rotor can't push down) or exceeds max RPM, the mixer
clamps and **logs a saturation warning**. It never silently exceeds the actuator bounds.
The four resulting RPMs go back to the plant, closing the loop.

---

## 12. How the stages are wired together

**Role:** Integration Architect. **Files:**
[`pipeline/orchestrator.py`](../src/interceptor/pipeline/orchestrator.py),
[`pipeline/scheduler.py`](../src/interceptor/pipeline/scheduler.py),
[`common/types.py`](../src/interceptor/common/types.py).

### 12.1 The data contracts

Each arrow in the pipeline is a specific, **immutable, self-validating** message defined
in `common/types.py`:

| Message | From → To | Carries |
| :-- | :-- | :-- |
| `RawSensorMeasurement` | Sim → Estimation | range, LOS azimuth/elevation, timestamp, latency |
| `TargetStateEstimate` | Estimation → Guidance | relative pos/vel/accel, range, LOS rate, covariance, quality |
| `AccelerationCommand` | Guidance → Limiter | the ideal (unclamped) acceleration |
| `LimitedAccelerationCommand` | Limiter → Control | clamped acceleration + saturation flag/magnitude |
| `AttitudeReference` | Outer → Inner | target roll/pitch/yaw + thrust |
| `BodyTorqueThrustCommand` | Inner → Mixer | three body torques + thrust |
| `MotorCommand` | Mixer → Sim | four rotor RPMs |

Each is a **frozen dataclass**: its arrays are made read-only so a downstream stage can't
mutate a producer's data, and its `__post_init__` validates shapes and **rejects
NaN/Inf** on construction. Units are stated on every field. A stage literally *cannot*
receive data across a boundary it isn't supposed to — the type system enforces the
architecture.

### 12.2 The multi-rate scheduler

**File:** `scheduler.py`. Different stages run at different rates:

| Loop | Rate |
| :-- | :-- |
| Physics step & inner control loop | 400 Hz |
| Outer control loop & guidance | 50 Hz |
| Estimation (sensor cadence) | 100 Hz |

The scheduler turns these into **integer periods in sim-steps** (e.g. the 50 Hz outer loop
fires every 8th step) so loops fire on exact step boundaries with **no floating-point
drift**. It refuses to construct if any rate doesn't evenly divide the sim rate (fail
loud). Each `Tick` it yields carries booleans saying which loops are due this step. This is
what makes runs bit-for-bit reproducible.

### 12.3 The orchestrator

**File:** `orchestrator.py`. The `StubOrchestrator` runs the loop. Each step it:

1. Reads the interceptor position (from the plant) and target position (from the
   trajectory).
2. If estimation is due: takes a sensor measurement, updates the EKF.
3. If guidance is due: runs OGL, then the limiter.
4. If the outer loop is due: computes the attitude reference.
5. If the inner loop is due: runs the attitude PID (using the gyro), then the mixer.
6. Steps the physics with the latest motor command.
7. Logs one row (positions, orientation, estimate, saturation, RPMs).

Slower loops **reuse** their most recent output on steps where they don't fire. The stages
are **injected** (`PipelineComponents`) rather than hard-coded — this is **dependency
inversion**. The same orchestrator runs the trivial Phase-0 "stub" components (pass-through
placeholders) *and* the real Phase-2+ components (MuJoCo, EKF, OGL, ...) with **zero
changes** — you just inject different implementations behind the same interfaces.

### 12.4 Engagement termination

By default a run stops **at closest approach**: once the interceptor gets within a capture
radius (2 m) and the range then starts *growing*, the intercept moment has passed — flying
on would be physically meaningless thrashing (past the target, OGL's geometry inverts). So
the loop breaks and the log ends exactly at the point of closest approach. This also keeps
the saturation KPI honest — it's measured over the real engagement, not a pointless flyby
tail.

---

## 13. Configuration: constants vs. tunable parameters

Two files, a deliberate split, in [`config/`](../src/interceptor/config/):

### 13.1 [`constants.py`](../src/interceptor/config/constants.py) — physical & structural

The **single source of truth** for numbers that rarely change: gravity, air density,
drone mass/inertia, arm length, motor RPM limits, thrust/torque coefficients, loop rates,
the tilt-delay time constant, and all the **KPI thresholds**. Every value has explicit
units *in its name* (e.g. `MOTOR_RPM_MAX`, `TILT_DELAY_TIME_CONSTANT_S`) and a comment
explaining *why* it exists. **No magic numbers** are allowed elsewhere — everything traces
back here. Some airframe figures are honest placeholders to be refined by the Simulation
Engineer; they're labeled as such.

### 13.2 [`params.py`](../src/interceptor/config/params.py) — tunable knobs

The values engineers **sweep** while tuning: EKF noise covariances, PID gains, the
navigation-ratio bounds, the reference closing speed, the limiter bounds, the sensor noise
profile, the wind profile. These are plain dataclasses so they serialize cleanly into the
per-run reproducibility snapshot. A **YAML scenario file can override any of them** via a
deep merge — and a typo'd key **fails loud** rather than silently doing nothing.

This split matters: physics constants changing would mean a different drone; tuning
parameters changing is normal engineering. The two are governed differently (changing a
KPI-affecting default requires user sign-off).

---

## 14. Determinism and reproducibility

Determinism is the project's north star, and three files enforce it, in
[`common/`](../src/interceptor/common/):

- **[`rng.py`](../src/interceptor/common/rng.py)** — *all* randomness flows through one
  seeded `RngFactory`. Each stochastic component (sensor, wind) gets its own **named,
  independent** random stream derived from the root seed. Adding a new random component
  never disturbs the numbers seen by existing ones. **Nobody** is allowed to call the
  global `random`/`np.random`.
- **[`logging.py`](../src/interceptor/common/logging.py)** — writes one CSV row per
  timestep (deterministic: fixed column order, fixed float formatting, `\n` newlines) and a
  `run_config.json` snapshot recording the **seed, resolved parameters, and git commit
  hash** for every run.
- **[`guards.py`](../src/interceptor/common/guards.py)** — the fail-loud helpers
  (`ensure_finite`, `ensure_vector`, `ensure_in_range`, `freeze`) used everywhere to reject
  bad data immediately.

The guarantee: **identical seed + identical config ⇒ byte-identical run log**, on any
machine. That's what makes results trustworthy and regressions detectable.

---

## 15. Testing, KPIs, and scenarios

**Role:** Test, Validation & KPI Engineer. **Files:** [`analysis/`](../src/interceptor/analysis/),
[`tests/`](../tests/), [`scenarios/`](../scenarios/).

### 15.1 The KPIs (the report card)

Success is defined by six measured metrics, each with a target (with a 5% margin baked in):

| Metric | Target | Meaning |
| :-- | :-- | :-- |
| **Miss distance** `R_miss` | ≤ 1.05 m | Closest the interceptor got. A "hit" if within this. |
| **Time-to-intercept** | Static < 10 s, Moving < 20 s | How quickly it closed. |
| **Z-axis overshoot** | ≤ 0.5 m | How far it flew *past* the target's altitude. |
| **Command saturation** | ≤ 5% of flight time | Fraction of time pinned at a physical limit. |
| **Max target speed** | ≥ 83.6 km/h | Fastest target it can still defeat. |
| **Mission success rate** | ≥ 90% | Fraction of randomized trials that hit (Phase 4). |

### 15.2 KPI measurement: [`kpis.py`](../src/interceptor/analysis/kpis.py)

The **single source of truth** for turning a raw `run_log.csv` into graded metrics. It
never re-runs physics — it only *measures* a recorded run. Notable subtlety: **Z-overshoot
is measured sign-aware** — for a climbing intercept it's how far the interceptor rose
*above* the target; for a descending one, how far it sank *below*. This fixed a bug where a
descending approach counted its benign initial altitude gap as huge "overshoot."

### 15.3 Scenarios: [`scenarios.py`](../src/interceptor/analysis/scenarios.py) + [`scenarios/*.yaml`](../scenarios/)

A **scenario** is a small YAML file fully specifying one trial: seed, target trajectory,
interceptor start, time limit, optional parameter overrides, and (always) OGL. Example
([`static_diagonal.yaml`](../scenarios/static_diagonal.yaml)):

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

The runner reuses the existing trajectory generators and the exact same closed loop —
it only *declares and drives*, no physics logic of its own. It fails loud on unknown
trajectory types, missing keys, or a non-OGL law. A scenario may also name a `wind_preset`
(`calm`/`moderate`/`gusty`) as a shorthand for the wind profile. The library has **6 static +
5 linear** geometries (Phase 3), a `scenarios/ablation/` pair for the `b`-penalty study, and
**`scenarios/phase4/`** with **4 sinusoidal (evasive) + 3 varying-speed (high-speed, to 90
km/h) + 4 wind** stress scenarios (Phase 4).

### 15.3b Randomized Monte-Carlo trials: [`montecarlo.py`](../src/interceptor/analysis/montecarlo.py)

Named scenarios probe *specific* geometries; the **Monte-Carlo harness** samples the *whole*
threat envelope. From one `master_seed` it draws a seeded batch of randomized 3D engagements
(geometry in a frontal cone, a weighted trajectory family, family parameters, and a weighted
wind preset), turns each draw into a validated `Scenario`, and flies it. It measures the
**Mission Success Rate = interception fraction** (matching the design review's *"≥ 90%
interception"*), reports the other KPIs as separate compliance rates, and breaks results down
per family and per wind preset so weak regimes are exposed, not hidden. Each trial's run seed
is its index, so `(master_seed, num_trials)` reproduces the whole batch byte-for-byte, and a
batch manifest records the master seed + git hash + the committed tuning. This is what
certifies the Phase 4 headline: **93% mission success**, a **90 km/h**-class target
intercepted, and interception essentially flat under wind.

### 15.4 Reporting: [`reporting.py`](../src/interceptor/analysis/reporting.py)

Turns a batch of scenario results into a KPI summary table (CSV + Markdown) and
per-scenario diagnostic plots (X-Y geometry, altitude-vs-time with the overshoot band,
range + command-effort with saturated frames shaded). It forces matplotlib's **headless
"Agg" backend** so it never opens a window — safe for automated runs.

### 15.5 The test suite

Under [`tests/`](../tests/), split into `unit/` (per-component: EKF, guidance, control,
sensors, trajectories, frames, KPIs, Monte-Carlo sampling/aggregation, wind wiring, ...) and
`integration/` (whole-pipeline: stub loop, real interception, scenario suites, a reproducible
Monte-Carlo batch, MuJoCo headless render). Tests are **headless, non-interactive, and
seeded**. As of Phase 4: **208 passing tests**. Tests that need the GL context are marked
`mujoco` so they can be skipped where there's no display.

Key rule from `AGENTS.md`: *the Test/KPI role measures and reports faithfully; it never
tweaks the guidance/control internals or relaxes a target to manufacture a pass.* When it
finds a systemic failure, it files a finding for the owning role — as happened with the
saturation KPI in Phase 3.

---

## 16. The project map, file by file

```
Workshop_Autonomous_Systems/
├── AGENTS.md                  # The engineering contract — rules, roles, boundaries
├── CLAUDE.md / GEMINI.md      # Point AI assistants at AGENTS.md
├── README.md                  # Quick-start
├── pyproject.toml             # Pinned dependencies (mujoco, numpy, scipy, pyyaml, matplotlib)
│
├── docs/
│   ├── Autonomous_Drone_Interceptor_Design_Review.md   # THE authoritative design
│   ├── implementation_plan.md, phase0..4.md            # Phased roadmap
│   ├── phase*_progress.md                              # What was actually built each phase
│   └── PROJECT_EXPLAINED.md                            # (this document)
│
├── models/                    # MuJoCo world (MJCF XML)
│   ├── scene.xml              # World: solver, floor, lights; includes the two bodies
│   ├── quadcopter.xml         # The interceptor body
│   └── target.xml             # The target (a kinematic "mocap" body)
│
├── scenarios/                 # Declarative trial configs (YAML)
│   ├── static_*.yaml          # 6 static-target geometries
│   ├── linear_*.yaml          # 5 constant-velocity geometries
│   ├── ablation/*.yaml        # b=0 controls for the altitude-penalty study
│   └── phase4/*.yaml          # 4 sinusoidal + 3 varying-speed + 4 wind stress scenarios
│
├── scripts/                   # Entry points
│   ├── check_env.py           # Environment doctor (verifies MuJoCo, off-screen render)
│   ├── run_stub_pipeline.py   # Phase 0 — the loop on pass-through stubs
│   ├── run_intercept.py       # Phase 2 — real guided interception, static target
│   ├── run_scenarios.py       # Phase 3 — run scenario(s), print KPI table, optional report
│   ├── run_montecarlo.py      # Phase 4 — randomized 3D Monte-Carlo mission-success batch
│   ├── run_sim_demo.py        # Simulation-only demo
│   └── replay.py              # Interactive replay viewer of a recorded run
│
├── src/interceptor/
│   ├── config/
│   │   ├── constants.py       # Physical constants + KPI thresholds (one source of truth)
│   │   └── params.py          # Tunable parameters (EKF/PID/guidance/limiter/sensor/wind)
│   ├── common/
│   │   ├── types.py           # The 7 immutable pipeline messages (data contracts)
│   │   ├── frames.py          # Coordinate frames, quaternions, LOS math
│   │   ├── rng.py             # Seeded, named RNG streams (determinism)
│   │   ├── logging.py         # Per-step CSV + reproducibility snapshot
│   │   └── guards.py          # Fail-loud validators
│   ├── simulation/            # STAGE 1 (Role 1)
│   │   ├── mujoco_plant.py    # MuJoCo wrapper — the plant
│   │   ├── actuators.py       # Rotor thrust/torque model + RPM saturation
│   │   ├── sensors/noisy_sensor.py   # Noisy, biased, delayed sensor
│   │   ├── trajectories/generators.py# Static/linear/sinusoidal/varying-speed/wind targets
│   │   ├── kinematics.py      # Ground-truth geometry (sim-only; never leaks)
│   │   ├── wind.py            # Steady wind + OU-process gusts
│   │   ├── rendering.py       # Off-screen renderer
│   │   ├── interfaces.py      # Plant / SensorModel / TargetTrajectory / Renderer ABCs
│   │   └── stubs.py           # Trivial pass-through implementations (Phase 0/tests)
│   ├── estimation/            # STAGE 2 (Role 2)
│   │   ├── ekf.py             # The Extended Kalman Filter
│   │   ├── interfaces.py      # Estimator ABC
│   │   └── stubs.py           # Pass-through estimator
│   ├── guidance/              # STAGE 3 (Role 3)
│   │   ├── ogl.py             # Optimal Guidance Law (the sole law)
│   │   ├── zem.py             # Zero-Effort-Miss
│   │   ├── time_to_go.py      # Conditioned time-to-go
│   │   ├── interfaces.py      # GuidanceLaw ABC
│   │   └── stubs.py           # Zero-guidance stub
│   ├── control/               # STAGES 4 & 5 (Role 4)
│   │   ├── command_limiter.py # Saturation — tilt & magnitude clamps
│   │   ├── outer_loop.py      # Differential-flatness accel → attitude
│   │   ├── inner_loop.py      # Gyro-fed attitude PID → body torque
│   │   ├── motor_mixer.py     # Torque+thrust → four rotor RPMs
│   │   ├── interfaces.py      # Controller/mixer ABCs
│   │   └── stubs.py           # Pass-through controllers
│   ├── pipeline/              # STAGE wiring (Role 6)
│   │   ├── orchestrator.py    # Runs the 6-stage loop; injects components
│   │   └── scheduler.py       # Deterministic multi-rate clock
│   └── analysis/              # KPI/validation (Role 5)
│       ├── kpis.py            # Measure metrics from a run log
│       ├── scenarios.py       # Declarative scenario runner
│       ├── montecarlo.py      # Randomized 3D Monte-Carlo harness + aggregation
│       └── reporting.py       # Summary tables, batch dataset/manifest, diagnostic plots
│
├── tests/                     # unit/ + integration/ — headless, seeded
└── results/                   # Generated run logs, snapshots, reports (per run_id)
```

---

## 17. How to run everything

From the project root, on Windows (PowerShell). MuJoCo is installed at
`C:/Dev/Libraries/mujoco`; the pip `mujoco` wheel bundles its own libraries.

```powershell
# One-time setup
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -e ".[dev]"

# Verify the environment (checks Python, imports, MuJoCo, one off-screen frame)
python scripts/check_env.py

# The whole 6-stage loop on trivial stubs (Phase 0), deterministic
python scripts/run_stub_pipeline.py --steps 400 --seed 0

# A real guided interception against a static target (Phase 2)
python scripts/run_intercept.py --target 8 3 6 --seconds 9
python scripts/replay.py results/intercept                    # watch it (top view)
python scripts/replay.py results/intercept --view interceptor # chase cam

# Run a declarative scenario, or the whole suite with a KPI report (Phase 3)
python scripts/run_scenarios.py scenarios/linear_crossing.yaml
python scripts/run_scenarios.py scenarios/ --report           # -> results/phase3/

# The evasive/high-speed/wind stress probes, and the randomized batch (Phase 4)
python scripts/run_scenarios.py scenarios/phase4 --results-dir results/phase4
python scripts/run_montecarlo.py --trials 100 --seed 0 --report --results-dir results/phase4/montecarlo

# The tests
pytest                    # everything (~208 tests)
pytest -m "not mujoco"    # skip the off-screen GL render test
```

Every run writes a folder under `results/<run_id>/` containing `run_log.csv` (per-step
data) and `run_config.json` (seed + params + git hash) — enough to replay or reproduce it
exactly.

---

## 18. Current status and roadmap

The project is built in four phases (from the design review):

- **Phase 0 — ✅ Done.** Environment, project skeleton, data contracts, the stub pipeline
  proving the loop closes deterministically.
- **Phase 1 — ✅ Done.** The MuJoCo world: quadcopter/target models, rotor actuator model,
  noisy/delayed sensors, trajectory generators, wind, ground-truth kinematics.
- **Phase 2 — ✅ Done.** The real algorithms wired in: EKF, OGL, command limiter, dual-loop
  control, motor mixer. The loop intercepts static targets to well within 1.05 m.
- **Phase 3 — ✅ Done.** KPI + scenario tooling, and tuning to *meet spec* on static and
  linear targets: **11/11 scenarios pass every KPI** (miss distances 0.005–0.063 m, all
  saturation ≤ 5%). Two tuning changes were made with sign-off: reference closing speed
  5.0 → 3.5 m/s, and max tilt 35° → 45°.
- **Phase 4 — ✅ Done.** Randomized 3D trials against **evasive** (sinusoidal),
  **high-speed** (to 90 km/h), and **windy** targets, plus a seeded Monte-Carlo harness for
  the mission-success KPI. Headline results: **93% mission success (interception)**, a
  cleanly intercepted **89.7 km/h** target, and Z-overshoot within KPI, with wind robustness
  confirmed. One params-only tuning change was made with sign-off: max tilt 45° → 60° (and
  the total-accel cap 30 → 40 m/s²). The residual **command-saturation** tail on very short
  high-speed intercepts is characterized and filed for a future adaptive-authority refinement
  (see `docs/phase4_progress.md`). The current git branch is `phase-4`.

Three design decisions worth remembering (recorded in the project's memory):
- **OGL is the sole guidance law.** PN/APN were evaluated and rejected; they are not
  implemented.
- **The augmented-ZEM term remains gated off.** With a relative-state EKF it would feed the
  interceptor's own maneuver back as positive feedback; it stays the candidate next step for
  the fast-crossing tail rather than a shipped feature.
- **Mission success is measured as *interception***, per the design review — the other KPIs
  (saturation, Z-overshoot, time) are reported as separate compliance rates.

---

## 19. Glossary

- **Attitude** — the drone's orientation (roll/pitch/yaw).
- **Azimuth / Elevation** — the horizontal / vertical angles of the line of sight.
- **Body frame** — coordinates fixed to the drone (X forward, Y left, Z up).
- **Closing speed** — how fast the interceptor–target range is shrinking.
- **Covariance** — a matrix expressing the Kalman filter's uncertainty about its estimate.
- **Determinism** — same inputs always produce byte-identical outputs.
- **Differential flatness** — property letting us map a desired acceleration directly to
  tilt + thrust with an algebraic formula.
- **EKF (Extended Kalman Filter)** — the estimator; a Kalman filter linearized to handle
  the nonlinear range/angle sensor.
- **Fail loud** — raise an error on bad data (NaN, divergence, out-of-range) instead of
  continuing silently.
- **Innovation** — in a Kalman filter, the difference between the actual and expected
  measurement.
- **Jacobian** — the matrix of partial derivatives used to linearize the sensor equations.
- **Kalman gain** — the weight blending prediction vs. measurement each update.
- **KPI** — Key Performance Indicator; a graded success metric.
- **Line of Sight (LOS)** — the straight line from interceptor to target; its *rate* is
  central to guidance.
- **Mocap body** — a body whose pose is prescribed (teleported), not physically simulated.
- **MJCF** — MuJoCo's XML format for describing a physics model.
- **Motor mixer** — converts desired thrust + torques into four rotor speeds.
- **Navigation ratio (N')** — the proportionality gain in the guidance law (3–5).
- **OGL (Optimal Guidance Law)** — the sole guidance law; a lag-aware, altitude-penalized
  Zero-Effort-Miss law.
- **Ornstein–Uhlenbeck process** — a way to generate smooth, correlated random gusts.
- **PID / PD controller** — Proportional-Integral-Derivative feedback controller; the
  inner loop uses PD.
- **Plant** — control-theory term for the system being controlled (here, the MuJoCo drone).
- **Process noise** — the Kalman filter's assumed uncertainty in its motion model.
- **Quaternion** — a 4-number encoding of a 3D rotation, robust and gimbal-lock-free.
- **RPM** — revolutions per minute; the rotor speed command.
- **Saturation** — a command hitting a physical limit (clamped); tracked as a KPI.
- **Tilt delay** — the drone's inability to change attitude instantly, modeled as a
  first-order lag `1/(T·s+1)`.
- **Time-to-go (t_go)** — estimated time remaining until intercept.
- **Wrench** — a combined force + torque acting on a body.
- **World frame** — the fixed ground frame (Z up = altitude).
- **Zero-Effort-Miss (ZEM)** — the predicted miss vector if no further acceleration were
  applied; guidance drives it to zero.

---

*This document reflects the codebase as of the `phase-4` branch. For the authoritative
design rationale see the Design Review; for the engineering rules see `AGENTS.md`; for what
was actually built and verified in each phase see the `docs/phase*_progress.md` reports.*
