# Design Review: Autonomous Quadcopter Drone Interceptor

**Author:** Dor Varsulker  
**Topic:** Workshop in Autonomous Systems Simulation  

---

## 1. Executive Summary & Context

The rapid proliferation of Unmanned Aerial Vehicles (UAVs) has necessitated the development of advanced Counter-UAS (C-UAS) technologies. This design review explores the architecture, simulation, and algorithmic backbone of an **Autonomous Quadcopter Drone Interceptor**. The core objective is to design a system capable of autonomously tracking, navigating towards, and physically intercepting (or neutralizing) dynamic, evasive target drones within a simulated 3D environment.

## 2. Project Requirements & Constraints

To ensure a reliable and physically viable interception system, the project operates under strict functional and technical parameters.

### Functional Requirements

* **Target Detection & Tracking:** Continuous, real-time estimation of the relative position, range, and Line-of-Sight (LOS) rate of dynamic target UAVs.
* **Autonomous Interception:** Computation of real-time guidance commands to ensure a reliable collision trajectory or neutralization.
* **Autonomous Flight Control:** Automatic translation of required acceleration vectors into specific motor speeds, maintaining quadcopter stability even during highly aggressive maneuvers.

### Technical Constraints

* **Actuator Saturation:** Motor RPM (Revolutions Per Minute) limits dictate that thrust vectoring must remain within the quadcopter's physical motor boundaries to prevent stalls or loss of control.
* **Sensor Noise & Latency:** Real-world sensors (Radar/LiDAR/Cameras) suffer from inherent delays and high-frequency noise. This raw data must be aggressively filtered before it can be fed into the guidance loops.

---

## 3. Simulation Environment: MuJoCo

The project leverages **MuJoCo (Multi-Joint dynamics with Contact)**, a highly accurate physics engine widely used in robotics and autonomous systems research.

**Environment Pipeline:**

1. **The Physics Engine (Constraint Solver):** Models continuous rigid body dynamics, fluid dynamics, and calculates energy conservation, stability, and integration.
2. **Active Space Modeling:** Defines the active 3D space, computing relative kinematics and look-angles between the interceptor and the target.
3. **Continuous Feedback Loop:** Models physical dynamics and aerodynamic forces, constantly looping sensor data back into the environment.

---

## 4. System Architecture Evaluation

The project evaluated two distinctly different approaches to mapping sensor data to motor commands: **Classical Hierarchical Architecture** and **Deep Reinforcement Learning**.

### Architecture A: Classical Hierarchical Architecture (Selected)

This approach breaks the interception problem into distinct, specialized, and mathematically explainable layers.

* **Target Estimation:** Uses an Extended Kalman Filter (EKF) or Moving Horizon Estimation (MHE) to process noisy sensor data.
* **Guidance:** Uses geometric guidance algorithms (e.g., Optimal Guidance Law) to compute required acceleration.
* **Control:** Translates acceleration into target roll/pitch angles and applies stabilizing corrections.
* **Actuators:** Motor Mixers convert physical limits into RPMs for the 4 rotors.

### Architecture B: Deep Reinforcement Learning (DRL)

This approach bypasses traditional control theory, feeding state information (interceptor orientation, relative target vectors) directly into a Deep Neural Network (trained via PPO or SAC) which outputs low-level actuator commands directly.

### The Verdict: Why Classical over Reinforcement Learning?

While DRL has the potential to discover "super-maneuverable" flight profiles, the **Classical Hierarchical Architecture was chosen** for its deterministic reliability:

* **Deterministic Efficiency:** Calculus-driven frameworks guarantee the smoothest path, minimizing motor strain and battery usage.
* **Valid Physics:** Ensures repeatable performance within established laws of motion.
* *DRL Drawbacks:* RL agents often exploit simulation quirks, attempting physically impossible maneuvers that would cause real-world motor stalls. Furthermore, "black box" RL policies are highly sensitive to sensor noise, often resulting in erratic "twitching."

---

## 5. Deep Dive: Classical Architecture Components

The winning architecture operates through a 6-stage cyclic pipeline:

1. **Simulation:** Advances the physics simulation, calculating aerodynamic forces and updating 3D positions.
2. **Estimation (Perception):** An **Extended Kalman Filter (EKF)** processes raw, delayed, and noisy sensor data to extract clean estimates of the target's position and angular rates.
3. **Guidance Layer (Decision):** An advanced navigation algorithm computes the necessary acceleration vector to put the interceptor on a collision course.
4. **Command Limiter (Safety):** Enforces physical constraints. It clamps extreme acceleration requests to ensure the drone remains stable and the rotors are not damaged.
5. **Flight Control System:** Operates via a dual-loop system:
   * *Outer Loop (~50Hz):* Translates acceleration commands into target roll and pitch tilt angles.
   * *Inner Loop (~400Hz):* A rapid PID (Proportional-Integral-Derivative) controller that adjusts motor outputs based on real-time gyroscope data to match the target tilt.
6. **Motor Mixer:** Mathematically converts the overarching roll, pitch, yaw, and thrust commands into specific RPM values for each of the four individual rotors.

---

## 6. Guidance Algorithms Comparison

The core "brain" of the interception logic lies in the Guidance Algorithm. Three algorithms were evaluated:

### 1. Proportional Navigation (PN)

* **Concept:** A classical algorithm that aims ahead of the target to create a "collision triangle," commanding an acceleration perpendicular to the instantaneous Line-of-Sight (LOS) rate.
* **Limitations:** Assumes instantaneous turns and struggles with maneuvering targets.
* **Adaptation:** Partitioned into three 2D sub-problems (Sxy, Sxz, Syz) for 3D drone flight.

### 2. Augmented Proportional Navigation (APN)

* **Concept:** Builds upon PN by adding a feed-forward term to the "Zero Effort Miss" calculation, explicitly accounting for the target's evasive acceleration.
* **Limitations:** If the target maintains a constant cruising velocity (acceleration = 0), APN behaves identically to standard PN. It suffers from severe Z-axis (altitude) overshooting and fails when targets exceed 55-60 km/h.

### 3. Optimal Guidance Law (OGL) — The Winner

* **Concept:** Modeled as a **Linear Quadratic (LQ) optimization problem**. It explicitly anticipates the quadcopter's mechanical tilt delay (using a first-order lag transfer function $1/(Ts+1)$), rather than assuming the drone can turn instantly.
* **Mathematical Advantages:**
  * Minimizes both miss distance and control effort ($J = y(t_f)^2 + \int u(t)^2 dt$), saving battery and preventing violent maneuvers.
  * Dynamically adjusts steering aggressiveness using a time-varying Navigation Ratio ($N'$) based on the remaining "Time-to-Go".
* **Performance:** * By tuning the penalty parameter ($b=0.1$), it completely eliminates altitude overshooting.
  * Successfully tracks targets moving at up to **90 km/h**.
  * Reaches the target **12x faster** than standard PN/APN strategies.

---

## 7. Testing Scenarios & Success Criteria

To rigorously validate the interceptor, simulations cover a broad spectrum of real-world variables:

### Scenarios

* **Static Targets:** Baseline validation of algorithms and stability.
* **Linear Moving Targets:** Testing constant-velocity tracking and closing speeds.
* **Sinusoidal Trajectories:** Evasive maneuvers to stress-test the EKF tracking and OGL responsiveness.
* **Varying Target Speeds:** Scaling up to 90 km/h to test maximum physical constraints.
* **Wind & Gusts:** Introducing environmental disturbances to test control loop robustness.

### Key Performance Indicators (KPIs) & Success Criteria

| Metric | Description | Success Target (5% Margin) |
| :--- | :--- | :--- |
| **Miss Distance ($R_{miss}$)** | Proximity required for successful neutralization/blast radius. | $\le 1.05$ Meters |
| **Time-to-Intercept ($t_{int}$)** | Time efficiency of the OGL trajectory. | Static: $< 10$s Moving: $< 20$s |
| **Z-Axis Overshooting** | Altitude leveling precision. | Max $0.5m$ above target |
| **Command Saturation** | Ratio of time the drone is pushed to physical limits. | $\le 5\%$ of Total Flight Time |
| **Max Target Speed** | Maximum evasive threat speed successfully intercepted. | $\ge 83.6$ km/h |
| **Mission Success Rate** | Aggregate robustness across randomized 3D trials. | $\ge 90\%$ Interception Rate |

---

## 8. Implementation Roadmap

The project unfolds across four distinct phases:

* **Phase 1 (June 17 - June 30):** Initialize the MuJoCo 3D space, configure quadcopter aerodynamic models, and map simulated sensor noise profiles.
* **Phase 2 (July 1 - July 15):** Implement the Extended Kalman Filter (EKF) and integrate the Optimal Guidance Law (OGL) logic with physical command limiters.
* **Phase 3 (July 16 - August 5):** Fine-tune guidance parameters. Execute initial simulations against static and slow-moving linear targets.
* **Phase 4 (August 6 - August 20):** Execute extensive, randomized 3D trials against dynamic, evasive trajectories (sinusoidal, fast-moving, windy conditions) and compile the final performance data.
