"""Shared physical and system constants — the single source of truth.

Every value here carries explicit units in its name and a ``Why`` note tying it to
the Design Review / AGENTS.md. No downstream module may hard-code any of these
numbers (Clean Code → "No magic numbers"; DRY → one source of truth).

Phase 0 note: airframe/motor figures are *documented placeholders* refined by the
Simulation Engineer (Role 1) in Phase 1. They exist now so contracts and stubs have
typed, named values to reference — not to be trusted as final physics.

Coordinate frame reminder (see ``common.frames``): world frame is **Z-up**; the
altitude axis is Z. The Z axis gets first-class attention throughout because the
Design Review flags it as overshoot-sensitive.
"""

from __future__ import annotations

# --------------------------------------------------------------------------------
# Physics
# --------------------------------------------------------------------------------

GRAVITY_M_S2: float = 9.81
"""Standard gravitational acceleration [m/s^2]. Why: governs hover thrust and the
weight the motor mixer must counter every step."""

AIR_DENSITY_KG_M3: float = 1.225
"""Air density at sea level, 15 degC [kg/m^3]. Why: scales rotor thrust/drag and the
wind/gust disturbance model (Role 1)."""

WIND_DRAG_COEFF_N_PER_M_S: float = 0.05
"""Linear aerodynamic coupling between air-relative velocity and force [N per m/s].
PLACEHOLDER (Role 1, Phase 1). Why: the wind disturbance pushes the airframe with a
force F = k * (v_wind - v_body); a single lumped coefficient keeps the model explicit
and reproducible without a full aerodynamic surface model."""

# --------------------------------------------------------------------------------
# Airframe (PLACEHOLDERS — refined in Phase 1 by Role 1)
# --------------------------------------------------------------------------------

QUAD_MASS_KG: float = 1.0
"""Interceptor total mass [kg]. PLACEHOLDER. Why: sets weight (m*g) and translates
force commands to accelerations (a = F/m)."""

# Diagonal moments of inertia about body x/y/z [kg*m^2]. PLACEHOLDER.
# Why: map body torques to angular accelerations in the inner control loop.
QUAD_INERTIA_IXX_KG_M2: float = 0.01
QUAD_INERTIA_IYY_KG_M2: float = 0.01
QUAD_INERTIA_IZZ_KG_M2: float = 0.02

ARM_LENGTH_M: float = 0.15
"""Rotor arm length, center to rotor [m]. PLACEHOLDER. Why: lever arm converting
differential rotor thrust into roll/pitch torque in the motor mixer."""

# --------------------------------------------------------------------------------
# Motors  (saturation limits are SAFETY-critical — Role 4 must honor them)
# --------------------------------------------------------------------------------

MOTOR_RPM_MIN: float = 0.0
"""Minimum commandable rotor speed [RPM]. Why: a rotor cannot spin backwards;
clamping below this is a saturation event the limiter must surface."""

MOTOR_RPM_MAX: float = 25000.0
"""Maximum rotor speed [RPM]. PLACEHOLDER. Why: hard actuator ceiling; exceeding it
risks stall/loss of control. Command saturation against this bound is a tracked KPI
(<= 5% of flight time)."""

THRUST_COEFF_KT: float = 1.0e-7
"""Rotor thrust coefficient kT so that thrust = kT * RPM^2 [N per RPM^2].
PLACEHOLDER. Why: converts rotor speed to lift in the motor mixer."""

TORQUE_COEFF_KQ: float = 1.0e-9
"""Rotor reaction-torque coefficient kQ so that torque = kQ * RPM^2 [N*m per RPM^2].
PLACEHOLDER. Why: yaw authority and gyroscopic reaction in the motor mixer."""

# --------------------------------------------------------------------------------
# Loop rates  (NEVER collapse the two control loops — Role 4 / Role 6)
# --------------------------------------------------------------------------------

SIM_HZ: int = 400
"""Physics integration rate [Hz]. Why: chosen equal to the inner-loop rate so the
multi-rate scheduler divides evenly (no float drift). Must be an integer multiple of
every slower loop rate below."""

INNER_LOOP_HZ: int = 400
"""Inner attitude/rate PID loop rate [Hz]. Why: Design Review — fast loop tracking
target tilt from gyro feedback."""

OUTER_LOOP_HZ: int = 50
"""Outer position/attitude-reference loop rate [Hz]. Why: Design Review — translates
acceleration commands into target roll/pitch tilt."""

ESTIMATION_HZ: int = 100
"""Estimator (EKF) update rate [Hz], tied to the sensor sample rate. PLACEHOLDER —
finalized with the sensor model in Phase 1. Why: the EKF runs at sensor cadence, not
the control cadence."""

GUIDANCE_HZ: int = 50
"""Guidance-law update rate [Hz]. Why: guidance issues acceleration commands at the
outer-loop cadence; kept as its own constant so it can diverge later if needed."""

# --------------------------------------------------------------------------------
# Guidance
# --------------------------------------------------------------------------------

TILT_DELAY_TIME_CONSTANT_S: float = 0.2
"""Mechanical tilt-delay time constant T in the first-order lag 1/(T*s + 1) [s].
PLACEHOLDER. Why: the quad cannot change attitude instantaneously; OGL must account
for this lag (Design Review). Never assume instantaneous turns."""

ALTITUDE_PENALTY_B: float = 0.1
"""Altitude (Z-axis) penalty weight b in the OGL cost [dimensionless]. Why: Design
Review default b = 0.1 eliminates Z-axis overshooting. Changing it affects a KPI and
needs user confirmation."""

NAV_RATIO_BASE: float = 3.0
"""Baseline navigation ratio N' for PN/APN baselines [dimensionless]. PLACEHOLDER.
Why: classic PN uses N' in [3, 5]; OGL replaces this with a time-to-go schedule."""

NAV_RATIO_MIN: float = 3.0
NAV_RATIO_MAX: float = 5.0
"""Bounds for the time-varying navigation ratio schedule [dimensionless].
PLACEHOLDER. Why: keep N' physically reasonable when driven by time-to-go (Role 3)."""

# --------------------------------------------------------------------------------
# KPI thresholds  (acceptance bar for Phases 3-4; 5% margin baked into targets)
# --------------------------------------------------------------------------------

R_MISS_MAX_M: float = 1.05
"""Max miss distance R_miss [m]. KPI: interception counts as a hit at <= 1.05 m."""

T_INT_STATIC_MAX_S: float = 10.0
"""Max time-to-intercept for a static target [s]. KPI."""

T_INT_MOVING_MAX_S: float = 20.0
"""Max time-to-intercept for a moving target [s]. KPI."""

Z_OVERSHOOT_MAX_M: float = 0.5
"""Max allowed Z-axis overshoot above the target [m]. KPI; the b-penalty exists to
keep this satisfied."""

CMD_SATURATION_MAX_FRAC: float = 0.05
"""Max fraction of flight time the command may be saturated [dimensionless]. KPI."""

MAX_TARGET_SPEED_MIN_KMH: float = 83.6
"""Minimum top target speed the interceptor must still defeat [km/h] (~90 km/h class
with margin). KPI."""

MISSION_SUCCESS_MIN: float = 0.90
"""Minimum mission success rate over randomized 3D trials [fraction]. KPI."""
