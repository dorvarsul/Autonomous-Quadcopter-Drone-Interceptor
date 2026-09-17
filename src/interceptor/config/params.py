"""Runtime-tunable parameters with safe defaults, overridable from YAML.

Constants in :mod:`constants` are *physical/structural* and rarely change. The values
here are *tuning knobs* (EKF covariances, PID gains, navigation-ratio schedule,
limiter bounds) that Roles 2-4 sweep.

Design intent:
 - Defaults are the tuned baseline used across the scenario suite; a scenario YAML
   overrides only the knobs it needs.
 - Everything is a plain dataclass so it serializes cleanly into the per-run config
   snapshot (``common.logging``) for reproducibility.
 - :func:`load_params` merges a YAML override on top of the defaults, so a scenario
   file can change tuning without editing code.

Constraint (AGENTS.md → Workflow): changing a KPI-affecting tuning value in a way
that is committed as a new default requires user confirmation.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any

from interceptor.config import constants


@dataclass(frozen=True)
class EkfParams:
    """Extended Kalman Filter tuning (Role 2).

    The EKF tracks a 9-state relative target model ``[pos(3), vel(3), acc(3)]`` in the
    world frame with a constant-acceleration process (acceleration is a random walk).
    Defaults are conservative starting points; Role 2 tunes them against the
    sensor noise/latency profiles.
    """

    # Process-noise spectral densities per state group (constant-acceleration model with
    # an acceleration random walk). Larger => the filter adapts faster but trusts noisy
    # measurements more. Units chosen so ``q * dt`` has the state group's variance units.
    process_noise_position: float = 1.0e-3  # [m^2/s] on relative position
    process_noise_velocity: float = 1.0  # [m^2/s^3] on relative velocity
    process_noise_acceleration: float = 5.0  # [m^2/s^5] jerk PSD on relative acceleration
    # Measurement-noise variances; default to the sensor 1-sigma values squared
    # (range 0.30 m -> 0.09 m^2; angle 0.0035 rad -> ~1.2e-5 rad^2).
    measurement_noise_range_m2: float = 0.09
    measurement_noise_angle_rad2: float = 1.225e-5
    # Initial state covariance scale [dimensionless multiplier on the unit prior].
    initial_covariance_scale: float = 10.0
    # Innovation/covariance divergence guard: if the covariance trace exceeds this, the
    # filter has diverged and must fail loud rather than emit garbage (AGENTS.md).
    divergence_covariance_trace_max: float = 1.0e9


@dataclass(frozen=True)
class PidGains:
    """A single PID channel's gains [output-unit per error-unit]."""

    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0


@dataclass(frozen=True)
class ControlParams:
    """Dual-loop flight-control tuning (Role 4).

    Inner-loop gains are attitude-PD constants: desired angular acceleration =
    ``kp * attitude_error - kd * body_rate`` (torque = inertia * that). Roll/pitch use a
    critically-damped-ish response with natural frequency ~17 rad/s, well inside the
    400 Hz loop. **Yaw gains are deliberately ~100x smaller**: yaw torque is produced by
    rotor-drag differential (``kQ``), which is ~100x weaker than the arm-lever roll/pitch
    torque (``kT * arm``), so a large yaw gain would demand physically impossible rotor
    differentials and saturate all four motors. Yaw does not affect interception (the quad
    translates by tilting), so a gentle yaw hold is sufficient. The outer loop is
    algebraic (flatness) so its gains are unused placeholders for now.
    """

    # Inner loop (~400 Hz) attitude PDs, one per body axis (ki reserved for future use).
    inner_roll: PidGains = field(default_factory=lambda: PidGains(kp=300.0, ki=0.0, kd=30.0))
    inner_pitch: PidGains = field(default_factory=lambda: PidGains(kp=300.0, ki=0.0, kd=30.0))
    inner_yaw: PidGains = field(default_factory=lambda: PidGains(kp=2.0, ki=0.0, kd=0.5))
    # Angular-acceleration authority limit [rad/s^2] applied to the inner-loop PD output
    # before it becomes torque. Why: on a large-angle attitude step (a from-rest launch or a
    # hard reversal chasing a fast/evasive target) the raw PD command ``kp*error`` reaches
    # ~360 rad/s^2, far more roll/pitch torque than the rotors can allocate around the
    # collective thrust — so the mixer clamps a rotor to an RPM limit (actuator saturation).
    # The physical ceiling near hover thrust is ~73 rad/s^2: the differential rotor thrust
    # available while a rotor stays >= 0 (hover per-rotor thrust m*g/4 = 2.45 N, so |f_hi-f_lo|
    # <= 2*2.45 N) times the arm (0.15 m) over the roll/pitch inertia (0.01 kg*m^2) =
    # 0.15*4.9/0.01. Clamping the command to this envelope makes a large slew a proper
    # rate-limited turn instead of an infeasible torque spike the mixer would clamp (actuator
    # saturation). Small tracking errors stay far under the cap, so steady tracking (and OGL's
    # first-order tilt-lag model) is unchanged. Tuning (Role 4, user-approved): set to 70.0,
    # right at the hover authority — over a 60-trial randomized batch this lifts *honest*
    # command-saturation compliance from 40% to ~62% and nudges mission success 91.7%->93.3%
    # with no time-to-intercept regression; going lower (<=50) starts slowing intercepts (time
    # and mission both regress) for little further saturation gain. See the F4-1 fix notes.
    max_angular_accel_rad_s2: float = 70.0
    # Outer loop (~50 Hz) attitude-reference PIDs (unused; flatness map needs no gains yet).
    outer_xy: PidGains = field(default_factory=PidGains)
    outer_z: PidGains = field(default_factory=PidGains)


@dataclass(frozen=True)
class GuidanceParams:
    """OGL guidance tuning (Role 3)."""

    # Bounds clamped onto the lag-aware navigation-ratio schedule N'(t_go/T). Far from
    # intercept N' -> 3 (classic PN limit); near intercept the lag term drives it up, so
    # we cap it for numerical safety (Design Review — time-varying nav ratio).
    nav_ratio_min: float = constants.NAV_RATIO_MIN
    nav_ratio_max: float = constants.NAV_RATIO_MAX
    altitude_penalty_b: float = constants.ALTITUDE_PENALTY_B
    tilt_delay_time_constant_s: float = constants.TILT_DELAY_TIME_CONSTANT_S
    # Time-to-go conditioning: floor avoids the terminal 1/t_go^2 blow-up; cap keeps a
    # from-rest engagement finite. reference_closing_speed is used to synthesize a t_go
    # when the true closing speed is ~0 (static target / interceptor at rest) so OGL still
    # produces a closing command (ZEM trajectory-shaping).
    time_to_go_min_s: float = 0.05
    time_to_go_max_s: float = 30.0
    # Tuning (user-approved): 4.25 m/s. From rest OGL has no true closing speed, so it
    # synthesizes t_go from this reference to shape the launch command; a higher reference
    # means a smaller t_go and a more aggressive (faster) launch. Earlier this was held down
    # at 3.5 because an aggressive launch made the airframe overshoot in Z on same-altitude
    # lateral dashes — but that overshoot was a *thrust/tilt-lag coupling*, not a guidance
    # limit, and is now cancelled by the inner-loop thrust projection (see control/inner_loop
    # thrust-projection note). With the coupling gone, the binding constraint reverts to
    # command saturation, and 4.25 m/s keeps the worst-case baseline saturation ~2% (well
    # inside the 5% KPI, with margin) while cutting mean time-to-intercept on the slow
    # static/linear geometries by ~15% and holding Z-overshoot < 0.4 m. Pushing to 5.0 breaks
    # the 5% saturation budget; below 4.0 leaves speed on the table. See the tuning notes.
    reference_closing_speed_m_s: float = 4.25
    # Augmented-ZEM (target-acceleration feed-forward) switch. OFF by default: the EKF
    # estimates *relative* acceleration (a_target - a_interceptor), so against static/
    # linear targets it mostly reflects the interceptor's own maneuver — feeding that back
    # through the 0.5*a*t_go^2 term is positive feedback and destabilizes the loop. The
    # correct feed-forward needs the target's *absolute* acceleration (isolated using the
    # known interceptor acceleration); that compensation is future work for evasive
    # targets. Until then OGL uses the classic ZEM = r + v*t_go.
    use_target_acceleration: bool = False


@dataclass(frozen=True)
class LimiterParams:
    """Command-limiter bounds (Role 4, SAFETY)."""

    # Max commandable linear acceleration magnitude [m/s^2]. Tuning (user-approved,
    # params-only): raised 30 -> 40. The total-magnitude cap only binds on the most aggressive
    # climbing dashes (the tilt cap below governs the horizontal component first); 40 m/s^2
    # gives the fast-target/evasive engagements headroom before clamping while staying far
    # inside the airframe's ~250 m/s^2 collective-thrust capacity. NOTE: this bounds the
    # *linear* command, not the *attitude* command -- the motor mixer can still saturate on the
    # torque needed for an aggressive tilt slew even when this cap is not binding (it drives a
    # rotor below 0 RPM, not to max), which is why that saturation is now counted in the KPI
    # (orchestrator) and curbed at its source by the inner-loop angular-accel clamp. See F4-1.
    max_acceleration_m_s2: float = 40.0
    # Max commandable tilt angle [rad]. Tuning history: 0.6109 (35 deg) -> 0.7854 (45 deg) ->
    # 1.0472 (60 deg), then (user-approved) 1.0472 (60 deg) -> 1.2217 (70 deg). The horizontal
    # acceleration authority is g*tan(max_tilt); the fast crossing/quartering geometries pin
    # against it mid-course, the dominant command-saturation source. 60 deg gave g*tan60 =
    # 17.0 m/s^2; 70 deg raises it to g*tan70 = 27.0 m/s^2, roughly halving the *honest*
    # saturation on the hardest short high-speed crossers (e.g. varying_speed_crossing_84kmh
    # 40% -> 17%). 60 deg was previously the ceiling because 65 deg overshot easy static
    # targets -- but that overshoot was the thrust/tilt-lag coupling since cancelled by the
    # inner-loop thrust projection (see control/inner_loop), so with the coupling gone 70 deg
    # no longer regresses static Z-overshoot (verified: static_high/far/diagonal unchanged at
    # <= 0.013 m). The vertical thrust component cos(70) = 0.34 is still covered by the thrust
    # headroom. Paired with the inner-loop angular-accel clamp (ControlParams). See F4-1 notes.
    max_tilt_rad: float = 1.2217


@dataclass(frozen=True)
class SensorParams:
    """Sensor noise/latency profile (Role 1).

    These are *intentional* corruptions of the ground-truth geometry — the EKF exists
    to fight exactly this noise and delay (AGENTS.md → Role 1 must not sanitize signals
    for downstream convenience). A sensor model must be handed an explicit profile; the
    defaults below are a deliberate, documented baseline, not "no noise".
    """

    # Per-channel zero-mean Gaussian noise standard deviations.
    range_noise_std_m: float = 0.30  # range channel 1-sigma [m]
    azimuth_noise_std_rad: float = 0.0035  # ~0.2 deg LOS azimuth 1-sigma [rad]
    elevation_noise_std_rad: float = 0.0035  # ~0.2 deg LOS elevation 1-sigma [rad]
    # Constant per-channel biases (systematic offset the EKF cannot average away).
    range_bias_m: float = 0.0
    azimuth_bias_rad: float = 0.0
    elevation_bias_rad: float = 0.0
    # Quantization step per channel (0.0 disables quantization on that channel).
    range_quantization_m: float = 0.0
    angle_quantization_rad: float = 0.0
    # Finite sensor sample rate [Hz]; the sensor is slower than the sim.
    update_rate_hz: float = float(constants.ESTIMATION_HZ)
    # Measurement transport delay [s]; each sample is stamped with its age.
    latency_s: float = 0.02


@dataclass(frozen=True)
class WindParams:
    """Wind/gust disturbance profile (Role 1).

    Steady wind plus a seeded first-order Gauss-Markov (Ornstein-Uhlenbeck) gust
    process. The zero default is the *calm* preset and must reduce to undisturbed
    dynamics exactly.
    """

    # Constant mean wind velocity in the world frame [m/s].
    steady_velocity_m_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Gust 1-sigma magnitude per axis [m/s]; 0.0 => no gusts.
    gust_std_m_s: float = 0.0
    # Gust correlation (decorrelation) time [s]; larger => smoother, slower gusts.
    gust_correlation_time_s: float = 1.0


@dataclass(frozen=True)
class Params:
    """Top-level tunable-parameter bundle, snapshotted with every run."""

    ekf: EkfParams = field(default_factory=EkfParams)
    control: ControlParams = field(default_factory=ControlParams)
    guidance: GuidanceParams = field(default_factory=GuidanceParams)
    limiter: LimiterParams = field(default_factory=LimiterParams)
    sensor: SensorParams = field(default_factory=SensorParams)
    wind: WindParams = field(default_factory=WindParams)

    def to_dict(self) -> dict[str, Any]:
        """Plain-dict view for the reproducibility config snapshot."""
        return asdict(self)


def default_params() -> Params:
    """Return the safe default parameter set."""
    return Params()


def _merge_into(base: Any, override: dict[str, Any]) -> Any:
    """Recursively apply a (possibly partial) dict override onto a dataclass.

    Only keys present in the override are touched; unknown keys fail loud so a typo in
    a YAML scenario file cannot silently no-op a tuning change.
    """
    updates: dict[str, Any] = {}
    valid = {f for f in base.__dataclass_fields__}  # type: ignore[attr-defined]
    for key, value in override.items():
        if key not in valid:
            raise KeyError(
                f"Unknown parameter '{key}' for {type(base).__name__}; "
                f"valid keys: {sorted(valid)}"
            )
        current = getattr(base, key)
        if hasattr(current, "__dataclass_fields__") and isinstance(value, dict):
            updates[key] = _merge_into(current, value)
        else:
            updates[key] = value
    return replace(base, **updates)


def load_params(yaml_path: str | Path | None = None) -> Params:
    """Load tunable parameters, optionally overlaying a YAML override file.

    With no path, returns the defaults. With a path, deep-merges the YAML on top so a
    scenario file may override only the knobs it cares about.
    """
    params = default_params()
    if yaml_path is None:
        return params

    import yaml  # local import keeps PyYAML optional for pure-default callers

    path = Path(yaml_path)
    with path.open("r", encoding="utf-8") as handle:
        override = yaml.safe_load(handle) or {}
    if not isinstance(override, dict):
        raise ValueError(f"Parameter override in {path} must be a mapping.")
    return _merge_into(params, override)
