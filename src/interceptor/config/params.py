"""Runtime-tunable parameters with safe defaults, overridable from YAML.

Constants in :mod:`constants` are *physical/structural* and rarely change. The values
here are *tuning knobs* (EKF covariances, PID gains, navigation-ratio schedule,
limiter bounds) that Roles 2-4 sweep during Phases 2-3.

Design intent:
 - Defaults are deliberately conservative placeholders; Phase 0 wires no algorithm,
   so nothing here is tuned yet.
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
    Defaults are conservative starting points; Role 2 tunes them in Phase 3 against the
    Phase 1 noise/latency profiles.
    """

    # Process-noise spectral densities per state group (constant-acceleration model with
    # an acceleration random walk). Larger => the filter adapts faster but trusts noisy
    # measurements more. Units chosen so ``q * dt`` has the state group's variance units.
    process_noise_position: float = 1.0e-3  # [m^2/s] on relative position
    process_noise_velocity: float = 1.0  # [m^2/s^3] on relative velocity
    process_noise_acceleration: float = 5.0  # [m^2/s^5] jerk PSD on relative acceleration
    # Measurement-noise variances; default to the Phase 1 sensor 1-sigma values squared
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
    translates by tilting), so a gentle yaw hold is sufficient. Retuned in Phase 3; the
    outer loop is algebraic (flatness) so its gains are unused placeholders for now.
    """

    # Inner loop (~400 Hz) attitude PDs, one per body axis (ki reserved for Phase 3).
    inner_roll: PidGains = field(default_factory=lambda: PidGains(kp=300.0, ki=0.0, kd=30.0))
    inner_pitch: PidGains = field(default_factory=lambda: PidGains(kp=300.0, ki=0.0, kd=30.0))
    inner_yaw: PidGains = field(default_factory=lambda: PidGains(kp=2.0, ki=0.0, kd=0.5))
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
    # Phase 3 tuning (T3.6, user-approved): lowered 5.0 -> 3.5 m/s. From rest OGL has no true
    # closing speed, so it synthesizes t_go from this reference to shape the launch command.
    # At 5 m/s the from-rest command over-drove the tilt bound and saturated the first ~20%
    # of frames (the dominant term in the >5% command-saturation KPI miss). 3.5 m/s softens
    # the launch enough to bring saturation within 5% across the static/linear suite (paired
    # with the 45 deg tilt bound below) while still closing the farthest static target inside
    # the < 10 s KPI; going as low as 2.5 cut saturation further but slowed the 12.4 m target
    # past 10 s, and 1.5 made some geometries miss. See docs/phase3_progress.md (T3.6).
    reference_closing_speed_m_s: float = 3.5
    # Augmented-ZEM (target-acceleration feed-forward) switch. OFF for Phase 2: the EKF
    # estimates *relative* acceleration (a_target - a_interceptor), so against static/
    # linear targets it mostly reflects the interceptor's own maneuver — feeding that back
    # through the 0.5*a*t_go^2 term is positive feedback and destabilizes the loop. The
    # correct feed-forward needs the target's *absolute* acceleration (isolated using the
    # known interceptor acceleration); that compensation is Phase 4 work for evasive
    # targets. Until then OGL uses the classic ZEM = r + v*t_go.
    use_target_acceleration: bool = False


@dataclass(frozen=True)
class LimiterParams:
    """Command-limiter bounds (Role 4, SAFETY)."""

    # Max commandable linear acceleration magnitude [m/s^2].
    max_acceleration_m_s2: float = 30.0
    # Max commandable tilt angle [rad]. Phase 3 tuning (T3.7, user-approved): raised 0.6109
    # (~35 deg) -> 0.7854 (45 deg). The horizontal acceleration authority is g*tan(max_tilt),
    # so 35 deg capped it at ~6.87 m/s^2 and the launch/cross-range dashes clamped against it
    # (command-saturation KPI). 45 deg raises the authority to g = 9.81 m/s^2, keeping the
    # aggressive geometries within the <= 5% saturation KPI; 45 deg remains conservative for a
    # quadrotor. Paired with the softened reference_closing_speed above (see phase3_progress).
    max_tilt_rad: float = 0.7854


@dataclass(frozen=True)
class SensorParams:
    """Sensor noise/latency profile (Role 1, Phase 1).

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
    """Wind/gust disturbance profile (Role 1, Phase 1).

    Steady wind plus a seeded first-order Gauss-Markov (Ornstein-Uhlenbeck) gust
    process. The zero default is the *calm* preset and must reduce to undisturbed
    dynamics exactly (T1.6 DoD).
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
