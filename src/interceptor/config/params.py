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
    """Extended Kalman Filter tuning (Role 2). Placeholders until Phase 2."""

    # Process-noise covariance diagonal [units vary per state element].
    process_noise_diag: tuple[float, ...] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    # Measurement-noise covariance diagonal [range m^2, angle rad^2 ...].
    measurement_noise_diag: tuple[float, ...] = (0.1, 0.01, 0.01)
    # Initial state covariance scale [dimensionless multiplier].
    initial_covariance_scale: float = 10.0


@dataclass(frozen=True)
class PidGains:
    """A single PID channel's gains [output-unit per error-unit]."""

    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0


@dataclass(frozen=True)
class ControlParams:
    """Dual-loop flight-control tuning (Role 4). Placeholders until Phase 2."""

    # Inner loop (~400 Hz) rate PIDs, one per body axis.
    inner_roll: PidGains = field(default_factory=PidGains)
    inner_pitch: PidGains = field(default_factory=PidGains)
    inner_yaw: PidGains = field(default_factory=PidGains)
    # Outer loop (~50 Hz) attitude-reference PIDs.
    outer_xy: PidGains = field(default_factory=PidGains)
    outer_z: PidGains = field(default_factory=PidGains)


@dataclass(frozen=True)
class GuidanceParams:
    """Guidance-law tuning (Role 3). Placeholders until Phase 2."""

    nav_ratio_min: float = constants.NAV_RATIO_MIN
    nav_ratio_max: float = constants.NAV_RATIO_MAX
    altitude_penalty_b: float = constants.ALTITUDE_PENALTY_B
    tilt_delay_time_constant_s: float = constants.TILT_DELAY_TIME_CONSTANT_S


@dataclass(frozen=True)
class LimiterParams:
    """Command-limiter bounds (Role 4, SAFETY). Placeholders until Phase 2."""

    # Max commandable linear acceleration magnitude [m/s^2].
    max_acceleration_m_s2: float = 30.0
    # Max commandable tilt angle [rad] (~35 deg default).
    max_tilt_rad: float = 0.6109


@dataclass(frozen=True)
class Params:
    """Top-level tunable-parameter bundle, snapshotted with every run."""

    ekf: EkfParams = field(default_factory=EkfParams)
    control: ControlParams = field(default_factory=ControlParams)
    guidance: GuidanceParams = field(default_factory=GuidanceParams)
    limiter: LimiterParams = field(default_factory=LimiterParams)

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
