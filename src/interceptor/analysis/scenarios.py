"""Declarative, seeded scenario runner — Role 5 (Test/Validation/KPI), Phase 3 T3.2.

A *scenario* is a YAML file that fully specifies one interception trial: the RNG seed,
the target trajectory, the interceptor's initial state, the (optional) parameter overrides
(sensor/wind profile, guidance tuning), the active guidance law, and a time limit. Running
one produces a logged, reproducible run plus its measured KPIs.

Design intent:

- **Reuse, don't reinvent.** Trajectories come from the existing
  :mod:`interceptor.simulation.trajectories.generators` families; parameter overrides go
  through the same deep-merge (:func:`interceptor.config.params._merge_into`) that
  ``load_params`` uses; the run itself is the ordinary :class:`StubOrchestrator` closed
  loop wired by :meth:`PipelineComponents.build_intercept`. This module only *declares* and
  *drives* — no physics/estimation/guidance/control logic lives here (Role 5 boundary).
- **Reproducibility contract.** Every run persists its resolved params + seed + git hash
  (via the orchestrator snapshot) and, additionally, the scenario name and its resolved
  spec, so a result is fully traceable to the file that produced it (T3.2).
- **Fail loud on a bad spec.** Unknown trajectory types, missing keys, or a non-OGL
  guidance law raise immediately rather than silently defaulting (AGENTS.md).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from interceptor.analysis.kpis import KpiRecord, kpis_from_log
from interceptor.common.rng import RngFactory
from interceptor.config import constants
from interceptor.config.params import Params, _merge_into, default_params
from interceptor.pipeline.orchestrator import (
    PipelineComponents,
    RunResult,
    StubOrchestrator,
)
from interceptor.simulation.interfaces import TargetTrajectory
from interceptor.simulation.trajectories.generators import (
    LinearTrajectory,
    SinusoidalTrajectory,
    StaticTrajectory,
    VaryingSpeedTrajectory,
)

# The sole guidance law (OGL-only scope). A scenario may name it explicitly; anything else
# is a defect, not a silent fallback.
_GUIDANCE_LAW = "OGL"

# Target classes and the time-to-intercept KPI bound each selects (Design Review §7).
_T_INT_MAX_BY_CLASS = {
    "static": constants.T_INT_STATIC_MAX_S,
    "moving": constants.T_INT_MOVING_MAX_S,
}

# Trajectory-type keys that are inherently *moving* (used to default ``target_class``).
_MOVING_TRAJECTORY_TYPES = frozenset({"linear", "sinusoidal", "varying_speed"})


def _require(spec: dict[str, Any], key: str, context: str) -> Any:
    """Fetch a required key or fail loud naming the offending scenario section."""
    if key not in spec:
        raise KeyError(f"Scenario {context} is missing required key '{key}'.")
    return spec[key]


def _vec3(value: Any, name: str) -> np.ndarray:
    """Coerce a YAML sequence into a 3-vector [m] / [m/s], failing loud on the wrong shape."""
    arr = np.asarray(value, dtype=np.float64)
    if arr.shape != (3,):
        raise ValueError(f"'{name}' must be a 3-element vector; got shape {arr.shape}.")
    return arr


def build_trajectory(spec: dict[str, Any]) -> TargetTrajectory:
    """Map a scenario ``target:`` block to a concrete trajectory generator (reuse only).

    Supported ``type`` values mirror the Phase 1 generator families. Static and linear are
    the Phase 3 focus; sinusoidal and varying-speed are wired here too so the Phase 4
    evasive/high-speed suites need no new plumbing (Open/Closed).
    """
    kind = _require(spec, "type", "target")
    if kind == "static":
        return StaticTrajectory(_vec3(_require(spec, "position_m", "target(static)"), "position_m"))
    if kind == "linear":
        return LinearTrajectory(
            _vec3(_require(spec, "start_position_m", "target(linear)"), "start_position_m"),
            _vec3(_require(spec, "velocity_m_s", "target(linear)"), "velocity_m_s"),
        )
    if kind == "sinusoidal":
        return SinusoidalTrajectory(
            _vec3(_require(spec, "start_position_m", "target(sinusoidal)"), "start_position_m"),
            _vec3(_require(spec, "drift_velocity_m_s", "target(sinusoidal)"), "drift_velocity_m_s"),
            _vec3(_require(spec, "amplitude_m", "target(sinusoidal)"), "amplitude_m"),
            float(_require(spec, "frequency_hz", "target(sinusoidal)")),
        )
    if kind == "varying_speed":
        return VaryingSpeedTrajectory(
            _vec3(_require(spec, "start_position_m", "target(varying_speed)"), "start_position_m"),
            _vec3(_require(spec, "heading", "target(varying_speed)"), "heading"),
            float(_require(spec, "initial_speed_m_s", "target(varying_speed)")),
            float(_require(spec, "peak_speed_m_s", "target(varying_speed)")),
            float(_require(spec, "ramp_duration_s", "target(varying_speed)")),
        )
    raise ValueError(
        f"Unknown target trajectory type '{kind}'. "
        f"Expected one of: static, linear, sinusoidal, varying_speed."
    )


@dataclass(frozen=True)
class Scenario:
    """A fully-resolved interception trial parsed from a scenario YAML file."""

    name: str
    seed: int
    interceptor_start_m: np.ndarray
    target_spec: dict[str, Any]
    target_class: str  # "static" | "moving" -> selects the time-to-intercept KPI bound
    time_limit_s: float
    params: Params
    raw_spec: dict[str, Any] = field(default_factory=dict)

    @property
    def time_to_intercept_max_s(self) -> float:
        """The time-to-intercept KPI bound for this scenario's target class [s]."""
        return _T_INT_MAX_BY_CLASS[self.target_class]

    def build_trajectory(self) -> TargetTrajectory:
        return build_trajectory(self.target_spec)


def scenario_from_dict(spec: dict[str, Any]) -> Scenario:
    """Parse and validate a scenario spec dict (fail loud on anything malformed)."""
    if not isinstance(spec, dict):
        raise ValueError("A scenario must be a YAML mapping.")

    law = spec.get("guidance_law", _GUIDANCE_LAW)
    if law != _GUIDANCE_LAW:
        raise ValueError(
            f"Scenario guidance_law must be '{_GUIDANCE_LAW}' (OGL is the sole guidance "
            f"law); got '{law}'."
        )

    target_spec = _require(spec, "target", "root")
    if not isinstance(target_spec, dict):
        raise ValueError("Scenario 'target' must be a mapping.")

    # Default the target class from the trajectory type unless explicitly declared.
    default_class = "moving" if target_spec.get("type") in _MOVING_TRAJECTORY_TYPES else "static"
    target_class = spec.get("target_class", default_class)
    if target_class not in _T_INT_MAX_BY_CLASS:
        raise ValueError(
            f"Scenario target_class must be one of {sorted(_T_INT_MAX_BY_CLASS)}; "
            f"got '{target_class}'."
        )

    interceptor = _require(spec, "interceptor", "root")
    start_m = _vec3(_require(interceptor, "start_m", "interceptor"), "interceptor.start_m")

    # Deep-merge the optional params override onto the defaults (same path as load_params).
    override = spec.get("params", {}) or {}
    if not isinstance(override, dict):
        raise ValueError("Scenario 'params' override must be a mapping.")
    params = _merge_into(default_params(), override)

    return Scenario(
        name=str(_require(spec, "name", "root")),
        seed=int(spec.get("seed", 0)),
        interceptor_start_m=start_m,
        target_spec=target_spec,
        target_class=target_class,
        time_limit_s=float(_require(spec, "time_limit_s", "root")),
        params=params,
        raw_spec=spec,
    )


def load_scenario(path: str | Path) -> Scenario:
    """Load and validate a single scenario YAML file."""
    import yaml  # local import keeps PyYAML optional for pure-default callers

    path = Path(path)
    with path.open("r", encoding="utf-8") as handle:
        spec = yaml.safe_load(handle)
    if not isinstance(spec, dict):
        raise ValueError(f"Scenario file {path} must contain a single YAML mapping.")
    spec.setdefault("name", path.stem)
    return scenario_from_dict(spec)


def discover_scenarios(path: str | Path) -> list[Path]:
    """Return the scenario YAML files at ``path`` (a single file or a directory).

    Directory results are sorted for a deterministic batch order.
    """
    path = Path(path)
    if path.is_dir():
        return sorted(p for p in path.glob("*.yaml"))
    return [path]


@dataclass(frozen=True)
class ScenarioResult:
    """One scenario's run outcome and measured KPIs."""

    scenario: Scenario
    run: RunResult
    kpis: KpiRecord


def run_scenario(scenario: Scenario, results_dir: str | Path) -> ScenarioResult:
    """Run one scenario headlessly and measure its KPIs.

    The engagement terminates at closest approach (so the saturation KPI is measured over
    the real engagement, not a flyby), and the run snapshot records the scenario name +
    resolved spec on top of the params/seed/git-hash the orchestrator already persists.
    """
    rng = RngFactory(scenario.seed)
    components = PipelineComponents.build_intercept(
        rng,
        scenario.params,
        trajectory=scenario.build_trajectory(),
        interceptor_position_m=scenario.interceptor_start_m,
    )
    run_dir = Path(results_dir) / scenario.name
    orchestrator = StubOrchestrator(
        components=components, params=scenario.params, seed=scenario.seed
    )
    result = orchestrator.run(
        num_steps=int(scenario.time_limit_s * constants.SIM_HZ),
        run_dir=run_dir,
        run_id=scenario.name,
        terminate_on_intercept=True,
        extra_metadata={
            "scenario": scenario.name,
            "target_class": scenario.target_class,
            "scenario_spec": scenario.raw_spec,
        },
    )
    kpis = kpis_from_log(
        result.log_path, time_to_intercept_max_s=scenario.time_to_intercept_max_s
    )
    return ScenarioResult(scenario=scenario, run=result, kpis=kpis)


def run_suite(path: str | Path, results_dir: str | Path) -> list[ScenarioResult]:
    """Run every scenario at ``path`` (file or directory) headlessly, in deterministic order."""
    return [run_scenario(load_scenario(p), results_dir) for p in discover_scenarios(path)]
