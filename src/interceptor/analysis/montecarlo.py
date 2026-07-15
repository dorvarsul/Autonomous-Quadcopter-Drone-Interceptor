"""Randomized 3D Monte-Carlo trial harness — Role 5.

The hand-written ``scenarios/`` library probes specific *named* geometries; this harness
instead samples the **whole threat envelope** — random 3D engagement geometry, a random
target-trajectory family (static / linear / sinusoidal / varying-speed) with random
parameters, and a random wind preset — and flies a seeded batch of them to measure the
**Mission Success Rate** KPI (``>= MISSION_SUCCESS_MIN``, Design Review §7).

Design intent (all in service of AGENTS.md → determinism & Role-5 boundary):

- **Reuse, don't reinvent.** Every sampled trial is turned into an ordinary validated
  :class:`~interceptor.analysis.scenarios.Scenario` via ``scenario_from_dict`` and flown by
  the same ``run_scenario`` closed loop the named scenario suite uses. This module only *samples*
  and *aggregates*; it contains no physics/estimation/guidance/control logic.
- **Reproducible batch.** One ``master_seed`` seeds a single sampling RNG, and each trial's
  run seed is its index, so a given ``(master_seed, num_trials)`` reproduces the whole batch
  — geometry, noise, and all — byte-for-byte. The batch manifest records the master seed +
  git hash so the dataset is traceable (reproducibility contract).
- **Fair, documented envelope.** The sampling ranges below are a deliberate, named threat
  distribution (a realistic mix with a hard evasive/high-speed tail), not a set cherry-picked
  to inflate the pass rate. The per-family breakdown in :class:`BatchSummary` exposes weak
  regimes rather than hiding them.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from interceptor.analysis.kpis import KpiRecord
from interceptor.analysis.scenarios import (
    Scenario,
    ScenarioResult,
    run_scenario,
    scenario_from_dict,
)

# --------------------------------------------------------------------------------
# Sampling envelope (named — no magic numbers). A realistic 3D threat distribution.
# --------------------------------------------------------------------------------

# Target-trajectory families and their sampling weights. Static and constant-velocity
# threats dominate a realistic mix; the evasive/high-speed families form the hard tail that
# stresses the estimator/guidance/control stack (Design Review §7 scenario spread).
_FAMILY_WEIGHTS: dict[str, float] = {
    "static": 0.20,
    "linear": 0.30,
    "sinusoidal": 0.30,
    "varying_speed": 0.20,
}

# Wind presets and weights: most engagements are calm; a minority face moderate/gusty
# disturbance so the batch also samples control-loop robustness (Role 4).
_WIND_WEIGHTS: dict[str, float] = {"calm": 0.60, "moderate": 0.25, "gusty": 0.15}

# Engagement geometry: the target is placed in a frontal cone at a random slant range.
_RANGE_MIN_M, _RANGE_MAX_M = 8.0, 14.0
_AZIMUTH_MAX_RAD = np.deg2rad(50.0)  # +/- off the interceptor's forward (+X) axis
_ELEVATION_MIN_RAD, _ELEVATION_MAX_RAD = np.deg2rad(8.0), np.deg2rad(45.0)
_INTERCEPTOR_ALT_MIN_M, _INTERCEPTOR_ALT_MAX_M = 1.5, 2.5

# Linear (constant-velocity) target: horizontal speed and a gentle vertical rate.
_LINEAR_SPEED_MIN_MS, _LINEAR_SPEED_MAX_MS = 2.0, 5.0
# Vertical velocity is kept small so a fair aerial threat stays airborne (a target diving
# through the ground plane is not a valid intercept trial — Role-5 fair-envelope design).
_LINEAR_VERTICAL_MIN_MS, _LINEAR_VERTICAL_MAX_MS = -0.3, 0.5
# Blend a random heading with the closing direction so targets move *through* the
# engagement zone rather than fleeing straight away from a from-rest interceptor.
_CLOSING_BLEND = 0.5

# Sinusoidal (evasive) target: lateral/vertical weave amplitude, frequency, drift.
_SINE_AMPLITUDE_MIN_M, _SINE_AMPLITUDE_MAX_M = 0.5, 1.5
_SINE_FREQUENCY_MIN_HZ, _SINE_FREQUENCY_MAX_HZ = 0.15, 0.40
_SINE_DRIFT_MAX_MS = 1.5
_SINE_VERTICAL_DRIFT_MAX_MS = 0.4  # keep the weave centre aloft (fair-envelope)

# Varying-speed target: an approaching threat ramping to a random peak up to the 90 km/h
# class (25 m/s). Heading points back toward the interceptor region (a closing engagement)
# plus a lateral perturbation, so the from-rest interceptor can realistically meet it.
_VSPEED_INITIAL_MS = 8.0
_VSPEED_PEAK_MIN_MS, _VSPEED_PEAK_MAX_MS = 10.0, 25.0
_VSPEED_RAMP_S = 3.0
_VSPEED_LATERAL_PERTURB = 0.35  # fraction of the closing direction added as random lateral

# Time budget per family [s]; the moving budget stays under the 20 s moving-KPI bound.
_TIME_LIMIT_STATIC_S = 12.0
_TIME_LIMIT_MOVING_S = 18.0


def _unit_from_az_el(azimuth_rad: float, elevation_rad: float) -> np.ndarray:
    """A unit direction from spherical angles about the forward (+X) axis, Z up."""
    ce = np.cos(elevation_rad)
    return np.array(
        [ce * np.cos(azimuth_rad), ce * np.sin(azimuth_rad), np.sin(elevation_rad)],
        dtype=np.float64,
    )


def _closing_biased_horizontal(
    rng: np.random.Generator, position: np.ndarray, interceptor_start: np.ndarray
) -> np.ndarray:
    """A unit horizontal heading blending a random direction with the closing direction.

    Pure-random headings let a target flee straight away from the from-rest interceptor
    (an unwinnable, unfair trial); pure-closing headings are trivially head-on. Blending the
    two (``_CLOSING_BLEND``) yields fair crossing/quartering threats that pass through the
    engagement zone. Horizontal only — the vertical rate is sampled separately and kept small.
    """
    random_dir = rng.normal(size=2)
    random_dir /= np.linalg.norm(random_dir) or 1.0
    closing = (interceptor_start - position)[:2]
    closing /= np.linalg.norm(closing) or 1.0
    blended = _CLOSING_BLEND * random_dir + (1.0 - _CLOSING_BLEND) * closing
    return blended / (np.linalg.norm(blended) or 1.0)


def _sample_target_position(rng: np.random.Generator, interceptor_start: np.ndarray) -> np.ndarray:
    """Place the target in a frontal cone at a random slant range from the interceptor."""
    slant = rng.uniform(_RANGE_MIN_M, _RANGE_MAX_M)
    azimuth = rng.uniform(-_AZIMUTH_MAX_RAD, _AZIMUTH_MAX_RAD)
    elevation = rng.uniform(_ELEVATION_MIN_RAD, _ELEVATION_MAX_RAD)
    return interceptor_start + slant * _unit_from_az_el(azimuth, elevation)


def _sample_target_spec(
    rng: np.random.Generator, family: str, interceptor_start: np.ndarray
) -> tuple[dict[str, Any], float]:
    """Sample one target ``target:`` block for ``family`` plus its time budget [s]."""
    position = _sample_target_position(rng, interceptor_start)

    if family == "static":
        return {"type": "static", "position_m": position.tolist()}, _TIME_LIMIT_STATIC_S

    if family == "linear":
        # Horizontal velocity: a random heading blended toward the closing direction so the
        # target crosses/approaches the engagement zone (fair, catchable), plus a gentle
        # vertical rate that keeps it airborne over the run.
        horizontal = _closing_biased_horizontal(rng, position, interceptor_start)
        speed = rng.uniform(_LINEAR_SPEED_MIN_MS, _LINEAR_SPEED_MAX_MS)
        vz = rng.uniform(_LINEAR_VERTICAL_MIN_MS, _LINEAR_VERTICAL_MAX_MS)
        velocity = np.array([horizontal[0] * speed, horizontal[1] * speed, vz])
        return {
            "type": "linear",
            "start_position_m": position.tolist(),
            "velocity_m_s": velocity.tolist(),
        }, _TIME_LIMIT_MOVING_S

    if family == "sinusoidal":
        # Weave about a gently-drifting centre that stays in the engagement zone (fair):
        # horizontal drift is closing-biased and small, vertical drift is capped so the
        # centre stays aloft. The oscillation amplitude/frequency provide the evasion.
        horizontal = _closing_biased_horizontal(rng, position, interceptor_start)
        drift_speed = rng.uniform(0.0, _SINE_DRIFT_MAX_MS)
        drift = np.array([
            horizontal[0] * drift_speed,
            horizontal[1] * drift_speed,
            rng.uniform(-_SINE_VERTICAL_DRIFT_MAX_MS, _SINE_VERTICAL_DRIFT_MAX_MS),
        ])
        amplitude = rng.uniform(_SINE_AMPLITUDE_MIN_M, _SINE_AMPLITUDE_MAX_M, size=3)
        frequency = rng.uniform(_SINE_FREQUENCY_MIN_HZ, _SINE_FREQUENCY_MAX_HZ)
        return {
            "type": "sinusoidal",
            "start_position_m": position.tolist(),
            "drift_velocity_m_s": drift.tolist(),
            "amplitude_m": amplitude.tolist(),
            "frequency_hz": float(frequency),
        }, _TIME_LIMIT_MOVING_S

    if family == "varying_speed":
        # Heading points from the target back toward the interceptor region (closing) with a
        # random lateral perturbation, so a fast target still enters the engagement envelope.
        closing = interceptor_start - position
        closing /= np.linalg.norm(closing)
        lateral = rng.normal(size=3) * _VSPEED_LATERAL_PERTURB
        heading = closing + lateral
        peak = rng.uniform(_VSPEED_PEAK_MIN_MS, _VSPEED_PEAK_MAX_MS)
        return {
            "type": "varying_speed",
            "start_position_m": position.tolist(),
            "heading": heading.tolist(),
            "initial_speed_m_s": _VSPEED_INITIAL_MS,
            "peak_speed_m_s": float(peak),
            "ramp_duration_s": _VSPEED_RAMP_S,
        }, _TIME_LIMIT_MOVING_S

    raise ValueError(f"Unknown target family '{family}'.")


def _weighted_choice(rng: np.random.Generator, weights: dict[str, float]) -> str:
    """Pick a key from a name->weight mapping using a seeded generator."""
    names = list(weights)
    probabilities = np.asarray([weights[n] for n in names], dtype=np.float64)
    probabilities /= probabilities.sum()
    return str(rng.choice(names, p=probabilities))


def sample_trial(index: int, rng: np.random.Generator) -> Scenario:
    """Sample and validate one randomized trial as a :class:`Scenario`.

    ``index`` becomes the trial name and its run seed, so the trial's sensor/wind noise is
    reproducible and independent of the geometry-sampling RNG (which is driven by the batch
    ``master_seed``). The sampled spec is routed through ``scenario_from_dict`` so a trial is
    validated exactly like a hand-written scenario file (single source of truth).
    """
    interceptor_start = np.array(
        [0.0, 0.0, rng.uniform(_INTERCEPTOR_ALT_MIN_M, _INTERCEPTOR_ALT_MAX_M)],
        dtype=np.float64,
    )
    family = _weighted_choice(rng, _FAMILY_WEIGHTS)
    wind_preset = _weighted_choice(rng, _WIND_WEIGHTS)
    target_spec, time_limit_s = _sample_target_spec(rng, family, interceptor_start)

    spec: dict[str, Any] = {
        "name": f"trial_{index:04d}",
        "seed": index,
        "guidance_law": "OGL",
        "time_limit_s": time_limit_s,
        "wind_preset": wind_preset,
        "interceptor": {"start_m": interceptor_start.tolist()},
        "target": target_spec,
    }
    return scenario_from_dict(spec)


@dataclass(frozen=True)
class TrialResult:
    """One Monte-Carlo trial: what was flown, its family/wind, and the measured KPIs."""

    scenario: Scenario
    kpis: KpiRecord
    family: str
    wind_preset: str

    @property
    def name(self) -> str:
        return self.scenario.name


def _family_of(scenario: Scenario) -> str:
    return str(scenario.target_spec["type"])


def _wind_of(scenario: Scenario) -> str:
    """Recover the wind label from the resolved params (calm when undisturbed)."""
    wind = scenario.params.wind
    if wind.gust_std_m_s == 0.0 and not np.any(np.asarray(wind.steady_velocity_m_s)):
        return "calm"
    return "gusty" if wind.gust_std_m_s >= 3.0 else "moderate"


# The per-trial KPI checks tracked for aggregate compliance, in report order.
_KPI_CHECKS: tuple[str, ...] = ("miss", "time", "z_overshoot", "saturation")


def _kpi_ok(kpis, name: str) -> bool:
    return {
        "miss": kpis.miss_ok,
        "time": kpis.time_ok,
        "z_overshoot": kpis.z_overshoot_ok,
        "saturation": kpis.saturation_ok,
    }[name]


@dataclass(frozen=True)
class BatchSummary:
    """Aggregate outcome of a Monte-Carlo batch — the Mission Success Rate KPI and breakdown.

    **Mission success = interception** (``R_miss <= R_MISS_MAX_M``), matching the Design
    Review's *"Mission Success Rate: >= 90% interception over randomized 3D trials."* The
    other KPIs (time, Z-overshoot, saturation) are separate acceptance criteria reported as
    aggregate compliance rates rather than folded into the per-trial success flag — so a very
    short high-speed intercept that clears the miss KPI but transiently exceeds 5% saturation
    is a *mission success* with a filed saturation finding, not a mission failure.
    """

    master_seed: int
    num_trials: int
    results: list[TrialResult]

    @property
    def num_intercepted(self) -> int:
        """Trials that came within the miss-distance KPI (the interception count)."""
        return sum(1 for r in self.results if r.kpis.miss_ok)

    @property
    def mission_success_rate(self) -> float:
        """Interception fraction — the Design Review Mission Success Rate KPI."""
        return self.num_intercepted / self.num_trials if self.num_trials else 0.0

    @property
    def num_full_kpi_pass(self) -> int:
        """Trials meeting *every* KPI (interception + time + Z-overshoot + saturation)."""
        return sum(1 for r in self.results if r.kpis.success)

    @property
    def kpi_compliance(self) -> dict[str, tuple[int, int]]:
        """``kpi -> (met, trials)`` aggregate compliance for each individual KPI."""
        return {
            name: (sum(1 for r in self.results if _kpi_ok(r.kpis, name)), self.num_trials)
            for name in _KPI_CHECKS
        }

    @property
    def max_intercepted_speed_kmh(self) -> float:
        """Fastest target the batch actually intercepted (miss within KPI) [km/h].

        Certifies the ``MAX_TARGET_SPEED_MIN_KMH`` requirement over the randomized batch.
        """
        speeds = [r.kpis.max_target_speed_kmh for r in self.results if r.kpis.miss_ok]
        return max(speeds) if speeds else 0.0

    def _breakdown(self, key) -> dict[str, tuple[int, int]]:
        """``bucket -> (interceptions, trials)`` grouped by ``key`` (mission-success basis)."""
        counts: dict[str, list[int]] = defaultdict(lambda: [0, 0])
        for r in self.results:
            bucket = counts[key(r)]
            bucket[0] += int(r.kpis.miss_ok)
            bucket[1] += 1
        return {k: (v[0], v[1]) for k, v in sorted(counts.items())}

    @property
    def by_family(self) -> dict[str, tuple[int, int]]:
        """``family -> (interceptions, trials)``, exposing weak trajectory regimes."""
        return self._breakdown(lambda r: r.family)

    @property
    def by_wind(self) -> dict[str, tuple[int, int]]:
        """``wind_preset -> (interceptions, trials)``, exposing disturbance sensitivity."""
        return self._breakdown(lambda r: r.wind_preset)

    def failures(self) -> list[TrialResult]:
        """Trials that failed to intercept, for failure-mode triage."""
        return [r for r in self.results if not r.kpis.miss_ok]

    def kpi_exceedances(self) -> list[TrialResult]:
        """Intercepting trials that still breached a non-miss KPI (saturation/Z/time)."""
        return [r for r in self.results if r.kpis.miss_ok and not r.kpis.success]


def run_montecarlo(
    num_trials: int, master_seed: int, results_dir: str | Path
) -> BatchSummary:
    """Fly a seeded randomized batch and aggregate the Mission Success Rate.

    Each trial is flown by the ordinary ``run_scenario`` loop (which persists its own
    config+seed+git-hash+spec snapshot), so the batch is fully reproducible from
    ``(master_seed, num_trials)`` alone. Headless, deterministic, and single-threaded for a
    stable ordering; a caller wanting parallelism can shard by trial index.
    """
    if num_trials <= 0:
        raise ValueError("num_trials must be > 0.")
    rng = np.random.default_rng(np.random.SeedSequence(int(master_seed)))
    results_dir = Path(results_dir)

    results: list[TrialResult] = []
    for index in range(num_trials):
        scenario = sample_trial(index, rng)
        outcome: ScenarioResult = run_scenario(scenario, results_dir)
        results.append(
            TrialResult(
                scenario=scenario,
                kpis=outcome.kpis,
                family=_family_of(scenario),
                wind_preset=_wind_of(scenario),
            )
        )

    return BatchSummary(
        master_seed=int(master_seed),
        num_trials=num_trials,
        results=results,
    )
