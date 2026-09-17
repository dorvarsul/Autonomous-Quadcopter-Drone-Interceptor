"""KPI measurement from a run log — Role 5 (Test/Validation/KPI).

This module is the **single source of truth** for turning a raw ``run_log.csv`` into the
Design Review's success metrics. Nothing here re-implements guidance/control/estimation;
it only *measures* an already-recorded engagement. All acceptance thresholds are pulled
from :mod:`interceptor.config.constants` (no inline KPI numbers — Clean Code / DRY).

The KPIs (Design Review §7; AGENTS.md → Role 5 table):

- **Miss distance** ``R_miss`` — the minimum interceptor↔target range over the run [m].
- **Time-to-intercept** — the first time the range falls to the miss threshold [s]; the
  static and moving cases have different targets, so the applicable bound is supplied by
  the caller (the scenario declares its class).
- **Z-axis overshoot** — the largest amount the interceptor rises *above* the target
  altitude during the approach [m] (the b-penalty exists to keep this small).
- **Command saturation** — the fraction of logged frames flagged ``saturated``. That flag
  is the *honest actuator-chain* flag written by the orchestrator: a frame is saturated if
  the command limiter clamped the acceleration request **or** the motor mixer clamped a
  rotor to an RPM limit. Counting only the limiter would hide mixer saturation on aggressive
  attitude slews (AGENTS.md → saturation must stay measurable, not hidden). Because the
  scenario runner terminates the engagement at closest approach, this is measured over the
  *real engagement*, not a post-intercept flyby.
- **Max target speed handled** — the peak target speed over the run [km/h], from a finite
  difference of the logged target positions (characterization / stress metric).

The saturation fraction is measured over the terminated engagement because the scenario
runner stops the run at closest approach; feeding this module a *non*-terminated log would
dilute the fraction with meaningless flyby frames (see ``run_intercept.py`` note).
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from interceptor.common import frames
from interceptor.config import constants


@dataclass(frozen=True)
class RunTrace:
    """The columns of a ``run_log.csv`` needed for KPI measurement.

    A thin, typed view over the CSV so KPI functions never re-parse rows or assume a
    column layout. Positions are ``(N, 3)`` world-frame arrays [m]; ``saturated`` is the
    per-frame combined actuator-chain flag (limiter OR mixer).
    """

    time_s: NDArray[np.float64]
    interceptor_pos_m: NDArray[np.float64]
    target_pos_m: NDArray[np.float64]
    saturated: NDArray[np.bool_]

    @property
    def range_m(self) -> NDArray[np.float64]:
        """Per-frame interceptor↔target range [m]."""
        return np.linalg.norm(self.target_pos_m - self.interceptor_pos_m, axis=1)


@dataclass(frozen=True)
class KpiRecord:
    """Measured KPIs for one run plus their pass/fail verdicts.

    Verdicts compare each measured value against the constant already carrying the 5%
    acceptance margin, so ``success`` is simply the conjunction of the individual flags.
    """

    miss_distance_m: float
    time_to_intercept_s: float
    z_overshoot_m: float
    command_saturation_frac: float
    max_target_speed_kmh: float
    time_to_intercept_max_s: float
    # Per-KPI verdicts (each already includes the Design Review's 5% margin via constants).
    miss_ok: bool
    time_ok: bool
    z_overshoot_ok: bool
    saturation_ok: bool

    @property
    def intercepted(self) -> bool:
        """True when the interceptor came within the miss-distance KPI at all."""
        return np.isfinite(self.time_to_intercept_s)

    @property
    def success(self) -> bool:
        """Overall per-trial success: every measured KPI is within its target."""
        return self.miss_ok and self.time_ok and self.z_overshoot_ok and self.saturation_ok


def load_run_trace(csv_path: str | Path) -> RunTrace:
    """Load the KPI-relevant columns of a run log into a :class:`RunTrace`.

    Fails loud on an empty log — a run that produced no frames is a defect, not a
    zero-KPI success (AGENTS.md → fail loud, not silent).
    """
    times: list[float] = []
    interceptor: list[list[float]] = []
    target: list[list[float]] = []
    saturated: list[bool] = []
    with Path(csv_path).open(newline="") as handle:
        for row in csv.DictReader(handle):
            times.append(float(row["sim_time_s"]))
            interceptor.append([float(row[f"interceptor_{a}_m"]) for a in "xyz"])
            target.append([float(row[f"target_{a}_m"]) for a in "xyz"])
            # RunLogger serializes booleans as "1"/"0" (see common.logging._format_value).
            saturated.append(row["saturated"] not in ("0", "False", ""))
    if not times:
        raise ValueError(f"Run log {csv_path} has no data rows; cannot compute KPIs.")
    return RunTrace(
        time_s=np.asarray(times, dtype=np.float64),
        interceptor_pos_m=np.asarray(interceptor, dtype=np.float64),
        target_pos_m=np.asarray(target, dtype=np.float64),
        saturated=np.asarray(saturated, dtype=bool),
    )


def _time_to_intercept_s(trace: RunTrace, range_m: NDArray[np.float64]) -> float:
    """First sim time the range reaches the miss-distance KPI [s], else ``inf``.

    ``inf`` (not the closest-approach time) marks a genuine failure to intercept, so the
    ``time_ok`` verdict cannot be accidentally satisfied by a run that merely got close.
    """
    within = np.flatnonzero(range_m <= constants.R_MISS_MAX_M)
    if within.size == 0:
        return float("inf")
    return float(trace.time_s[within[0]])


def _z_overshoot_m(trace: RunTrace, range_m: NDArray[np.float64]) -> float:
    """Altitude overshoot *past* the target during the approach [m], floored at 0.

    The Design Review's Z-overshoot is the interceptor flying **beyond** the target's
    altitude and having to come back — the oscillation the ``b`` penalty suppresses — not
    the benign initial altitude gap it closes on the way in. So overshoot is measured
    relative to the *side the interceptor starts on*:

    - starting at or **below** the target (``dz0 <= 0``, the usual climbing intercept) →
      overshoot is how far it rises **above** the target: ``max(0, max dz)``;
    - starting **above** the target (``dz0 > 0``, a descending intercept) → overshoot is
      how far it sinks **below** the target: ``max(0, -min dz)``.

    where ``dz = interceptor_z - target_z``. Measured up to and including the
    closest-approach frame (the approach is what the KPI governs; anything after is flyby).
    A monotone approach that never crosses the target altitude yields 0.
    """
    intercept_idx = int(np.argmin(range_m))
    dz = (
        trace.interceptor_pos_m[: intercept_idx + 1, frames.Z]
        - trace.target_pos_m[: intercept_idx + 1, frames.Z]
    )
    # Overshoot is the excursion beyond the target on the side opposite the start.
    if dz[0] > 0.0:  # descending intercept: overshoot is dipping below the target
        return float(max(0.0, -np.min(dz)))
    return float(max(0.0, np.max(dz)))  # climbing/level intercept: overshoot is rising above


def _max_target_speed_kmh(trace: RunTrace) -> float:
    """Peak target speed over the run [km/h] from a finite difference of its positions.

    Characterization only (the ≥ 83.6 km/h KPI is a stress metric); a single
    logged frame yields 0.
    """
    if trace.time_s.size < 2:
        return 0.0
    dt = np.diff(trace.time_s)
    dt[dt == 0.0] = np.inf  # guard against duplicate timestamps -> zero speed, not inf
    step_speed = np.linalg.norm(np.diff(trace.target_pos_m, axis=0), axis=1) / dt
    return float(np.max(step_speed) * 3.6)  # m/s -> km/h


def compute_kpis(trace: RunTrace, *, time_to_intercept_max_s: float) -> KpiRecord:
    """Measure every KPI for a run and grade it against the constant thresholds.

    ``time_to_intercept_max_s`` is supplied by the caller because it differs by target
    class (``T_INT_STATIC_MAX_S`` vs ``T_INT_MOVING_MAX_S``); the scenario declares which.
    """
    range_m = trace.range_m
    miss_distance_m = float(np.min(range_m))
    time_to_intercept_s = _time_to_intercept_s(trace, range_m)
    z_overshoot_m = _z_overshoot_m(trace, range_m)
    saturation_frac = float(np.mean(trace.saturated)) if trace.saturated.size else 0.0
    max_target_speed_kmh = _max_target_speed_kmh(trace)

    return KpiRecord(
        miss_distance_m=miss_distance_m,
        time_to_intercept_s=time_to_intercept_s,
        z_overshoot_m=z_overshoot_m,
        command_saturation_frac=saturation_frac,
        max_target_speed_kmh=max_target_speed_kmh,
        time_to_intercept_max_s=time_to_intercept_max_s,
        miss_ok=miss_distance_m <= constants.R_MISS_MAX_M,
        time_ok=time_to_intercept_s <= time_to_intercept_max_s,
        z_overshoot_ok=z_overshoot_m <= constants.Z_OVERSHOOT_MAX_M,
        saturation_ok=saturation_frac <= constants.CMD_SATURATION_MAX_FRAC,
    )


def kpis_from_log(csv_path: str | Path, *, time_to_intercept_max_s: float) -> KpiRecord:
    """Convenience: load a run log and compute its KPIs in one call."""
    return compute_kpis(load_run_trace(csv_path), time_to_intercept_max_s=time_to_intercept_max_s)
