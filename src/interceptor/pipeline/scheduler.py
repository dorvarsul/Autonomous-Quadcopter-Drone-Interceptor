"""Deterministic multi-rate clock for the pipeline.

Coordinates the sim step and the slower loops (inner 400 Hz, outer 50 Hz, estimation,
guidance) **without collapsing them** (AGENTS.md → Role 4/Role 6). The two control
loops keep their distinct rates here.

Determinism: rates are expressed as integer *periods in sim steps*, so a loop fires on
exact integer step boundaries — no floating-point accumulation/drift. Every slower rate
must divide ``sim_hz`` evenly; a non-integer ratio fails loud at construction.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass


@dataclass(frozen=True)
class Tick:
    """One sim step's schedule: which loops fire, and the current sim time.

    Booleans are independent: e.g. on a step where both the outer loop and estimation
    are due, both flags are True. ``sim_step`` always fires (the physics advances every
    step).
    """

    step_index: int  # 0-based sim step counter
    sim_time_s: float  # step_index / sim_hz [s]
    run_inner_loop: bool  # 400 Hz inner attitude/rate loop is due
    run_outer_loop: bool  # 50 Hz outer attitude-reference loop is due
    run_estimation: bool  # estimator update is due
    run_guidance: bool  # guidance update is due


class MultiRateScheduler:
    """Produces a deterministic stream of :class:`Tick` events for a fixed run length.

    Each slower rate is converted to a period (in sim steps); a rate fires whenever
    ``step_index % period == 0`` (so all loops fire on step 0). This makes the tick
    pattern a pure function of the rates and step count — identical across runs.
    """

    def __init__(
        self,
        sim_hz: int,
        inner_loop_hz: int,
        outer_loop_hz: int,
        estimation_hz: int,
        guidance_hz: int,
    ) -> None:
        self._sim_hz = int(sim_hz)
        self._inner_period = self._period("inner_loop_hz", inner_loop_hz)
        self._outer_period = self._period("outer_loop_hz", outer_loop_hz)
        self._estimation_period = self._period("estimation_hz", estimation_hz)
        self._guidance_period = self._period("guidance_hz", guidance_hz)

    def _period(self, name: str, rate_hz: int) -> int:
        """Convert a rate to an integer sim-step period, failing loud on a bad ratio."""
        rate_hz = int(rate_hz)
        if rate_hz <= 0:
            raise ValueError(f"{name} must be positive, got {rate_hz}.")
        if rate_hz > self._sim_hz:
            raise ValueError(
                f"{name}={rate_hz} Hz exceeds sim_hz={self._sim_hz} Hz; a loop cannot "
                f"run faster than the physics step."
            )
        if self._sim_hz % rate_hz != 0:
            raise ValueError(
                f"sim_hz={self._sim_hz} is not an integer multiple of {name}={rate_hz}; "
                f"this would cause timing drift."
            )
        return self._sim_hz // rate_hz

    @property
    def sim_hz(self) -> int:
        return self._sim_hz

    def ticks(self, num_steps: int) -> Iterator[Tick]:
        """Yield ``num_steps`` ticks starting at step 0 (where every loop fires)."""
        if num_steps < 0:
            raise ValueError(f"num_steps must be non-negative, got {num_steps}.")
        for step_index in range(num_steps):
            yield Tick(
                step_index=step_index,
                sim_time_s=step_index / self._sim_hz,
                run_inner_loop=step_index % self._inner_period == 0,
                run_outer_loop=step_index % self._outer_period == 0,
                run_estimation=step_index % self._estimation_period == 0,
                run_guidance=step_index % self._guidance_period == 0,
            )

    def expected_tick_count(self, num_steps: int, rate_hz: int) -> int:
        """How many times a given rate fires over ``num_steps`` (for tests/accounting)."""
        period = self._period("rate_hz", rate_hz)
        if num_steps <= 0:
            return 0
        return (num_steps - 1) // period + 1
