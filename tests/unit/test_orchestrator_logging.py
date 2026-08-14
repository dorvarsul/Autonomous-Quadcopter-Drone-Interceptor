"""Unit tests for the orchestrator's honest command-saturation logging (Role 6 / KPI).

The per-step ``saturated`` column feeds the command-saturation KPI. It must be the
combined actuator-chain flag: True if the limiter clamped the acceleration request OR the
motor mixer clamped a rotor to an RPM limit. Counting only the limiter would hide mixer
saturation on aggressive attitude slews (AGENTS.md → saturation must stay measurable).
"""

from __future__ import annotations

import numpy as np
import pytest

from interceptor.common.types import LimitedAccelerationCommand, MotorCommand
from interceptor.pipeline.orchestrator import LOG_FIELDS, StubOrchestrator
from interceptor.pipeline.scheduler import Tick


def _row(*, limiter_sat: bool, mixer_sat: bool) -> dict:
    tick = Tick(
        step_index=0,
        sim_time_s=0.0,
        run_inner_loop=True,
        run_outer_loop=True,
        run_estimation=True,
        run_guidance=True,
    )
    limited = LimitedAccelerationCommand(
        acceleration_m_s2=np.zeros(3),
        saturated=limiter_sat,
        saturation_magnitude_m_s2=1.0 if limiter_sat else 0.0,
    )
    motor = MotorCommand(rotor_rpm=np.full(4, 5000.0), saturated=mixer_sat)
    return StubOrchestrator._log_row(
        tick, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3), None, limited, motor
    )


@pytest.mark.parametrize(
    ("limiter_sat", "mixer_sat", "expected"),
    [(False, False, False), (True, False, True), (False, True, True), (True, True, True)],
)
def test_saturated_is_limiter_or_mixer(limiter_sat, mixer_sat, expected):
    row = _row(limiter_sat=limiter_sat, mixer_sat=mixer_sat)
    assert row["saturated"] is expected
    assert row["limiter_saturated"] is limiter_sat
    assert row["mixer_saturated"] is mixer_sat


def test_mixer_only_saturation_is_not_hidden():
    """The regression this fix targets: mixer saturates, limiter does not -> KPI counts it."""
    assert _row(limiter_sat=False, mixer_sat=True)["saturated"] is True


def test_per_stage_flag_columns_are_logged():
    """The diagnostic per-stage columns exist so saturation can be attributed to a stage."""
    assert "limiter_saturated" in LOG_FIELDS
    assert "mixer_saturated" in LOG_FIELDS
    assert set(_row(limiter_sat=True, mixer_sat=False)) == set(LOG_FIELDS)
