"""Ground-truth relative-kinematics correctness.

Validates LOS angle/rate and closing-speed against hand-computed geometries, and that
the sign/axis conventions match :mod:`interceptor.common.frames`.
"""

from __future__ import annotations

import numpy as np
import pytest

from interceptor.common import guards
from interceptor.simulation.kinematics import compute_relative_state

ZERO = np.zeros(3)


def test_target_along_x_has_zero_los_angles() -> None:
    state = compute_relative_state(ZERO, ZERO, np.array([10.0, 0.0, 0.0]), ZERO)
    assert state.range_m == pytest.approx(10.0)
    assert state.los_azimuth_rad == pytest.approx(0.0)
    assert state.los_elevation_rad == pytest.approx(0.0)


def test_target_along_y_has_ninety_degree_azimuth() -> None:
    state = compute_relative_state(ZERO, ZERO, np.array([0.0, 7.0, 0.0]), ZERO)
    assert state.los_azimuth_rad == pytest.approx(np.pi / 2)


def test_target_overhead_has_ninety_degree_elevation() -> None:
    state = compute_relative_state(ZERO, ZERO, np.array([0.0, 0.0, 4.0]), ZERO)
    assert state.los_elevation_rad == pytest.approx(np.pi / 2)


def test_azimuth_rate_for_crossing_target() -> None:
    # Target at x=10 moving +Y at 2 m/s; az_rate = (rx*vy - ry*vx)/(rx^2+ry^2) = 20/100.
    state = compute_relative_state(
        ZERO, ZERO, np.array([10.0, 0.0, 0.0]), np.array([0.0, 2.0, 0.0])
    )
    assert state.los_rate_rad_s[0] == pytest.approx(0.2)
    assert state.los_rate_rad_s[1] == pytest.approx(0.0)


def test_closing_speed_positive_when_approaching() -> None:
    # Target at x=10 moving toward origin at 3 m/s -> closing speed +3.
    state = compute_relative_state(
        ZERO, ZERO, np.array([10.0, 0.0, 0.0]), np.array([-3.0, 0.0, 0.0])
    )
    assert state.closing_speed_m_s == pytest.approx(3.0)


def test_closing_speed_negative_when_receding() -> None:
    state = compute_relative_state(
        ZERO, ZERO, np.array([10.0, 0.0, 0.0]), np.array([5.0, 0.0, 0.0])
    )
    assert state.closing_speed_m_s == pytest.approx(-5.0)


def test_relative_velocity_accounts_for_interceptor_motion() -> None:
    state = compute_relative_state(
        ZERO, np.array([1.0, 0.0, 0.0]), np.array([10.0, 0.0, 0.0]), np.array([4.0, 0.0, 0.0])
    )
    np.testing.assert_allclose(state.relative_velocity_m_s, [3.0, 0.0, 0.0])


def test_zero_range_fails_loud() -> None:
    with pytest.raises(guards.ContractViolationError):
        compute_relative_state(ZERO, ZERO, ZERO, ZERO)
