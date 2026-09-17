"""Rotor actuator & saturation correctness (Phase 1 — T1.2 / T1.9). No MuJoCo needed."""

from __future__ import annotations

import numpy as np
import pytest

from interceptor.config import constants
from interceptor.simulation.actuators import RotorActuatorModel


def test_hover_rpm_balances_weight() -> None:
    model = RotorActuatorModel()
    rpm = model.hover_rpm()
    thrust = model.thrust_per_rotor_n(np.full(4, rpm))
    assert float(np.sum(thrust)) == pytest.approx(
        constants.QUAD_MASS_KG * constants.GRAVITY_M_S2, rel=1e-9
    )


def test_max_rpm_gives_expected_max_thrust() -> None:
    model = RotorActuatorModel()
    thrust = model.thrust_per_rotor_n(np.full(4, constants.MOTOR_RPM_MAX))
    expected = constants.THRUST_COEFF_KT * constants.MOTOR_RPM_MAX**2
    np.testing.assert_allclose(thrust, expected)


def test_commands_beyond_limits_clamp_and_flag_saturation() -> None:
    model = RotorActuatorModel()
    event = model.clamp_rpm(np.array([constants.MOTOR_RPM_MAX * 2, -100.0, 0.0, 0.0]))
    assert event.any_saturated
    assert event.per_rotor[0] == constants.MOTOR_RPM_MAX
    assert event.per_rotor[1] == constants.MOTOR_RPM_MIN
    assert event.max_overshoot_rpm == pytest.approx(constants.MOTOR_RPM_MAX)


def test_in_bounds_command_is_not_saturated() -> None:
    model = RotorActuatorModel()
    event = model.clamp_rpm(np.full(4, model.hover_rpm()))
    assert not event.any_saturated
    assert event.max_overshoot_rpm == 0.0


def test_balanced_hover_has_zero_torque() -> None:
    model = RotorActuatorModel()
    _, torque, _ = model.body_wrench(np.full(4, model.hover_rpm()))
    np.testing.assert_allclose(torque, [0.0, 0.0, 0.0], atol=1e-9)


def test_differential_thrust_produces_expected_roll_and_pitch() -> None:
    model = RotorActuatorModel()
    base = model.hover_rpm()
    # More left (idx3) than right (idx1) -> positive roll about +X.
    rpm = np.array([base, base * 0.9, base, base * 1.1])
    _, torque, _ = model.body_wrench(rpm)
    assert torque[0] > 0.0  # roll
    assert torque[1] == pytest.approx(0.0, abs=1e-9)  # pitch unchanged
    # More back (idx2) than front (idx0) -> positive pitch about +Y.
    rpm2 = np.array([base * 0.9, base, base * 1.1, base])
    _, torque2, _ = model.body_wrench(rpm2)
    assert torque2[1] > 0.0
    assert torque2[0] == pytest.approx(0.0, abs=1e-9)


def test_differential_spin_produces_yaw() -> None:
    model = RotorActuatorModel()
    base = model.hover_rpm()
    # Speed up the +1-spin pair (front/back), slow the -1 pair -> net yaw torque.
    rpm = np.array([base * 1.1, base * 0.9, base * 1.1, base * 0.9])
    _, torque, _ = model.body_wrench(rpm)
    assert abs(torque[2]) > 0.0
