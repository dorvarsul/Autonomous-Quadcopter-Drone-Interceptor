"""MuJoCo plant: hover, stability, saturation, wind (Phase 1 — T1.1/T1.2/T1.6/T1.8).

Marked ``mujoco`` because it loads and steps the real engine. Headless and
deterministic: no window is opened and no rendering occurs here.
"""

from __future__ import annotations

import numpy as np
import pytest

from interceptor.common.rng import RngFactory
from interceptor.common.types import MotorCommand
from interceptor.config import constants
from interceptor.simulation.actuators import RotorActuatorModel
from interceptor.simulation.mujoco_plant import MujocoPlant
from interceptor.simulation.wind import WindField, calm, gusty

pytestmark = pytest.mark.mujoco

SIM_DT = 1.0 / constants.SIM_HZ


def _hover_command(plant: MujocoPlant) -> MotorCommand:
    rpm = RotorActuatorModel().hover_rpm()
    return MotorCommand(rotor_rpm=np.full(4, rpm))


def test_model_loads_and_is_consistent() -> None:
    plant = MujocoPlant(initial_position_m=np.array([0.0, 0.0, 2.0]))
    assert plant.model.opt.timestep == pytest.approx(SIM_DT)
    np.testing.assert_allclose(plant.position_m, [0.0, 0.0, 2.0], atol=1e-6)


def test_hover_holds_altitude_over_30_seconds() -> None:
    plant = MujocoPlant(initial_position_m=np.array([0.0, 0.0, 2.0]))
    command = _hover_command(plant)
    steps = int(30.0 * constants.SIM_HZ)
    for _ in range(steps):
        plant.step(command, SIM_DT)
    final = plant.position_m
    # Equilibrium thrust == weight: altitude and horizontal drift stay tiny.
    assert final[2] == pytest.approx(2.0, abs=0.05)
    assert np.linalg.norm(final[:2]) < 0.05
    assert np.all(np.isfinite(plant.position_m))


def test_zero_thrust_falls_under_gravity() -> None:
    plant = MujocoPlant(initial_position_m=np.array([0.0, 0.0, 5.0]))
    zero = MotorCommand(rotor_rpm=np.zeros(4))
    for _ in range(int(0.5 * constants.SIM_HZ)):
        plant.step(zero, SIM_DT)
    # After 0.5 s of free fall from rest: drop ~ 0.5*g*t^2 ~ 1.23 m.
    drop = 5.0 - plant.position_m[2]
    assert drop == pytest.approx(0.5 * constants.GRAVITY_M_S2 * 0.5**2, rel=0.1)


def test_saturation_is_reported_on_overspeed_command() -> None:
    plant = MujocoPlant(initial_position_m=np.array([0.0, 0.0, 2.0]))
    over = MotorCommand(rotor_rpm=np.full(4, constants.MOTOR_RPM_MAX * 2))
    plant.step(over, SIM_DT)
    assert plant.last_saturation is not None
    assert plant.last_saturation.any_saturated


def test_stepping_is_deterministic() -> None:
    def run() -> np.ndarray:
        plant = MujocoPlant(initial_position_m=np.array([0.0, 0.0, 2.0]))
        cmd = _hover_command(plant)
        for _ in range(500):
            plant.step(cmd, SIM_DT)
        return plant.position_m

    np.testing.assert_array_equal(run(), run())


def test_wrong_dt_fails_loud() -> None:
    plant = MujocoPlant(initial_position_m=np.array([0.0, 0.0, 2.0]))
    with pytest.raises(ValueError):
        plant.step(_hover_command(plant), SIM_DT * 2)


def test_calm_wind_matches_no_wind_exactly() -> None:
    def run(wind: WindField | None) -> np.ndarray:
        plant = MujocoPlant(initial_position_m=np.array([0.0, 0.0, 2.0]), wind=wind)
        cmd = _hover_command(plant)
        for _ in range(400):
            plant.step(cmd, SIM_DT)
        return plant.position_m

    no_wind = run(None)
    calm_wind = run(WindField(calm(), rng=None, horizon_s=2.0))
    np.testing.assert_allclose(no_wind, calm_wind, atol=1e-12)


def test_gusty_wind_perturbs_hover() -> None:
    rng = RngFactory(3).stream("wind")
    wind = WindField(gusty(), rng=rng, horizon_s=3.0)
    plant = MujocoPlant(initial_position_m=np.array([0.0, 0.0, 2.0]), wind=wind)
    cmd = _hover_command(plant)
    for _ in range(400):
        plant.step(cmd, SIM_DT)
    # Gusts push the hovering quad off the start point horizontally.
    assert np.linalg.norm(plant.position_m[:2]) > 1e-3


def test_target_mocap_pose_can_be_set() -> None:
    plant = MujocoPlant(initial_position_m=np.array([0.0, 0.0, 2.0]))
    plant.set_target_pose(np.array([5.0, -3.0, 4.0]))
    plant.step(_hover_command(plant), SIM_DT)
    target_body_id = plant.model.body("target").id
    np.testing.assert_allclose(plant.data.xpos[target_body_id], [5.0, -3.0, 4.0], atol=1e-6)
