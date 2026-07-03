"""MuJoCo interceptor plant (Role 1, Phase 1 — T1.1 / T1.2 / T1.8).

The concrete :class:`Plant`: it loads ``models/scene.xml``, applies the rotor wrench
from the :class:`RotorActuatorModel` (plus an optional reproducible wind force) as an
external body wrench, and advances the MuJoCo physics one step per call — closing the
Motor Mixer -> Simulation edge of the pipeline.

Frames (see ``common.frames``): the rotor wrench is computed in the body frame and
rotated to the world frame for ``xfrc_applied`` (which MuJoCo expects in world
coordinates about the body CoM). Body angular rates (the gyro analogue the inner loop
tracks) are read directly from the free-joint DOFs. World linear velocity is obtained
by finite-differencing the world position, which is frame-unambiguous and matches the
ground-truth kinematics convention.

Determinism (AGENTS.md): MuJoCo stepping is deterministic; the only randomness is the
seeded wind field, so a fixed seed + command stream reproduces the trajectory exactly.
The plant also asserts the model timestep equals ``1/SIM_HZ`` so the XML and the
constants can never drift apart silently.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray

from interceptor.common import frames, guards
from interceptor.common.types import MotorCommand
from interceptor.config import constants
from interceptor.simulation.actuators import RotorActuatorModel, RotorSaturationEvent
from interceptor.simulation.interfaces import Plant
from interceptor.simulation.wind import WindField

# Repo-root-relative default model path.
DEFAULT_SCENE_PATH = Path("models/scene.xml")

# Tolerance for the timestep / mass / inertia consistency assertions on load.
_CONSISTENCY_TOL = 1.0e-9


class MujocoPlant(Plant):
    """MuJoCo-backed interceptor dynamics with rotor actuation and optional wind."""

    def __init__(
        self,
        scene_path: str | Path = DEFAULT_SCENE_PATH,
        *,
        actuator: RotorActuatorModel | None = None,
        wind: WindField | None = None,
        initial_position_m: NDArray[np.float64] | None = None,
    ) -> None:
        self._model = mujoco.MjModel.from_xml_path(str(scene_path))
        self._data = mujoco.MjData(self._model)
        self._actuator = actuator or RotorActuatorModel()
        self._wind = wind

        self._body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, "interceptor"
        )
        self._joint_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_JOINT, "interceptor_root"
        )
        self._dof_adr = int(self._model.jnt_dofadr[self._joint_id])
        self._qpos_adr = int(self._model.jnt_qposadr[self._joint_id])
        self._target_mocap_id = int(
            self._model.body_mocapid[
                mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, "target")
            ]
        )

        self._assert_model_consistency()

        if initial_position_m is not None:
            start = guards.ensure_vector("initial_position_m", initial_position_m, 3)
            self._data.qpos[self._qpos_adr : self._qpos_adr + 3] = start
        mujoco.mj_forward(self._model, self._data)
        self._prev_position = self._world_position().copy()
        self._last_saturation: RotorSaturationEvent | None = None

    def _assert_model_consistency(self) -> None:
        """Fail loud if the XML drifts from the constants the rest of the code trusts."""
        expected_dt = 1.0 / constants.SIM_HZ
        if abs(self._model.opt.timestep - expected_dt) > _CONSISTENCY_TOL:
            raise ValueError(
                f"scene.xml timestep {self._model.opt.timestep} != 1/SIM_HZ "
                f"({expected_dt}); keep models/scene.xml and constants.SIM_HZ in sync."
            )
        mass = float(self._model.body_mass[self._body_id])
        if abs(mass - constants.QUAD_MASS_KG) > 1e-6:
            raise ValueError(
                f"Interceptor mass {mass} kg != QUAD_MASS_KG {constants.QUAD_MASS_KG}."
            )

    # ------------------------------------------------------------------ Plant API
    def step(self, motor_command: MotorCommand, dt_s: float) -> None:
        """Apply the rotor command (+ wind) and advance one physics step.

        ``dt_s`` must equal the model timestep; the multi-rate scheduler guarantees this
        and we assert it so a mismatched caller fails loud instead of silently
        integrating the wrong interval.
        """
        if abs(dt_s - self._model.opt.timestep) > _CONSISTENCY_TOL:
            raise ValueError(
                f"step dt {dt_s} != model timestep {self._model.opt.timestep}."
            )

        body_force, body_torque, saturation = self._actuator.body_wrench(
            motor_command.rotor_rpm
        )
        self._last_saturation = saturation

        rotation = frames.quat_to_rotation_matrix(self._body_quat())
        world_force = rotation @ body_force
        world_torque = rotation @ body_torque

        if self._wind is not None:
            world_force = world_force + self._wind.force_on(
                self._world_velocity(), float(self._data.time), constants.WIND_DRAG_COEFF_N_PER_M_S
            )

        self._prev_position = self._world_position().copy()
        self._data.xfrc_applied[self._body_id, :3] = world_force
        self._data.xfrc_applied[self._body_id, 3:] = world_torque
        mujoco.mj_step(self._model, self._data)
        guards.ensure_finite("interceptor_qpos", self._data.qpos)

    @property
    def position_m(self) -> NDArray[np.float64]:
        return self._world_position().copy()

    @property
    def body_rates_rad_s(self) -> NDArray[np.float64]:
        """Body-frame angular velocity [rad/s] from the free-joint DOFs (gyro analogue)."""
        return np.array(
            self._data.qvel[self._dof_adr + 3 : self._dof_adr + 6], dtype=np.float64
        )

    # ------------------------------------------------------- Phase 1 extras (concrete)
    @property
    def velocity_m_s(self) -> NDArray[np.float64]:
        """World-frame linear velocity [m/s], finite-differenced from world position."""
        dt = self._model.opt.timestep
        return (self._world_position() - self._prev_position) / dt

    @property
    def orientation_quat(self) -> NDArray[np.float64]:
        """Body-to-world orientation quaternion [w,x,y,z] for logging/replay."""
        return self._body_quat().copy()

    @property
    def last_saturation(self) -> RotorSaturationEvent | None:
        """The most recent rotor-saturation report (None before the first step)."""
        return self._last_saturation

    @property
    def sim_time_s(self) -> float:
        return float(self._data.time)

    @property
    def model(self) -> mujoco.MjModel:
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        return self._data

    def set_target_pose(
        self, position_m: NDArray[np.float64], quat_wxyz: NDArray[np.float64] | None = None
    ) -> None:
        """Drive the kinematic target body to a prescribed world pose (mocap)."""
        pos = guards.ensure_vector("position_m", position_m, 3)
        self._data.mocap_pos[self._target_mocap_id] = pos
        if quat_wxyz is not None:
            self._data.mocap_quat[self._target_mocap_id] = guards.ensure_vector(
                "quat_wxyz", quat_wxyz, 4
            )

    # ------------------------------------------------------------------ internals
    def _world_position(self) -> NDArray[np.float64]:
        return np.array(self._data.xpos[self._body_id], dtype=np.float64)

    def _world_velocity(self) -> NDArray[np.float64]:
        return self.velocity_m_s

    def _body_quat(self) -> NDArray[np.float64]:
        return np.array(self._data.xquat[self._body_id], dtype=np.float64)
