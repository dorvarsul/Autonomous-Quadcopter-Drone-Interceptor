"""Coordinate frames, rotation conventions, and frame-transform helpers.

This module is the documented authority for *which way is up* and *how rotations are
represented*. Estimation (Role 2) and Guidance (Role 3) must follow the LOS sign rules
defined here so their math composes correctly.

Frame conventions
-----------------
World frame ``W`` (inertial):
    Right-handed, **Z-up**. The altitude axis is **+Z** (up). X is "north"/forward
    reference, Y completes the right-handed triad. The Design Review flags Z as
    overshoot-sensitive, so it is called out explicitly everywhere it appears.

Body frame ``B`` (attached to the interceptor):
    Right-handed, FLU convention: **+X forward**, **+Y left**, **+Z up**. Aligns with
    the world frame at zero attitude.

Rotation representation
-----------------------
Quaternions are the **primary** representation: unit quaternion ``q = [w, x, y, z]``
(scalar-first, Hamilton convention) describing the body-to-world rotation, i.e.
``v_world = R(q) @ v_body``.

Euler angles are **secondary** (for human-readable attitude references): intrinsic
Z-Y-X yaw-pitch-roll, with roll ``phi`` about body X, pitch ``theta`` about body Y,
yaw ``psi`` about body Z. Right-hand-rule positive.

Line-of-Sight (LOS) conventions
-------------------------------
LOS is the direction from interceptor to target, expressed in the world frame by two
angles:
    ``los_azimuth``   : angle in the X-Y plane, measured from +X toward +Y
                        (right-hand-rule about +Z). Range (-pi, pi].
    ``los_elevation`` : angle above the X-Y plane toward +Z. Range [-pi/2, pi/2].
The **LOS rate** is the time derivative of these angles (rad/s). Proportional
Navigation nulls the LOS rate; a positive ``los_rate`` about an axis means the LOS is
rotating right-hand-positive about that axis. Guidance consumes LOS rate already
sign-corrected to this convention.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

# Axis indices for readability when indexing 3-vectors.
X, Y, Z = 0, 1, 2

# Index of the altitude axis in the world frame. Named because it is special.
ALTITUDE_AXIS: int = Z


def quat_normalize(q: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return the unit quaternion; fail loud on a zero-norm (degenerate) quaternion."""
    q = np.asarray(q, dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        raise ValueError("Cannot normalize a near-zero quaternion.")
    return q / norm


def quat_to_rotation_matrix(q: NDArray[np.float64]) -> NDArray[np.float64]:
    """Body-to-world rotation matrix ``R`` from quaternion ``[w, x, y, z]``.

    ``v_world = R @ v_body``. The quaternion is normalized first so callers need not
    pre-normalize.
    """
    w, x, y, z = quat_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def body_to_world(q: NDArray[np.float64], v_body: NDArray[np.float64]) -> NDArray[np.float64]:
    """Rotate a vector from the body frame into the world frame."""
    return quat_to_rotation_matrix(q) @ np.asarray(v_body, dtype=np.float64)


def world_to_body(q: NDArray[np.float64], v_world: NDArray[np.float64]) -> NDArray[np.float64]:
    """Rotate a vector from the world frame into the body frame.

    Uses the transpose of the body-to-world matrix (rotation matrices are orthonormal,
    so the inverse equals the transpose).
    """
    return quat_to_rotation_matrix(q).T @ np.asarray(v_world, dtype=np.float64)


def euler_to_quat(roll: float, pitch: float, yaw: float) -> NDArray[np.float64]:
    """Intrinsic Z-Y-X (yaw-pitch-roll) Euler angles [rad] to quaternion ``[w,x,y,z]``."""
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=np.float64,
    )


def quat_to_euler(q: NDArray[np.float64]) -> NDArray[np.float64]:
    """Quaternion ``[w,x,y,z]`` to intrinsic Z-Y-X Euler ``[roll, pitch, yaw]`` [rad]."""
    w, x, y, z = quat_normalize(q)
    # Roll (about body X).
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    # Pitch (about body Y), clamped to avoid NaN at the singularity.
    sin_pitch = np.clip(2 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sin_pitch)
    # Yaw (about body Z).
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw], dtype=np.float64)


def los_rate_from_relative(
    relative_position_world: NDArray[np.float64],
    relative_velocity_world: NDArray[np.float64],
) -> NDArray[np.float64]:
    """LOS angular rate ``[azimuth_rate, elevation_rate]`` [rad/s] from relative pos/vel.

    Pure geometry (no ground truth): the same analytic derivative documented in the
    simulation-layer kinematics, exposed here in ``common`` so the Estimation layer can
    turn its *filtered* relative position/velocity estimate into a clean LOS rate for
    Guidance without importing the Simulation layer. Returns a zero azimuth rate when the
    target is near-vertical (horizontal radius ~0), where azimuth is ill-conditioned.

    See the module docstring for the sign convention (Guidance nulls this rate).
    """
    r = np.asarray(relative_position_world, dtype=np.float64)
    v = np.asarray(relative_velocity_world, dtype=np.float64)
    rx, ry, rz = float(r[X]), float(r[Y]), float(r[Z])
    vx, vy, vz = float(v[X]), float(v[Y]), float(v[Z])
    horizontal_sq = rx * rx + ry * ry
    horizontal = float(np.sqrt(horizontal_sq))
    range_sq = float(rx * rx + ry * ry + rz * rz)
    if horizontal < 1e-9 or range_sq < 1e-18:
        return np.zeros(2, dtype=np.float64)
    azimuth_rate = (rx * vy - ry * vx) / horizontal_sq
    dh_dt = (rx * vx + ry * vy) / horizontal
    elevation_rate = (horizontal * vz - rz * dh_dt) / range_sq
    return np.array([azimuth_rate, elevation_rate], dtype=np.float64)


def los_angles(relative_position_world: NDArray[np.float64]) -> tuple[float, float]:
    """LOS azimuth and elevation [rad] for a world-frame interceptor->target vector.

    See the module docstring for the sign convention. Fails loud on a zero-range
    vector, where LOS angles are undefined.
    """
    r = np.asarray(relative_position_world, dtype=np.float64)
    horizontal = float(np.hypot(r[X], r[Y]))
    if horizontal < 1e-12 and abs(r[Z]) < 1e-12:
        raise ValueError("LOS angles undefined for a zero-range relative position.")
    azimuth = float(np.arctan2(r[Y], r[X]))
    elevation = float(np.arctan2(r[Z], horizontal))
    return azimuth, elevation
