"""Unit tests for coordinate frames, rotations, and LOS conventions."""

from __future__ import annotations

import numpy as np
import pytest

from interceptor.common import frames


def test_identity_quat_is_identity_rotation():
    q = np.array([1.0, 0.0, 0.0, 0.0])
    R = frames.quat_to_rotation_matrix(q)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-12)


def test_world_body_roundtrip():
    q = frames.euler_to_quat(0.3, -0.2, 1.1)
    v_world = np.array([1.0, -2.0, 0.5])
    v_body = frames.world_to_body(q, v_world)
    np.testing.assert_allclose(frames.body_to_world(q, v_body), v_world, atol=1e-12)


def test_euler_quat_roundtrip():
    roll, pitch, yaw = 0.2, -0.4, 2.0
    q = frames.euler_to_quat(roll, pitch, yaw)
    np.testing.assert_allclose(frames.quat_to_euler(q), [roll, pitch, yaw], atol=1e-10)


def test_yaw_rotation_maps_x_to_y():
    # 90 deg yaw about +Z should rotate body +X to world +Y.
    q = frames.euler_to_quat(0.0, 0.0, np.pi / 2)
    out = frames.body_to_world(q, np.array([1.0, 0.0, 0.0]))
    np.testing.assert_allclose(out, [0.0, 1.0, 0.0], atol=1e-12)


def test_los_angles_along_x():
    az, el = frames.los_angles(np.array([5.0, 0.0, 0.0]))
    assert az == pytest.approx(0.0)
    assert el == pytest.approx(0.0)


def test_los_elevation_straight_up():
    az, el = frames.los_angles(np.array([0.0, 0.0, 3.0]))
    assert el == pytest.approx(np.pi / 2)


def test_los_azimuth_along_y():
    az, el = frames.los_angles(np.array([0.0, 4.0, 0.0]))
    assert az == pytest.approx(np.pi / 2)


def test_altitude_axis_is_z():
    assert frames.ALTITUDE_AXIS == frames.Z == 2


def test_zero_quat_fails_loud():
    with pytest.raises(ValueError):
        frames.quat_to_rotation_matrix(np.zeros(4))


def test_zero_range_los_fails_loud():
    with pytest.raises(ValueError):
        frames.los_angles(np.zeros(3))
