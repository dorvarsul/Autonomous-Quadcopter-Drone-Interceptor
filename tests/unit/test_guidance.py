"""Unit tests for the guidance layer (Role 3): time-to-go, ZEM, and OGL."""

from __future__ import annotations

import numpy as np

from interceptor.common import frames
from interceptor.common.types import AccelerationCommand, TargetStateEstimate
from interceptor.config.params import GuidanceParams
from interceptor.guidance.ogl import OptimalGuidanceLaw, lag_aware_nav_ratio
from interceptor.guidance.time_to_go import closing_speed_m_s, time_to_go_s
from interceptor.guidance.zem import perpendicular_component, zero_effort_miss


def _estimate(rel_pos, rel_vel=(0.0, 0.0, 0.0), rel_acc=(0.0, 0.0, 0.0)) -> TargetStateEstimate:
    rel_pos = np.asarray(rel_pos, dtype=float)
    return TargetStateEstimate(
        relative_position_m=rel_pos,
        relative_velocity_m_s=np.asarray(rel_vel, dtype=float),
        range_m=float(np.linalg.norm(rel_pos)),
        los_rate_rad_s=np.zeros(2),
        angular_rates_rad_s=np.zeros(3),
        covariance=np.eye(9),
        quality=1.0,
        relative_acceleration_m_s2=np.asarray(rel_acc, dtype=float),
    )


# --------------------------------------------------------------------- time-to-go
def test_time_to_go_uses_range_over_closing_speed_when_closing():
    p = GuidanceParams()
    r = np.array([10.0, 0.0, 0.0])
    v = np.array([-2.0, 0.0, 0.0])  # closing at 2 m/s
    assert closing_speed_m_s(r, v) == 2.0
    assert time_to_go_s(r, v, p) == 5.0  # 10 / 2


def test_time_to_go_falls_back_to_reference_speed_from_rest():
    p = GuidanceParams()  # reference_closing_speed_m_s = 4.25
    r = np.array([10.0, 0.0, 0.0])
    v = np.zeros(3)  # not closing at all
    assert time_to_go_s(r, v, p) == 10.0 / p.reference_closing_speed_m_s


def test_time_to_go_is_clamped():
    p = GuidanceParams()
    # Enormous range at the reference speed would exceed the cap.
    far = np.array([1.0e6, 0.0, 0.0])
    assert time_to_go_s(far, np.zeros(3), p) == p.time_to_go_max_s
    # Tiny range with huge closing speed would underflow the floor.
    near = np.array([1.0e-4, 0.0, 0.0])
    assert time_to_go_s(near, np.array([-1000.0, 0.0, 0.0]), p) == p.time_to_go_min_s


# --------------------------------------------------------------------------- ZEM
def test_zem_perpendicular_component_is_zero_for_constant_bearing():
    """Collinear closing (constant bearing) leaves ZEM along the LOS: no perpendicular part."""
    r = np.array([10.0, 0.0, 0.0])
    v = np.array([-1.0, 0.0, 0.0])
    zem = zero_effort_miss(r, v, np.zeros(3), time_to_go_s=5.0)
    perp = perpendicular_component(zem, r)
    assert np.linalg.norm(perp) < 1e-12


def test_zem_perpendicular_is_nonzero_for_drifting_los():
    r = np.array([10.0, 0.0, 0.0])
    v = np.array([0.0, 1.0, 0.0])  # lateral drift -> rotating LOS
    zem = zero_effort_miss(r, v, np.zeros(3), time_to_go_s=5.0)
    perp = perpendicular_component(zem, r)
    np.testing.assert_allclose(perp, np.array([0.0, 5.0, 0.0]))


def test_zem_includes_augmented_acceleration_term():
    r = np.array([10.0, 0.0, 0.0])
    a = np.array([0.0, 2.0, 0.0])
    zem = zero_effort_miss(r, np.zeros(3), a, time_to_go_s=2.0)
    # r + 0.5 * a * t^2 = [10,0,0] + 0.5*[0,2,0]*4
    np.testing.assert_allclose(zem, np.array([10.0, 4.0, 0.0]))


# --------------------------------------------------------------------------- OGL
def test_ogl_name():
    assert OptimalGuidanceLaw().name == "OGL"


def test_lag_aware_nav_ratio_tends_to_three_far_from_intercept():
    # Large t_go/T -> classic PN limit N' = 3.
    assert abs(lag_aware_nav_ratio(100.0, 0.2) - 3.0) < 0.05


def test_ogl_commands_toward_a_static_target_from_rest():
    ogl = OptimalGuidanceLaw()
    est = _estimate([6.0, 0.0, 3.0])  # target ahead and above, interceptor at rest
    cmd = ogl.compute(est)
    assert isinstance(cmd, AccelerationCommand)
    # The command has a positive projection on the line of sight (it closes range).
    assert float(np.dot(cmd.acceleration_m_s2, est.relative_position_m)) > 0.0


def test_ogl_is_well_conditioned_near_intercept():
    """As t_go -> 0 the 1/t_go^2 term must not blow up (t_go floored, N' clamped)."""
    ogl = OptimalGuidanceLaw()
    est = _estimate([0.01, 0.0, 0.0], rel_vel=[-100.0, 0.0, 0.0])
    cmd = ogl.compute(est)
    assert np.all(np.isfinite(cmd.acceleration_m_s2))


def test_ogl_altitude_penalty_de_weights_z():
    """With b>0 the Z command is attenuated relative to an equal X offset; b=0 leaves it."""
    est = _estimate([5.0, 0.0, 5.0])  # symmetric X/Z geometry
    with_penalty = OptimalGuidanceLaw(GuidanceParams(altitude_penalty_b=0.1)).compute(est)
    no_penalty = OptimalGuidanceLaw(GuidanceParams(altitude_penalty_b=0.0)).compute(est)
    ax_p, az_p = with_penalty.acceleration_m_s2[frames.X], with_penalty.acceleration_m_s2[frames.Z]
    ax_0, az_0 = no_penalty.acceleration_m_s2[frames.X], no_penalty.acceleration_m_s2[frames.Z]
    assert az_0 == ax_0  # symmetric with no penalty
    assert abs(az_p) < abs(ax_p)  # Z de-weighted with the penalty
    # Stronger penalty -> even smaller Z command.
    stronger = OptimalGuidanceLaw(GuidanceParams(altitude_penalty_b=0.5)).compute(est)
    assert abs(stronger.acceleration_m_s2[frames.Z]) < abs(az_p)


def test_ogl_ignores_relative_acceleration_by_default():
    """The augmented term is opt-in; a non-zero relative accel must not change the command."""
    ogl = OptimalGuidanceLaw()
    plain = ogl.compute(_estimate([6.0, 0.0, 3.0]))
    with_acc = ogl.compute(_estimate([6.0, 0.0, 3.0], rel_acc=[5.0, 5.0, 5.0]))
    np.testing.assert_allclose(plain.acceleration_m_s2, with_acc.acceleration_m_s2)
