"""Wind/gust disturbance reproducibility & calm-reduction."""

from __future__ import annotations

import numpy as np
import pytest

from interceptor.common.rng import RngFactory
from interceptor.config import constants
from interceptor.config.params import WindParams
from interceptor.simulation.wind import WindField, calm, gusty, moderate


def test_calm_preset_is_identically_zero() -> None:
    wind = WindField(calm(), rng=None, horizon_s=10.0)
    for t in (0.0, 1.0, 5.0, 9.9, 100.0):
        np.testing.assert_array_equal(wind.velocity_at(t), [0.0, 0.0, 0.0])


def test_steady_only_is_constant() -> None:
    wind = WindField(WindParams(steady_velocity_m_s=(3.0, -1.0, 0.0)), rng=None, horizon_s=5.0)
    for t in (0.0, 2.0, 4.0):
        np.testing.assert_allclose(wind.velocity_at(t), [3.0, -1.0, 0.0])


def test_gusts_require_seed() -> None:
    with pytest.raises(ValueError):
        WindField(gusty(), rng=None, horizon_s=5.0)


def test_gust_series_is_reproducible_for_fixed_seed() -> None:
    def series(seed: int) -> np.ndarray:
        rng = RngFactory(seed).stream("wind")
        wind = WindField(moderate(), rng=rng, horizon_s=5.0)
        return np.array([wind.velocity_at(t) for t in np.linspace(0, 5, 200)])

    np.testing.assert_array_equal(series(11), series(11))
    assert not np.allclose(series(11), series(22))


def test_gust_std_is_in_the_right_ballpark() -> None:
    rng = RngFactory(5).stream("wind")
    params = WindParams(
        steady_velocity_m_s=(0.0, 0.0, 0.0), gust_std_m_s=2.0, gust_correlation_time_s=1.0
    )
    wind = WindField(params, rng=rng, horizon_s=200.0, dt_s=1.0 / 100.0)
    samples = np.array([wind.velocity_at(t) for t in np.linspace(1.0, 199.0, 4000)])
    # Stationary OU std should approach the configured sigma per axis.
    assert samples[:, 0].std() == pytest.approx(2.0, rel=0.2)


def test_force_is_zero_in_calm_with_zero_body_velocity() -> None:
    wind = WindField(calm(), rng=None, horizon_s=5.0)
    force = wind.force_on(np.zeros(3), 1.0, constants.WIND_DRAG_COEFF_N_PER_M_S)
    np.testing.assert_array_equal(force, [0.0, 0.0, 0.0])


def test_force_opposes_relative_air_velocity() -> None:
    wind = WindField(WindParams(steady_velocity_m_s=(5.0, 0.0, 0.0)), rng=None, horizon_s=5.0)
    # Body still, wind +X -> force should push +X.
    force = wind.force_on(np.zeros(3), 1.0, 0.1)
    assert force[0] == pytest.approx(0.5)
    np.testing.assert_allclose(force[1:], [0.0, 0.0])
