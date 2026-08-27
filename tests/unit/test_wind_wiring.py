"""Unit tests for wind wiring: the orchestrator's wind-field factory and the
scenario ``wind_preset`` shorthand.

The wiring must satisfy two invariants: the **calm** profile reduces to *no* wind field (so
undisturbed runs stay byte-identical to the baseline), and any disturbed profile yields a
reproducible field seeded from a dedicated RNG stream. These run without MuJoCo — the
orchestrator helper and scenario parser are pure Python.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from interceptor.analysis.scenarios import scenario_from_dict
from interceptor.common.rng import RngFactory
from interceptor.config.params import WindParams, default_params
from interceptor.pipeline.orchestrator import _build_wind_field
from interceptor.simulation.wind import WindField, gusty, moderate


def _params_with_wind(wind: WindParams):
    return replace(default_params(), wind=wind)


def test_calm_profile_builds_no_wind_field():
    """The calm default must map to None so the dynamics stay exactly undisturbed."""
    assert _build_wind_field(default_params(), RngFactory(0)) is None


def test_steady_only_profile_needs_no_rng_stream():
    """A steady breeze with no gusts is deterministic and needs no RNG."""
    params = _params_with_wind(WindParams(steady_velocity_m_s=(3.0, 0.0, 0.0), gust_std_m_s=0.0))
    field = _build_wind_field(params, RngFactory(0))
    assert isinstance(field, WindField)
    # With no gusts the air velocity is exactly the steady vector at every time.
    assert np.allclose(field.velocity_at(1.23), [3.0, 0.0, 0.0])


def test_gusty_profile_is_reproducible():
    """A gusty profile yields a byte-reproducible gust series for a fixed seed."""
    params = _params_with_wind(gusty())
    a = _build_wind_field(params, RngFactory(7))
    b = _build_wind_field(params, RngFactory(7))
    assert isinstance(a, WindField)
    times = [0.0, 0.5, 1.0, 2.0]
    assert np.allclose([a.velocity_at(t) for t in times], [b.velocity_at(t) for t in times])
    # A different seed gives a different gust realization (the stream is actually used).
    c = _build_wind_field(params, RngFactory(8))
    assert not np.allclose(a.velocity_at(1.0), c.velocity_at(1.0))


def test_scenario_wind_preset_applies_named_profile():
    """``wind_preset`` sets params.wind from the shared preset table (DRY)."""
    scenario = scenario_from_dict({
        "name": "w",
        "time_limit_s": 12.0,
        "wind_preset": "moderate",
        "interceptor": {"start_m": [0.0, 0.0, 2.0]},
        "target": {"type": "static", "position_m": [6.0, 0.0, 4.0]},
    })
    assert scenario.params.wind == moderate()


def test_scenario_rejects_both_preset_and_explicit_wind():
    """Setting both ``wind_preset`` and params.wind is ambiguous and fails loud."""
    with pytest.raises(ValueError, match="both 'wind_preset' and params.wind"):
        scenario_from_dict({
            "name": "w",
            "time_limit_s": 12.0,
            "wind_preset": "gusty",
            "params": {"wind": {"gust_std_m_s": 2.0}},
            "interceptor": {"start_m": [0.0, 0.0, 2.0]},
            "target": {"type": "static", "position_m": [6.0, 0.0, 4.0]},
        })


def test_scenario_rejects_unknown_wind_preset():
    with pytest.raises(ValueError, match="Unknown wind_preset"):
        scenario_from_dict({
            "name": "w",
            "time_limit_s": 12.0,
            "wind_preset": "hurricane",
            "interceptor": {"start_m": [0.0, 0.0, 2.0]},
            "target": {"type": "static", "position_m": [6.0, 0.0, 4.0]},
        })
