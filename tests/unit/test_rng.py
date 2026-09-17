"""Unit tests for the seeded RNG factory (determinism + stream independence)."""

from __future__ import annotations

import numpy as np

from interceptor.common.rng import RngFactory, make_rng


def test_same_seed_same_sequence():
    a = RngFactory(7).stream("sensor").random(10)
    b = RngFactory(7).stream("sensor").random(10)
    np.testing.assert_array_equal(a, b)


def test_different_seed_different_sequence():
    a = RngFactory(1).stream("sensor").random(10)
    b = RngFactory(2).stream("sensor").random(10)
    assert not np.array_equal(a, b)


def test_streams_are_independent():
    factory = RngFactory(7)
    sensor = factory.stream("sensor").random(10)
    wind = factory.stream("wind").random(10)
    assert not np.array_equal(sensor, wind)


def test_stream_identity_is_order_independent():
    # Requesting streams in a different order must not change their values.
    f1 = RngFactory(7)
    wind_first = f1.stream("wind").random(5)
    sensor_first = f1.stream("sensor").random(5)

    f2 = RngFactory(7)
    sensor_second = f2.stream("sensor").random(5)
    wind_second = f2.stream("wind").random(5)

    np.testing.assert_array_equal(wind_first, wind_second)
    np.testing.assert_array_equal(sensor_first, sensor_second)


def test_repeated_stream_returns_same_generator():
    factory = RngFactory(7)
    assert factory.stream("a") is factory.stream("a")


def test_make_rng_is_deterministic():
    np.testing.assert_array_equal(make_rng(3).random(5), make_rng(3).random(5))
