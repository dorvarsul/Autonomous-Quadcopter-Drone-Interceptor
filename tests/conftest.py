"""Shared pytest fixtures for the infrastructure test suite.

All tests are headless and non-interactive (AGENTS.md / Phase 0 DoD). The MuJoCo-
dependent fixture is marked so it can be deselected on machines without a GL context.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from interceptor.common.rng import RngFactory


@pytest.fixture
def seed() -> int:
    """A fixed seed so every test is deterministic."""
    return 12345


@pytest.fixture
def rng_factory(seed: int) -> RngFactory:
    """A seeded RNG factory; components draw named independent streams from it."""
    return RngFactory(seed)


@pytest.fixture
def rng(rng_factory: RngFactory) -> np.random.Generator:
    """A single default generator for simple stochastic tests."""
    return rng_factory.stream("test")


@pytest.fixture
def run_dir(tmp_path: Path) -> Path:
    """An isolated per-test results directory."""
    return tmp_path / "run"


@pytest.fixture
def tiny_mjcf() -> str:
    """A trivial valid MJCF model string for MuJoCo smoke tests."""
    return """
    <mujoco>
      <worldbody>
        <light pos="0 0 3"/>
        <geom type="plane" size="2 2 0.1"/>
        <body pos="0 0 1">
          <joint type="free"/>
          <geom type="box" size="0.1 0.1 0.1" mass="1"/>
        </body>
      </worldbody>
    </mujoco>
    """
