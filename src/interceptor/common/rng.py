"""Centralized, seeded RNG factory for deterministic, reproducible runs.

Determinism is a core value (AGENTS.md): identical seed + identical config must yield
byte-identical results. To guarantee that, **no component may call the global
``random`` / ``np.random`` functions**. Instead, every stochastic component is handed
its own independent ``numpy.random.Generator`` drawn from a single root seed via
``SeedSequence.spawn``. Spawning gives statistically independent streams whose values
depend only on the root seed and the (stable) stream name ordering.
"""

from __future__ import annotations

import numpy as np


class RngFactory:
    """Hands out named, independent, reproducible RNG streams from one root seed.

    Each unique ``name`` maps to its own child generator, so adding a new stochastic
    component never perturbs the number sequence seen by existing ones (as long as the
    name is stable). Requesting the same name twice returns the same generator.
    """

    def __init__(self, seed: int) -> None:
        self._seed = int(seed)
        self._root = np.random.SeedSequence(self._seed)
        self._streams: dict[str, np.random.Generator] = {}

    @property
    def seed(self) -> int:
        """The root seed, recorded in the run-config snapshot for reproducibility."""
        return self._seed

    def stream(self, name: str) -> np.random.Generator:
        """Return the independent generator for ``name``, creating it on first use.

        The child seed is derived from a stable hash of the name, so stream identity is
        independent of call order — two runs with the same seed get the same stream for
        the same name regardless of which component asks first.
        """
        if name not in self._streams:
            # Derive a stable per-name child sequence; combine root entropy with a
            # name-derived integer so the mapping name->stream is order-independent.
            name_entropy = int.from_bytes(name.encode("utf-8"), "little") % (2**63)
            child = np.random.SeedSequence(entropy=self._seed, spawn_key=(name_entropy,))
            self._streams[name] = np.random.default_rng(child)
        return self._streams[name]


def make_rng(seed: int) -> np.random.Generator:
    """Convenience: a single default generator for simple, one-stream callers/tests."""
    return np.random.default_rng(np.random.SeedSequence(int(seed)))
