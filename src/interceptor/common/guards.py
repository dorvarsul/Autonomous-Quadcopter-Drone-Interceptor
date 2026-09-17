"""Fail-loud guards shared by every layer.

Core value (AGENTS.md): *fail loud, not silent*. NaN/Inf, divergence, and out-of-range
states must raise immediately rather than propagate quietly through the pipeline and
corrupt a run's results. Any layer may import these.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


class PipelineError(RuntimeError):
    """Base class for loud pipeline failures."""


class NumericalInstabilityError(PipelineError):
    """Raised when a value is non-finite (NaN/Inf) or otherwise diverged."""


class ContractViolationError(PipelineError):
    """Raised when a data contract (shape/range/ordering) is broken."""


def ensure_finite(name: str, value: NDArray[np.float64] | float) -> NDArray[np.float64]:
    """Return ``value`` as a float64 array, raising if any element is NaN or Inf.

    ``name`` is included in the message so the failing quantity is obvious in a log.
    """
    arr = np.asarray(value, dtype=np.float64)
    if not np.all(np.isfinite(arr)):
        raise NumericalInstabilityError(
            f"'{name}' contains non-finite values (NaN/Inf): {arr!r}"
        )
    return arr


def ensure_shape(
    name: str, value: NDArray[np.float64], shape: tuple[int, ...]
) -> NDArray[np.float64]:
    """Return ``value`` as a float64 array, raising if its shape differs from ``shape``."""
    arr = np.asarray(value, dtype=np.float64)
    if arr.shape != shape:
        raise ContractViolationError(
            f"'{name}' has shape {arr.shape}, expected {shape}."
        )
    return arr


def ensure_vector(name: str, value: NDArray[np.float64], length: int) -> NDArray[np.float64]:
    """Validate a 1-D vector of the given length and that all elements are finite."""
    arr = ensure_shape(name, value, (length,))
    return ensure_finite(name, arr)


def ensure_in_range(name: str, value: float, low: float, high: float) -> float:
    """Return ``value`` if ``low <= value <= high``; otherwise raise loud."""
    v = float(value)
    if not np.isfinite(v) or v < low or v > high:
        raise ContractViolationError(
            f"'{name}' = {v} is outside the valid range [{low}, {high}]."
        )
    return v


def freeze(arr: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return a read-only float64 copy, so immutable messages cannot be mutated.

    Used by the frozen dataclass contracts in :mod:`common.types` to make their array
    fields tamper-evident.
    """
    out = np.array(arr, dtype=np.float64, copy=True)
    out.setflags(write=False)
    return out
