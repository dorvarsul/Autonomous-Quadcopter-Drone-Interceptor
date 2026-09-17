"""Pass-through Guidance stub for the skeleton loop.

NOT a guidance law. Requests zero acceleration regardless of the estimate, so the loop
is fully deterministic and quiescent. OGL provides the real behavior behind the same
:class:`GuidanceLaw` interface.
"""

from __future__ import annotations

import numpy as np

from interceptor.common.types import AccelerationCommand, TargetStateEstimate
from interceptor.guidance.interfaces import GuidanceLaw


class ZeroGuidance(GuidanceLaw):
    """Always commands zero acceleration. Placeholder for PN/APN/OGL."""

    @property
    def name(self) -> str:
        return "ZERO_STUB"

    def compute(self, estimate: TargetStateEstimate) -> AccelerationCommand:
        return AccelerationCommand(acceleration_m_s2=np.zeros(3, dtype=np.float64))
