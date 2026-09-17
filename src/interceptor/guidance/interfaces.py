"""Abstract interface owned by the Guidance layer (Role 3)."""

from __future__ import annotations

from abc import ABC, abstractmethod

from interceptor.common.types import AccelerationCommand, TargetStateEstimate


class GuidanceLaw(ABC):
    """Maps a target state estimate to a required acceleration command.

    Owned by Role 3. PN, APN, and OGL all implement this *same* interface so they are
    Liskov-substitutable and KPI comparisons are apples-to-apples (Open/Closed). The
    law requests an *ideal* acceleration; it does NOT clamp to physical limits — that is
    the Command Limiter's job (Role 4).
    """

    @property
    @abstractmethod
    def name(self) -> str:
        """Short identifier (e.g. ``"PN"``, ``"APN"``, ``"OGL"``) for logs/KPI tables."""

    @abstractmethod
    def compute(self, estimate: TargetStateEstimate) -> AccelerationCommand:
        """Return the ideal (unclamped) acceleration command for this estimate."""
