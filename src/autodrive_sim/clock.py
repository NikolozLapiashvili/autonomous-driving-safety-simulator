"""Deterministic fixed-step simulation clock."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class SimulationClock:
    """Advance simulation time in reproducible fixed-size steps."""

    fixed_delta_s: float = 0.05
    time_s: float = 0.0
    frame: int = 0

    def __post_init__(self) -> None:
        if self.fixed_delta_s <= 0:
            raise ValueError("fixed simulation delta must be positive")
        if self.time_s < 0 or self.frame < 0:
            raise ValueError("clock state cannot be negative")

    def tick(self, steps: int = 1) -> float:
        """Advance by ``steps`` fixed intervals and return the new time."""
        if steps < 1:
            raise ValueError("tick steps must be at least one")
        self.frame += steps
        self.time_s += steps * self.fixed_delta_s
        return self.time_s

    def reset(self) -> None:
        """Return the clock to frame zero."""
        self.time_s = 0.0
        self.frame = 0

