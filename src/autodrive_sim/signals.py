"""Deterministic traffic-signal phase controller."""

from __future__ import annotations

from dataclasses import dataclass, field

from .models import TrafficPhase, TrafficSignalState, WorldState


PHASE_ORDER: tuple[TrafficPhase, ...] = (
    TrafficPhase.VEHICLE_GREEN,
    TrafficPhase.VEHICLE_YELLOW,
    TrafficPhase.ALL_RED,
    TrafficPhase.PEDESTRIAN_WALK,
    TrafficPhase.ALL_RED,
)


@dataclass(frozen=True, slots=True)
class SignalTiming:
    """Duration of each signal phase in seconds."""

    vehicle_green_s: float = 20.0
    vehicle_yellow_s: float = 3.0
    all_red_s: float = 1.0
    pedestrian_walk_s: float = 8.0

    def __post_init__(self) -> None:
        if any(
            duration <= 0
            for duration in (
                self.vehicle_green_s,
                self.vehicle_yellow_s,
                self.all_red_s,
                self.pedestrian_walk_s,
            )
        ):
            raise ValueError("all signal phase durations must be positive")

    def duration_for(self, phase: TrafficPhase) -> float:
        if phase is TrafficPhase.VEHICLE_GREEN:
            return self.vehicle_green_s
        if phase is TrafficPhase.VEHICLE_YELLOW:
            return self.vehicle_yellow_s
        if phase is TrafficPhase.PEDESTRIAN_WALK:
            return self.pedestrian_walk_s
        return self.all_red_s


@dataclass(slots=True)
class SignalController:
    """Advance one signal through a fixed, safe phase sequence."""

    signal_id: str
    timing: SignalTiming = field(default_factory=SignalTiming)

    def tick(self, state: WorldState, delta_s: float) -> TrafficSignalState:
        """Advance the owned signal, handling large deltas without drift."""
        if delta_s <= 0:
            raise ValueError("signal tick delta must be positive")
        signal = state.signals.get(self.signal_id)
        if signal is None:
            raise KeyError(f"signal does not exist: {self.signal_id}")

        signal.elapsed_s += delta_s
        while signal.elapsed_s >= self.timing.duration_for(signal.phase):
            signal.elapsed_s -= self.timing.duration_for(signal.phase)
            signal.phase = self._next_phase(signal.phase)
        return signal

    @staticmethod
    def _next_phase(phase: TrafficPhase) -> TrafficPhase:
        try:
            index = PHASE_ORDER.index(phase)
        except ValueError as error:
            raise ValueError(f"unsupported signal phase: {phase}") from error
        return PHASE_ORDER[(index + 1) % len(PHASE_ORDER)]

