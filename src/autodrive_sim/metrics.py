"""Online metrics for scenario evaluation."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from .events import EventType, SimulationEvent
from .models import PedestrianMode, WorldState


@dataclass(frozen=True, slots=True)
class MetricsReport:
    """Serializable summary of one simulation run."""

    policy_decisions: int
    vehicle_stop_decisions: int
    pedestrian_wait_time_s: float
    signal_phase_changes: int
    safety_violations: int
    collisions: int
    near_misses: int
    lane_invasions: int

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


class MetricsCollector:
    """Consume events and world observations to build a run report."""

    def __init__(self) -> None:
        self._policy_decisions = 0
        self._vehicle_stop_decisions = 0
        self._pedestrian_wait_time_s = 0.0
        self._signal_phase_changes = 0
        self._safety_violations = 0
        self._collisions = 0
        self._near_misses = 0
        self._lane_invasions = 0

    def record(self, event: SimulationEvent) -> None:
        if event.event_type is EventType.POLICY_DECISION:
            self._policy_decisions += 1
            if event.payload.get("actor_kind") == "vehicle" and event.payload.get("should_stop"):
                self._vehicle_stop_decisions += 1
        elif event.event_type is EventType.SIGNAL_PHASE_CHANGED:
            self._signal_phase_changes += 1
        elif event.event_type is EventType.SAFETY_VIOLATION:
            self._safety_violations += 1
            kind = event.payload.get("kind")
            if kind in {"collision", "carla_collision"}:
                self._collisions += 1
            elif kind == "near_miss":
                self._near_misses += 1
            elif kind == "lane_invasion":
                self._lane_invasions += 1

    def observe(self, state: WorldState, delta_s: float) -> None:
        """Accumulate time spent waiting at a crosswalk."""
        if delta_s < 0:
            raise ValueError("observation delta cannot be negative")
        waiting_pedestrians = sum(
            pedestrian.mode is PedestrianMode.WAITING
            for pedestrian in state.pedestrians.values()
        )
        self._pedestrian_wait_time_s += waiting_pedestrians * delta_s

    def report(self) -> MetricsReport:
        return MetricsReport(
            policy_decisions=self._policy_decisions,
            vehicle_stop_decisions=self._vehicle_stop_decisions,
            pedestrian_wait_time_s=self._pedestrian_wait_time_s,
            signal_phase_changes=self._signal_phase_changes,
            safety_violations=self._safety_violations,
            collisions=self._collisions,
            near_misses=self._near_misses,
            lane_invasions=self._lane_invasions,
        )

