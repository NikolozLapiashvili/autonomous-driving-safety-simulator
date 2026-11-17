"""Post-step safety checks for collisions and near misses."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .events import EventType, SimulationEvent
from .geometry import Rect
from .models import WorldState


@dataclass(frozen=True, slots=True)
class SafetyMonitorConfig:
    """Thresholds used by the post-step monitor."""

    near_miss_distance_m: float = 1.0
    include_pedestrian_pedestrian: bool = False

    def __post_init__(self) -> None:
        if self.near_miss_distance_m <= 0 or not math.isfinite(self.near_miss_distance_m):
            raise ValueError("near-miss distance must be finite and positive")


class SafetyMonitor:
    """Find vehicle-related conflicts in a backend-independent world snapshot.

    Pedestrian-to-pedestrian overlap is not an autonomous-vehicle safety
    violation, so it is excluded by default. It can be enabled for crowd
    simulation experiments with ``include_pedestrian_pedestrian=True``.
    """

    def __init__(self, config: SafetyMonitorConfig | None = None) -> None:
        self.config = config or SafetyMonitorConfig()
        self._active_conflicts: dict[tuple[str, str], str] = {}

    def inspect(self, state: WorldState) -> list[SimulationEvent]:
        """Return one event for each collision or near-miss pair."""
        actors = [
            (actor_id, vehicle.footprint)
            for actor_id, vehicle in state.vehicles.items()
        ] + [
            (actor_id, pedestrian.footprint)
            for actor_id, pedestrian in state.pedestrians.items()
        ]
        events: list[SimulationEvent] = []
        current_conflicts: dict[tuple[str, str], str] = {}
        vehicle_ids = set(state.vehicles)
        for index, (first_id, first_box) in enumerate(actors):
            for second_id, second_box in actors[index + 1 :]:
                if (
                    not self.config.include_pedestrian_pedestrian
                    and first_id not in vehicle_ids
                    and second_id not in vehicle_ids
                ):
                    continue
                pair = (first_id, second_id)
                if first_box.intersects(second_box):
                    kind = "collision"
                elif (
                    self._distance_between(first_box, second_box)
                    <= self.config.near_miss_distance_m
                ):
                    kind = "near_miss"
                else:
                    continue
                current_conflicts[pair] = kind
                if self._active_conflicts.get(pair) != kind:
                    events.append(self._event(state.time_s, kind, first_id, second_id))
        self._active_conflicts = current_conflicts
        return events

    @staticmethod
    def _distance_between(first: Rect, second: Rect) -> float:
        """Return the Euclidean gap between two axis-aligned rectangles."""
        dx = max(
            abs(first.center.x - second.center.x)
            - first.half_extents.x
            - second.half_extents.x,
            0.0,
        )
        dy = max(
            abs(first.center.y - second.center.y)
            - first.half_extents.y
            - second.half_extents.y,
            0.0,
        )
        return math.hypot(dx, dy)

    @staticmethod
    def _event(time_s: float, kind: str, first_id: str, second_id: str) -> SimulationEvent:
        return SimulationEvent(
            time_s=time_s,
            event_type=EventType.SAFETY_VIOLATION,
            payload={"kind": kind, "actor_a": first_id, "actor_b": second_id},
        )

