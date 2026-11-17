"""Domain entities and commands shared by simulation backends and policies."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from .geometry import Rect, Vec2


class TrafficPhase(StrEnum):
    """Right-of-way phase for a controlled crosswalk."""

    VEHICLE_GREEN = "vehicle_green"
    VEHICLE_YELLOW = "vehicle_yellow"
    ALL_RED = "all_red"
    PEDESTRIAN_WALK = "pedestrian_walk"


class VehicleRole(StrEnum):
    """The role a vehicle plays in a scenario."""

    EGO = "ego"
    BACKGROUND = "background"
    EMERGENCY = "emergency"


class PedestrianMode(StrEnum):
    """High-level pedestrian movement state."""

    WAITING = "waiting"
    CROSSING = "crossing"
    COMPLETED = "completed"


class VehicleDecisionReason(StrEnum):
    """Explainable reasons for longitudinal vehicle commands."""

    CLEAR_TO_PROCEED = "clear_to_proceed"
    SIGNAL_STOP = "signal_stop"
    PEDESTRIAN_IN_CONFLICT_ZONE = "pedestrian_in_conflict_zone"
    CONFLICT_ZONE_UNSAFE = "conflict_zone_unsafe"
    EMERGENCY_STOP = "emergency_stop"


class PedestrianDecisionReason(StrEnum):
    """Explainable reasons for pedestrian crossing commands."""

    WALK_SIGNAL = "walk_signal"
    WAIT_FOR_SIGNAL = "wait_for_signal"
    VEHICLE_CONFLICT = "vehicle_conflict"
    CROSSING_COMPLETE = "crossing_complete"


@dataclass(slots=True)
class VehicleState:
    """Ground-truth vehicle state used by the core and adapters."""

    actor_id: str
    position: Vec2
    heading: Vec2
    speed_mps: float
    role: VehicleRole = VehicleRole.BACKGROUND
    length_m: float = 4.5
    width_m: float = 1.9
    emergency_stop: bool = False

    def __post_init__(self) -> None:
        if self.speed_mps < 0:
            raise ValueError("vehicle speed cannot be negative")
        if self.length_m <= 0 or self.width_m <= 0:
            raise ValueError("vehicle dimensions must be positive")
        self.heading = self.heading.normalized()

    @property
    def footprint(self) -> Rect:
        """Return an axis-aligned approximation for collision checks."""
        return Rect(
            center=self.position,
            half_extents=Vec2(self.length_m / 2, self.width_m / 2),
        )


@dataclass(slots=True)
class PedestrianState:
    """Ground-truth pedestrian state used by the core and adapters."""

    actor_id: str
    position: Vec2
    heading: Vec2
    speed_mps: float = 1.4
    mode: PedestrianMode = PedestrianMode.WAITING
    crosswalk_id: str | None = None
    radius_m: float = 0.3

    def __post_init__(self) -> None:
        if self.speed_mps < 0:
            raise ValueError("pedestrian speed cannot be negative")
        if self.radius_m <= 0:
            raise ValueError("pedestrian radius must be positive")
        self.heading = self.heading.normalized()

    @property
    def footprint(self) -> Rect:
        return Rect(
            center=self.position,
            half_extents=Vec2(self.radius_m, self.radius_m),
        )


@dataclass(frozen=True, slots=True)
class Crosswalk:
    """A pedestrian crossing and its vehicle conflict zone."""

    crosswalk_id: str
    conflict_zone: Rect
    crossing_direction: Vec2
    signal_id: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "crossing_direction", self.crossing_direction.normalized())


@dataclass(slots=True)
class TrafficSignalState:
    """Current signal phase and elapsed time."""

    signal_id: str
    phase: TrafficPhase = TrafficPhase.VEHICLE_GREEN
    elapsed_s: float = 0.0


@dataclass(frozen=True, slots=True)
class VehicleCommand:
    """A policy output for one vehicle."""

    target_speed_mps: float
    should_stop: bool
    reason: VehicleDecisionReason

    def __post_init__(self) -> None:
        if self.target_speed_mps < 0:
            raise ValueError("target speed cannot be negative")


@dataclass(frozen=True, slots=True)
class PedestrianCommand:
    """A policy output for one pedestrian."""

    may_enter_crosswalk: bool
    reason: PedestrianDecisionReason


@dataclass(frozen=True, slots=True)
class PolicyDecision:
    """All commands produced for one world snapshot."""

    vehicle_commands: dict[str, VehicleCommand] = field(default_factory=dict)
    pedestrian_commands: dict[str, PedestrianCommand] = field(default_factory=dict)


@dataclass(slots=True)
class WorldState:
    """A backend-independent snapshot of the simulated world."""

    time_s: float = 0.0
    vehicles: dict[str, VehicleState] = field(default_factory=dict)
    pedestrians: dict[str, PedestrianState] = field(default_factory=dict)
    crosswalks: dict[str, Crosswalk] = field(default_factory=dict)
    signals: dict[str, TrafficSignalState] = field(default_factory=dict)

