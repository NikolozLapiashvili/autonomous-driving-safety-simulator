"""Backend contracts and a deterministic kinematic world for fast tests."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

from .models import (
    Crosswalk,
    PedestrianMode,
    PedestrianState,
    PolicyDecision,
    VehicleState,
    WorldState,
)


class WorldBackend(Protocol):
    """Interface implemented by deterministic and CARLA-backed worlds."""

    def snapshot(self) -> WorldState:
        """Return the current backend-independent world state."""

    def step(self, delta_s: float, decision: PolicyDecision) -> None:
        """Apply commands and advance the world by one fixed step."""


@dataclass(slots=True)
class KinematicWorld:
    """A small deterministic world with point-mass actor movement.

    This backend is not a replacement for a physics engine. It exists to make
    traffic policy behavior deterministic, inexpensive, and testable in CI.
    """

    state: WorldState = field(default_factory=WorldState)
    acceleration_mps2: float = 2.5
    braking_mps2: float = 6.0

    def __post_init__(self) -> None:
        if self.acceleration_mps2 <= 0 or self.braking_mps2 <= 0:
            raise ValueError("kinematic acceleration and braking must be positive")

    def snapshot(self) -> WorldState:
        """Return the live state for the in-process deterministic backend."""
        return self.state

    def add_crosswalk(self, crosswalk: Crosswalk) -> None:
        if crosswalk.crosswalk_id in self.state.crosswalks:
            raise ValueError(f"crosswalk already exists: {crosswalk.crosswalk_id}")
        self.state.crosswalks[crosswalk.crosswalk_id] = crosswalk

    def spawn_vehicle(self, vehicle: VehicleState) -> None:
        if vehicle.actor_id in self.state.vehicles:
            raise ValueError(f"vehicle already exists: {vehicle.actor_id}")
        self.state.vehicles[vehicle.actor_id] = vehicle

    def spawn_pedestrian(self, pedestrian: PedestrianState) -> None:
        if pedestrian.actor_id in self.state.pedestrians:
            raise ValueError(f"pedestrian already exists: {pedestrian.actor_id}")
        self.state.pedestrians[pedestrian.actor_id] = pedestrian

    def step(self, delta_s: float, decision: PolicyDecision) -> None:
        """Apply commands and advance all actors using semi-implicit Euler."""
        if delta_s <= 0:
            raise ValueError("backend step delta must be positive")
        self._step_vehicles(delta_s, decision)
        self._step_pedestrians(delta_s, decision)
        self.state.time_s += delta_s

    def _step_vehicles(self, delta_s: float, decision: PolicyDecision) -> None:
        for vehicle in self.state.vehicles.values():
            command = decision.vehicle_commands.get(vehicle.actor_id)
            if command is None:
                continue
            vehicle.speed_mps = _move_speed(
                current=vehicle.speed_mps,
                target=command.target_speed_mps,
                delta_s=delta_s,
                acceleration=self.acceleration_mps2,
                braking=self.braking_mps2,
            )
            vehicle.position = vehicle.position + vehicle.heading * (vehicle.speed_mps * delta_s)

    def _step_pedestrians(self, delta_s: float, decision: PolicyDecision) -> None:
        for pedestrian in self.state.pedestrians.values():
            command = decision.pedestrian_commands.get(pedestrian.actor_id)
            if command is None:
                continue
            if pedestrian.mode is PedestrianMode.WAITING and command.may_enter_crosswalk:
                pedestrian.mode = PedestrianMode.CROSSING
            if pedestrian.mode is not PedestrianMode.CROSSING:
                continue
            was_inside_zone = False
            if pedestrian.crosswalk_id is not None:
                crosswalk = self.state.crosswalks[pedestrian.crosswalk_id]
                was_inside_zone = crosswalk.conflict_zone.expanded(0.5).contains(
                    pedestrian.position
                )
            pedestrian.position = pedestrian.position + pedestrian.heading * (
                pedestrian.speed_mps * delta_s
            )
            if pedestrian.crosswalk_id is None:
                continue
            crosswalk = self.state.crosswalks[pedestrian.crosswalk_id]
            is_inside_zone = crosswalk.conflict_zone.expanded(0.5).contains(pedestrian.position)
            if was_inside_zone and not is_inside_zone:
                pedestrian.mode = PedestrianMode.COMPLETED


def _move_speed(
    *,
    current: float,
    target: float,
    delta_s: float,
    acceleration: float,
    braking: float,
) -> float:
    """Move speed toward a target without overshooting it."""
    if current < 0 or target < 0:
        raise ValueError("speeds cannot be negative")
    difference = target - current
    if difference == 0:
        return current
    rate = acceleration if difference > 0 else braking
    maximum_change = rate * delta_s
    if abs(difference) <= maximum_change:
        return target
    return current + maximum_change if difference > 0 else current - maximum_change

