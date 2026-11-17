"""Explainable right-of-way and conflict-zone safety policy."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .geometry import Vec2
from .models import (
    Crosswalk,
    PedestrianCommand,
    PedestrianDecisionReason,
    PedestrianMode,
    PedestrianState,
    PolicyDecision,
    TrafficPhase,
    VehicleCommand,
    VehicleDecisionReason,
    VehicleState,
    WorldState,
)


@dataclass(frozen=True, slots=True)
class SafetyPolicyConfig:
    """Safety margins and control targets for the first policy version."""

    speed_limit_mps: float = 13.9
    vehicle_approach_horizon_m: float = 30.0
    pedestrian_buffer_m: float = 0.75
    vehicle_buffer_m: float = 1.0
    pedestrian_min_ttc_s: float = 3.0
    min_following_gap_m: float = 6.0
    time_headway_s: float = 1.5

    def __post_init__(self) -> None:
        values = (
            self.speed_limit_mps,
            self.vehicle_approach_horizon_m,
            self.pedestrian_buffer_m,
            self.vehicle_buffer_m,
            self.pedestrian_min_ttc_s,
            self.min_following_gap_m,
            self.time_headway_s,
        )
        if any(value <= 0 or not math.isfinite(value) for value in values):
            raise ValueError("safety policy parameters must be finite and positive")


class SafetyPolicy:
    """Compute safe commands for all actors from one immutable-looking snapshot.

    This policy is intentionally conservative. It uses ground-truth state in
    the deterministic backend; the CARLA adapter will later provide the same
    domain snapshot from simulator actors and sensors.
    """

    def __init__(self, config: SafetyPolicyConfig | None = None) -> None:
        self.config = config or SafetyPolicyConfig()

    def evaluate(self, state: WorldState) -> PolicyDecision:
        """Evaluate every actor without mutating the world."""
        vehicle_commands = {
            vehicle.actor_id: self._vehicle_command(vehicle, state)
            for vehicle in state.vehicles.values()
        }
        pedestrian_commands = {
            pedestrian.actor_id: self._pedestrian_command(pedestrian, state)
            for pedestrian in state.pedestrians.values()
        }
        return PolicyDecision(
            vehicle_commands=vehicle_commands,
            pedestrian_commands=pedestrian_commands,
        )

    def _vehicle_command(self, vehicle: VehicleState, state: WorldState) -> VehicleCommand:
        if vehicle.emergency_stop:
            return VehicleCommand(
                target_speed_mps=0.0,
                should_stop=True,
                reason=VehicleDecisionReason.EMERGENCY_STOP,
            )

        relevant_crosswalks = [
            crosswalk
            for crosswalk in state.crosswalks.values()
            if self._vehicle_relevant_to_crosswalk(vehicle, crosswalk)
        ]
        following_command = self._following_command(vehicle, state)
        if not relevant_crosswalks:
            return following_command or self._proceed_command()

        for crosswalk in relevant_crosswalks:
            if self._vehicle_in_conflict_zone(vehicle, crosswalk):
                continue
            if self._pedestrian_in_conflict_zone(state, crosswalk):
                return VehicleCommand(
                    target_speed_mps=0.0,
                    should_stop=True,
                    reason=VehicleDecisionReason.PEDESTRIAN_IN_CONFLICT_ZONE,
                )
            if self._another_vehicle_in_conflict_zone(vehicle, state, crosswalk):
                return VehicleCommand(
                    target_speed_mps=0.0,
                    should_stop=True,
                    reason=VehicleDecisionReason.CONFLICT_ZONE_UNSAFE,
                )
            signal = state.signals.get(crosswalk.signal_id)
            phase = signal.phase if signal is not None else TrafficPhase.ALL_RED
            if phase is not TrafficPhase.VEHICLE_GREEN:
                return VehicleCommand(
                    target_speed_mps=0.0,
                    should_stop=True,
                    reason=VehicleDecisionReason.SIGNAL_STOP,
                )

        if following_command is not None:
            return following_command
        return self._proceed_command()

    def _pedestrian_command(
        self,
        pedestrian: PedestrianState,
        state: WorldState,
    ) -> PedestrianCommand:
        if pedestrian.mode is PedestrianMode.COMPLETED:
            return PedestrianCommand(
                may_enter_crosswalk=False,
                reason=PedestrianDecisionReason.CROSSING_COMPLETE,
            )
        if pedestrian.crosswalk_id is None:
            return PedestrianCommand(
                may_enter_crosswalk=False,
                reason=PedestrianDecisionReason.WAIT_FOR_SIGNAL,
            )

        crosswalk = state.crosswalks.get(pedestrian.crosswalk_id)
        if crosswalk is None:
            return PedestrianCommand(
                may_enter_crosswalk=False,
                reason=PedestrianDecisionReason.WAIT_FOR_SIGNAL,
            )
        if pedestrian.mode is PedestrianMode.CROSSING:
            return PedestrianCommand(
                may_enter_crosswalk=True,
                reason=PedestrianDecisionReason.WALK_SIGNAL,
            )

        signal = state.signals.get(crosswalk.signal_id)
        phase = signal.phase if signal is not None else TrafficPhase.ALL_RED
        if phase is not TrafficPhase.PEDESTRIAN_WALK:
            return PedestrianCommand(
                may_enter_crosswalk=False,
                reason=PedestrianDecisionReason.WAIT_FOR_SIGNAL,
            )
        if self._vehicle_conflict_for_pedestrians(state, crosswalk):
            return PedestrianCommand(
                may_enter_crosswalk=False,
                reason=PedestrianDecisionReason.VEHICLE_CONFLICT,
            )
        return PedestrianCommand(
            may_enter_crosswalk=True,
            reason=PedestrianDecisionReason.WALK_SIGNAL,
        )

    def _proceed_command(self) -> VehicleCommand:
        return VehicleCommand(
            target_speed_mps=self.config.speed_limit_mps,
            should_stop=False,
            reason=VehicleDecisionReason.CLEAR_TO_PROCEED,
        )

    def _following_command(
        self,
        vehicle: VehicleState,
        state: WorldState,
    ) -> VehicleCommand | None:
        leader = self._nearest_leader(vehicle, state)
        if leader is None:
            return None
        longitudinal_gap = self._longitudinal_gap(vehicle, leader)
        safe_gap = max(
            self.config.min_following_gap_m,
            vehicle.speed_mps * self.config.time_headway_s,
        )
        if longitudinal_gap <= self.config.min_following_gap_m * 0.75:
            return VehicleCommand(
                target_speed_mps=0.0,
                should_stop=True,
                reason=VehicleDecisionReason.FOLLOWING_TRAFFIC,
            )
        if longitudinal_gap < safe_gap:
            target_speed = min(self.config.speed_limit_mps, leader.speed_mps)
            return VehicleCommand(
                target_speed_mps=target_speed,
                should_stop=target_speed == 0,
                reason=VehicleDecisionReason.FOLLOWING_TRAFFIC,
            )
        return None

    def _nearest_leader(
        self,
        vehicle: VehicleState,
        state: WorldState,
    ) -> VehicleState | None:
        lateral_direction = Vec2(-vehicle.heading.y, vehicle.heading.x)
        leaders: list[tuple[float, VehicleState]] = []
        for other in state.vehicles.values():
            if other.actor_id == vehicle.actor_id:
                continue
            relative = other.position - vehicle.position
            longitudinal = relative.dot(vehicle.heading)
            lateral = abs(relative.dot(lateral_direction))
            lane_tolerance = (vehicle.width_m + other.width_m) / 2 + 0.5
            if longitudinal > 0 and lateral <= lane_tolerance:
                leaders.append((longitudinal, other))
        if not leaders:
            return None
        return min(leaders, key=lambda item: item[0])[1]

    @staticmethod
    def _longitudinal_gap(follower: VehicleState, leader: VehicleState) -> float:
        distance = (leader.position - follower.position).dot(follower.heading)
        return distance - (follower.length_m + leader.length_m) / 2

    def _vehicle_relevant_to_crosswalk(
        self,
        vehicle: VehicleState,
        crosswalk: Crosswalk,
    ) -> bool:
        if self._vehicle_in_conflict_zone(vehicle, crosswalk):
            return True
        zone = crosswalk.conflict_zone.expanded(self.config.vehicle_buffer_m)
        direction = vehicle.heading
        to_zone = crosswalk.conflict_zone.center - vehicle.position
        longitudinal_distance = to_zone.dot(direction)
        if longitudinal_distance < -vehicle.length_m:
            return False
        if longitudinal_distance > self.config.vehicle_approach_horizon_m:
            return False
        lateral_direction = Vec2(-direction.y, direction.x)
        lateral_distance = abs(to_zone.dot(lateral_direction))
        lateral_extent = zone.half_extents.x + zone.half_extents.y + vehicle.width_m
        return lateral_distance <= lateral_extent

    def _vehicle_in_conflict_zone(self, vehicle: VehicleState, crosswalk: Crosswalk) -> bool:
        vehicle_box = vehicle.footprint
        zone = crosswalk.conflict_zone.expanded(self.config.vehicle_buffer_m)
        return zone.intersects(vehicle_box)

    def _pedestrian_in_conflict_zone(self, state: WorldState, crosswalk: Crosswalk) -> bool:
        zone = crosswalk.conflict_zone.expanded(self.config.pedestrian_buffer_m)
        return any(
            pedestrian.crosswalk_id == crosswalk.crosswalk_id
            and pedestrian.mode is PedestrianMode.CROSSING
            and zone.contains(pedestrian.position)
            for pedestrian in state.pedestrians.values()
        )

    def _another_vehicle_in_conflict_zone(
        self,
        vehicle: VehicleState,
        state: WorldState,
        crosswalk: Crosswalk,
    ) -> bool:
        return any(
            other.actor_id != vehicle.actor_id
            and self._vehicle_in_conflict_zone(other, crosswalk)
            for other in state.vehicles.values()
        )

    def _vehicle_conflict_for_pedestrians(
        self,
        state: WorldState,
        crosswalk: Crosswalk,
    ) -> bool:
        for vehicle in state.vehicles.values():
            if self._vehicle_in_conflict_zone(vehicle, crosswalk):
                return True
            if not self._vehicle_relevant_to_crosswalk(vehicle, crosswalk):
                continue
            if vehicle.speed_mps <= 0:
                continue
            to_zone = crosswalk.conflict_zone.center - vehicle.position
            longitudinal_distance = max(0.0, to_zone.dot(vehicle.heading))
            ttc = longitudinal_distance / vehicle.speed_mps
            if ttc <= self.config.pedestrian_min_ttc_s:
                return True
        return False

