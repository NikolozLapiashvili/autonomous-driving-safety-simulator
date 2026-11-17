"""Deterministic scenario construction and scalable actor spawning."""

from __future__ import annotations

from dataclasses import dataclass
import random

from .backend import KinematicWorld
from .config import ScenarioConfig
from .geometry import Rect, Vec2
from .models import (
    Crosswalk,
    PedestrianState,
    TrafficPhase,
    TrafficSignalState,
    VehicleState,
    WorldState,
)
from .signals import SignalController


@dataclass(slots=True)
class BuiltScenario:
    """Runtime objects produced from a validated scenario configuration."""

    config: ScenarioConfig
    world: KinematicWorld
    signal_controllers: list[SignalController]


class ScenarioFactory:
    """Build repeatable signalized-crosswalk scenarios."""

    crosswalk_id = "cw-main"
    signal_id = "signal-main"

    def build(self, config: ScenarioConfig) -> BuiltScenario:
        rng = random.Random(config.seed)
        world = KinematicWorld(state=WorldState())
        crosswalk = Crosswalk(
            crosswalk_id=self.crosswalk_id,
            conflict_zone=Rect(center=Vec2(0, 0), half_extents=Vec2(3, 2)),
            crossing_direction=Vec2(0, 1),
            signal_id=self.signal_id,
        )
        world.add_crosswalk(crosswalk)
        world.state.signals[self.signal_id] = TrafficSignalState(
            signal_id=self.signal_id,
            phase=TrafficPhase.VEHICLE_GREEN,
        )

        self._spawn_vehicles(world, config, rng)
        self._spawn_pedestrians(world, config, rng)
        return BuiltScenario(
            config=config,
            world=world,
            signal_controllers=[SignalController(self.signal_id)],
        )

    def _spawn_vehicles(
        self,
        world: KinematicWorld,
        config: ScenarioConfig,
        rng: random.Random,
    ) -> None:
        lane_centers = (-1.5, 1.5)
        for index in range(config.vehicle_count):
            lane_index = index % len(lane_centers)
            queue_index = index // len(lane_centers)
            jitter_m = rng.uniform(-1.0, 1.0)
            vehicle = VehicleState(
                actor_id=f"car-{index:04d}",
                position=Vec2(
                    -20.0 - queue_index * config.vehicle_spacing_m + jitter_m,
                    lane_centers[lane_index],
                ),
                heading=Vec2(1, 0),
                speed_mps=config.vehicle_speed_mps * rng.uniform(0.85, 1.0),
            )
            world.spawn_vehicle(vehicle)

    def _spawn_pedestrians(
        self,
        world: KinematicWorld,
        config: ScenarioConfig,
        rng: random.Random,
    ) -> None:
        for index in range(config.pedestrian_count):
            from_negative_side = index % 2 == 0
            side = -1 if from_negative_side else 1
            heading = Vec2(0, 1) if from_negative_side else Vec2(0, -1)
            crosswalk = world.state.crosswalks[self.crosswalk_id]
            y = side * (crosswalk.conflict_zone.half_extents.y + config.pedestrian_wait_distance_m)
            pedestrian = PedestrianState(
                actor_id=f"ped-{index:04d}",
                position=Vec2(rng.uniform(-1.5, 1.5), y),
                heading=heading,
                crosswalk_id=self.crosswalk_id,
            )
            world.spawn_pedestrian(pedestrian)

