"""Optional CARLA 3D backend.

The core package deliberately imports without CARLA installed. This module
loads the CARLA Python API only when a connection is requested.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import importlib
import math
from typing import Any

from .models import (
    PedestrianMode,
    PedestrianState,
    PolicyDecision,
    TrafficPhase,
    VehicleState,
    WorldState,
)


class CarlaUnavailableError(RuntimeError):
    """Raised when the optional CARLA Python client is not installed."""


class CarlaSpawnError(RuntimeError):
    """Raised when CARLA cannot create a requested actor."""


@dataclass(frozen=True, slots=True)
class CarlaConnectionConfig:
    """Connection and stepping settings for a CARLA server."""

    host: str = "127.0.0.1"
    port: int = 2000
    timeout_s: float = 10.0
    synchronous_mode: bool = True
    fixed_delta_s: float = 0.05

    def __post_init__(self) -> None:
        if not self.host.strip():
            raise ValueError("CARLA host cannot be empty")
        if not 1 <= self.port <= 65_535:
            raise ValueError("CARLA port must be between 1 and 65535")
        if self.timeout_s <= 0 or self.fixed_delta_s <= 0:
            raise ValueError("CARLA timeout and fixed delta must be positive")


@dataclass(slots=True)
class CarlaBackend:
    """Translate backend-independent actor commands to CARLA actors."""

    client: Any
    carla_world: Any
    config: CarlaConnectionConfig
    state: WorldState = field(default_factory=WorldState)
    _carla: Any = field(default=None, repr=False)
    _compact_traffic: bool = field(default=False, repr=False)
    _traffic_manager: Any = field(default=None, repr=False)
    _traffic_manager_port: int = field(default=8000, repr=False)
    _vehicle_actors: dict[str, Any] = field(default_factory=dict, repr=False)
    _vehicle_spawn_points: list[Any] = field(default_factory=list, repr=False)
    _used_vehicle_spawn_points: set[int] = field(default_factory=set, repr=False)
    _pedestrian_actors: dict[str, Any] = field(default_factory=dict, repr=False)
    _pedestrian_controllers: dict[str, Any] = field(default_factory=dict, repr=False)
    _pedestrian_targets: dict[str, Any] = field(default_factory=dict, repr=False)
    _pedestrian_routes: list[tuple[Any, Any]] = field(default_factory=list, repr=False)
    _pedestrian_route_cursor: int = field(default=0, repr=False)
    _pedestrian_route_by_id: dict[str, tuple[Any, Any]] = field(default_factory=dict, repr=False)
    _active_pedestrians: set[str] = field(default_factory=set, repr=False)
    _relocating_pedestrians: set[str] = field(default_factory=set, repr=False)
    _traffic_lights: dict[str, list[Any]] = field(default_factory=dict, repr=False)
    _spectator_follow_id: str | None = field(default=None, repr=False)

    @classmethod
    def connect(
        cls,
        config: CarlaConnectionConfig | None = None,
        *,
        compact_traffic: bool = False,
        vehicle_count: int = 0,
        traffic_manager: bool = False,
        traffic_manager_port: int = 8000,
        traffic_manager_seed: int = 42,
    ) -> CarlaBackend:
        """Connect to a running CARLA server and configure stepping."""
        connection = config or CarlaConnectionConfig()
        if not 1 <= traffic_manager_port <= 65_535:
            raise ValueError("Traffic Manager port must be between 1 and 65535")
        try:
            carla = importlib.import_module("carla")
        except ModuleNotFoundError as error:
            raise CarlaUnavailableError(
                "CARLA is not installed; install the client matching the server release"
            ) from error
        client = carla.Client(connection.host, connection.port)
        client.set_timeout(connection.timeout_s)
        world = client.get_world()
        manager = None
        if traffic_manager:
            manager = client.get_trafficmanager(traffic_manager_port)
        backend = cls(
            client,
            world,
            connection,
            _carla=carla,
            _compact_traffic=compact_traffic,
            _traffic_manager=manager,
            _traffic_manager_port=traffic_manager_port,
        )
        backend._vehicle_spawn_points = list(world.get_map().get_spawn_points())
        if compact_traffic:
            backend._vehicle_spawn_points = backend._select_compact_spawn_points(
                backend._vehicle_spawn_points,
                max(vehicle_count, 1),
            )
        backend._configure_world()
        if hasattr(world, "set_pedestrians_seed"):
            world.set_pedestrians_seed(traffic_manager_seed)
        if hasattr(world, "set_pedestrians_cross_factor"):
            world.set_pedestrians_cross_factor(0.0)
        if manager is not None:
            backend._configure_traffic_manager(traffic_manager_seed)
            backend._restore_native_traffic_lights()
            backend._prepare_pedestrian_routes()
        return backend

    def snapshot(self) -> WorldState:
        """Refresh and return the domain snapshot from live CARLA actors."""
        self._refresh_state()
        return self.state

    def spawn_vehicle(self, vehicle: VehicleState) -> None:
        """Spawn a drivable CARLA vehicle at the requested domain pose."""
        if vehicle.actor_id in self._vehicle_actors:
            raise ValueError(f"vehicle already exists: {vehicle.actor_id}")
        blueprint_library = self.carla_world.get_blueprint_library()
        blueprints = blueprint_library.filter("vehicle.tesla.model3") or blueprint_library.filter(
            "vehicle.*"
        )
        if not blueprints:
            raise CarlaSpawnError("CARLA has no vehicle blueprints")
        blueprint = blueprints[0]
        actor = self._try_spawn_vehicle_from_map(blueprint)
        if actor is None:
            raise CarlaSpawnError(
                f"CARLA rejected vehicle spawn: {vehicle.actor_id}; "
                "no free drivable map spawn point was available"
            )
        self._vehicle_actors[vehicle.actor_id] = actor
        self.state.vehicles[vehicle.actor_id] = vehicle
        self._configure_traffic_manager_vehicle(actor)
        self._update_vehicle_state(vehicle)
        if self._spectator_follow_id is None:
            self._spectator_follow_id = vehicle.actor_id
            self._update_spectator()

    def spawn_pedestrian(self, pedestrian: PedestrianState) -> None:
        """Spawn a walker and its AI controller in CARLA."""
        if pedestrian.actor_id in self._pedestrian_actors:
            raise ValueError(f"pedestrian already exists: {pedestrian.actor_id}")
        blueprint_library = self.carla_world.get_blueprint_library()
        blueprints = blueprint_library.filter("walker.pedestrian.*")
        if not blueprints:
            raise CarlaSpawnError("CARLA has no pedestrian blueprints")
        preferred_location, preferred_target = self._next_pedestrian_route()
        walker = self._try_spawn_walker_from_navigation(
            blueprints[0],
            pedestrian.heading,
            preferred_location,
        )
        if walker is None:
            raise CarlaSpawnError(
                f"CARLA rejected pedestrian spawn: {pedestrian.actor_id}; "
                "no free sidewalk navigation location was available"
            )
        controller_blueprint = blueprint_library.find("controller.ai.walker")
        controller = self.carla_world.spawn_actor(
            controller_blueprint,
            self._carla.Transform(),
            attach_to=walker,
        )
        if controller is None:
            walker.destroy()
            raise CarlaSpawnError(f"CARLA rejected walker controller: {pedestrian.actor_id}")
        used_crosswalk_route = (
            preferred_location is not None
            and preferred_target is not None
            and self._location_distance(walker.get_location(), preferred_location) <= 3.0
        )
        target = preferred_target if used_crosswalk_route else None
        if target is None:
            target = self._select_pedestrian_target(walker)
        controller.set_max_speed(pedestrian.speed_mps)
        controller.go_to_location(target)
        controller.start()
        controller.stop()
        self._pedestrian_actors[pedestrian.actor_id] = walker
        self._pedestrian_controllers[pedestrian.actor_id] = controller
        self._pedestrian_targets[pedestrian.actor_id] = target
        if used_crosswalk_route and preferred_location is not None and preferred_target is not None:
            self._pedestrian_route_by_id[pedestrian.actor_id] = (
                preferred_location,
                preferred_target,
            )
        self.state.pedestrians[pedestrian.actor_id] = pedestrian
        self._update_pedestrian_state(pedestrian)

    def _prepare_pedestrian_routes(self) -> None:
        """Build sidewalk-to-sidewalk routes through real CARLA crosswalks."""
        get_crosswalks = getattr(self.carla_world.get_map(), "get_crosswalks", None)
        if not callable(get_crosswalks):
            return
        try:
            raw_points = list(get_crosswalks())
        except RuntimeError:
            return
        polygons = self._split_crosswalk_points(raw_points)
        if not polygons:
            return

        if self._vehicle_spawn_points:
            center_x = sum(point.location.x for point in self._vehicle_spawn_points) / len(
                self._vehicle_spawn_points
            )
            center_y = sum(point.location.y for point in self._vehicle_spawn_points) / len(
                self._vehicle_spawn_points
            )
            center = self._carla.Location(x=center_x, y=center_y, z=0.0)
            polygons.sort(
                key=lambda polygon: self._location_distance(
                    self._polygon_center(polygon), center
                )
            )

        for polygon in polygons[:12]:
            route = self._build_crosswalk_route(polygon)
            if route is not None:
                self._pedestrian_routes.append(route)

    @classmethod
    def _split_crosswalk_points(cls, points: list[Any]) -> list[list[Any]]:
        polygons: list[list[Any]] = []
        current: list[Any] = []
        for point in points:
            if not current:
                current.append(point)
                continue
            current.append(point)
            if len(current) >= 4 and cls._location_distance(point, current[0]) <= 0.05:
                polygons.append(current[:-1])
                current = []
        if len(current) >= 3:
            polygons.append(current)
        return polygons

    @classmethod
    def _polygon_center(cls, polygon: list[Any]) -> Any:
        return type(polygon[0])(
            x=sum(point.x for point in polygon) / len(polygon),
            y=sum(point.y for point in polygon) / len(polygon),
            z=sum(point.z for point in polygon) / len(polygon),
        )

    def _build_crosswalk_route(self, polygon: list[Any]) -> tuple[Any, Any] | None:
        center = self._polygon_center(polygon)
        edges = [
            (polygon[index - 1], polygon[index])
            for index in range(len(polygon))
        ]
        long_edge = max(
            edges,
            key=lambda edge: self._location_distance(edge[0], edge[1]),
        )
        edge_dx = long_edge[1].x - long_edge[0].x
        edge_dy = long_edge[1].y - long_edge[0].y
        edge_length = math.hypot(edge_dx, edge_dy)
        if edge_length < 1.0:
            return None
        axis_x = edge_dx / edge_length
        axis_y = edge_dy / edge_length
        half_length = max(
            abs((point.x - center.x) * axis_x + (point.y - center.y) * axis_y)
            for point in polygon
        )
        endpoint_a = self._carla.Location(
            x=center.x - axis_x * (half_length + 3.0),
            y=center.y - axis_y * (half_length + 3.0),
            z=center.z,
        )
        endpoint_b = self._carla.Location(
            x=center.x + axis_x * (half_length + 3.0),
            y=center.y + axis_y * (half_length + 3.0),
            z=center.z,
        )
        samples = [
            self.carla_world.get_random_location_from_navigation()
            for _ in range(120)
        ]
        samples = [sample for sample in samples if sample is not None]
        if not samples:
            return None
        start = min(samples, key=lambda sample: self._location_distance(sample, endpoint_a))
        target = min(samples, key=lambda sample: self._location_distance(sample, endpoint_b))
        if (
            self._location_distance(start, endpoint_a) > 18.0
            or self._location_distance(target, endpoint_b) > 18.0
            or self._location_distance(start, target) < 6.0
        ):
            return None
        return start, target

    def _next_pedestrian_route(self) -> tuple[Any | None, Any | None]:
        if not self._pedestrian_routes:
            return None, None
        route = self._pedestrian_routes[
            self._pedestrian_route_cursor % len(self._pedestrian_routes)
        ]
        direction = self._pedestrian_route_cursor % 2
        self._pedestrian_route_cursor += 1
        if direction == 0:
            return route
        return route[1], route[0]

    def _next_route_from_location(self, location: Any) -> tuple[Any, Any] | None:
        if not self._pedestrian_routes:
            return None
        for offset in range(len(self._pedestrian_routes)):
            index = (self._pedestrian_route_cursor + offset) % len(self._pedestrian_routes)
            route = self._pedestrian_routes[index]
            if self._location_distance(location, route[0]) <= self._location_distance(
                location, route[1]
            ):
                start, target = route
            else:
                start, target = route[1], route[0]
            if self._location_distance(location, start) > 2.0 or len(self._pedestrian_routes) == 1:
                self._pedestrian_route_cursor = index + 1
                return start, target
        return None

    def _select_pedestrian_target(self, walker: Any) -> Any:
        """Choose a forward navigation target instead of an arbitrary curb point."""
        origin = walker.get_location()
        transform = walker.get_transform()
        forward = transform.get_forward_vector()
        fallback = None
        for _ in range(40):
            target = self.carla_world.get_random_location_from_navigation()
            if target is None:
                continue
            dx = target.x - origin.x
            dy = target.y - origin.y
            distance = math.hypot(dx, dy)
            if fallback is None and distance >= 8.0:
                fallback = target
            if distance < 8.0:
                continue
            forward_projection = (dx * forward.x + dy * forward.y) / distance
            if forward_projection >= 0.2:
                return target
        if fallback is not None:
            return fallback
        return origin

    def _apply_pedestrian_commands(self, decision: PolicyDecision) -> None:
        """Hold walkers until the policy grants them permission to move."""
        for actor_id, controller in self._pedestrian_controllers.items():
            if not controller.is_alive:
                continue
            if actor_id in self._relocating_pedestrians:
                if actor_id not in self._active_pedestrians:
                    controller.start()
                    self._active_pedestrians.add(actor_id)
                target = self._pedestrian_targets.get(actor_id)
                if target is not None:
                    controller.go_to_location(target)
                continue
            command = decision.pedestrian_commands.get(actor_id)
            state = self.state.pedestrians.get(actor_id)
            already_crossing = (
                state is not None and state.mode is PedestrianMode.CROSSING
            )
            native_walk_signal = self._pedestrian_native_walk_allowed(actor_id)
            may_move = already_crossing or (
                command is not None
                and command.may_enter_crosswalk
                and native_walk_signal
            )
            if may_move and actor_id not in self._active_pedestrians:
                controller.start()
                target = self._pedestrian_targets.get(actor_id)
                if target is not None:
                    controller.go_to_location(target)
                self._active_pedestrians.add(actor_id)
            elif not may_move and actor_id in self._active_pedestrians:
                controller.stop()
                self._active_pedestrians.remove(actor_id)

    def _pedestrian_native_walk_allowed(self, actor_id: str) -> bool:
        """Use the actual nearby CARLA signal before starting a crossing."""
        if self._traffic_manager is None:
            return True
        actor = self._pedestrian_actors.get(actor_id)
        if actor is None or not actor.is_alive:
            return False
        get_waypoint = getattr(self.carla_world.get_map(), "get_waypoint", None)
        get_lights = getattr(
            self.carla_world.get_map(),
            "get_traffic_lights_from_waypoint",
            None,
        )
        if not callable(get_waypoint) or not callable(get_lights):
            return True
        try:
            waypoint = get_waypoint(
                actor.get_location(),
                project_to_road=True,
                lane_type=self._carla.LaneType.Driving,
            )
            if waypoint is None:
                return True
            lights = get_lights(waypoint, 30.0)
        except RuntimeError:
            return True
        if not lights:
            return True
        return any(
            light.get_state() == self._carla.TrafficLightState.Red
            for light in lights
            if light.is_alive
        )

    def step(self, delta_s: float, decision: PolicyDecision) -> None:
        """Apply policy commands and advance the CARLA world one frame."""
        if delta_s <= 0:
            raise ValueError("CARLA step delta must be positive")
        self._apply_pedestrian_commands(decision)
        if self._traffic_manager is None:
            emergency_brakes = self._imminent_vehicle_conflicts()
            for actor_id, command in decision.vehicle_commands.items():
                actor = self._vehicle_actors.get(actor_id)
                if actor is None or not actor.is_alive:
                    continue
                speed_mps = self._actor_speed_mps(actor)
                should_stop = (
                    command.should_stop
                    or actor_id in emergency_brakes
                    or self._traffic_light_requires_stop(actor)
                )
                target_speed_mps = (
                    0.0
                    if should_stop
                    else self._safe_target_speed(actor, command.target_speed_mps)
                )
                speed_error = target_speed_mps - speed_mps
                throttle = max(0.0, min(1.0, speed_error / 5.0))
                brake = 1.0 if should_stop else max(0.0, min(1.0, -speed_error / 5.0))
                steer = self._lane_following_steer(actor)
                actor.apply_control(
                    self._carla.VehicleControl(
                        throttle=throttle,
                        steer=steer,
                        brake=brake,
                    )
                )
        if self.config.synchronous_mode:
            self.carla_world.tick()
        else:
            self.carla_world.wait_for_tick()
        self.state.time_s += delta_s
        self._refresh_state()
        self._update_spectator()

    def sync_signals(self) -> None:
        """Apply domain phases only in manual mode.

        Traffic Manager mode leaves CARLA's native signal groups in charge so
        intersecting approaches do not all become green at once.
        """
        if self._traffic_manager is not None:
            return
        for signal_id, signal in self.state.signals.items():
            lights = self._traffic_lights.setdefault(signal_id, self._discover_lights())
            for light in lights:
                if not light.is_alive:
                    continue
                light.set_state(self._carla_signal_state(signal.phase))

    def close(self) -> None:
        """Stop controllers and destroy every actor created by this backend."""
        for controller in self._pedestrian_controllers.values():
            if controller.is_alive:
                controller.stop()
        actors = [*self._vehicle_actors.values(), *self._pedestrian_actors.values()]
        actors.extend(self._pedestrian_controllers.values())
        if actors:
            commands = [self._carla.command.DestroyActor(actor.id) for actor in actors]
            self.client.apply_batch_sync(commands, self.config.synchronous_mode)
        self._vehicle_actors.clear()
        self._pedestrian_actors.clear()
        self._pedestrian_controllers.clear()
        self._pedestrian_targets.clear()
        self._active_pedestrians.clear()

    def _configure_world(self) -> None:
        settings = self.carla_world.get_settings()
        settings.synchronous_mode = self.config.synchronous_mode
        settings.fixed_delta_seconds = (
            self.config.fixed_delta_s if self.config.synchronous_mode else None
        )
        self.carla_world.apply_settings(settings)

    def _configure_traffic_manager(self, seed: int) -> None:
        if self._traffic_manager is None:
            return
        self._traffic_manager.set_synchronous_mode(self.config.synchronous_mode)
        if hasattr(self._traffic_manager, "set_random_device_seed"):
            self._traffic_manager.set_random_device_seed(seed)
        if hasattr(self._traffic_manager, "global_percentage_speed_difference"):
            self._traffic_manager.global_percentage_speed_difference(0.0)
        if hasattr(self._traffic_manager, "set_global_distance_to_leading_vehicle"):
            self._traffic_manager.set_global_distance_to_leading_vehicle(5.0)

    def _configure_traffic_manager_vehicle(self, actor: Any) -> None:
        if self._traffic_manager is None:
            return
        actor.set_autopilot(True, self._traffic_manager_port)
        manager = self._traffic_manager
        if hasattr(manager, "auto_lane_change"):
            manager.auto_lane_change(actor, False)
        if hasattr(manager, "distance_to_leading_vehicle"):
            manager.distance_to_leading_vehicle(actor, 5.0)
        if hasattr(manager, "vehicle_percentage_speed_difference"):
            manager.vehicle_percentage_speed_difference(actor, 0.0)
        if hasattr(manager, "ignore_lights_percentage"):
            manager.ignore_lights_percentage(actor, 0.0)
        if hasattr(manager, "ignore_signs_percentage"):
            manager.ignore_signs_percentage(actor, 0.0)
        if hasattr(manager, "ignore_vehicles_percentage"):
            manager.ignore_vehicles_percentage(actor, 0.0)
        if hasattr(manager, "ignore_walkers_percentage"):
            manager.ignore_walkers_percentage(actor, 0.0)
        if hasattr(manager, "collision_detection"):
            for other in self._vehicle_actors.values():
                if other is not actor and other.is_alive:
                    manager.collision_detection(actor, other, True)

    def _restore_native_traffic_lights(self) -> None:
        """Release any lights previously held by a manual experiment."""
        for light in self._discover_lights():
            if light.is_alive and hasattr(light, "freeze"):
                light.freeze(False)

    def _discover_lights(self) -> list[Any]:
        return list(self.carla_world.get_actors().filter("traffic.traffic_light"))

    def _carla_signal_state(self, phase: TrafficPhase) -> Any:
        if phase is TrafficPhase.VEHICLE_GREEN:
            return self._carla.TrafficLightState.Green
        if phase is TrafficPhase.VEHICLE_YELLOW:
            return self._carla.TrafficLightState.Yellow
        return self._carla.TrafficLightState.Red

    def _refresh_state(self) -> None:
        for actor_id, actor in self._vehicle_actors.items():
            if not actor.is_alive:
                continue
            state = self.state.vehicles[actor_id]
            self._update_vehicle_state(state)
        for actor_id, actor in self._pedestrian_actors.items():
            if not actor.is_alive:
                continue
            state = self.state.pedestrians[actor_id]
            self._update_pedestrian_state(state)
            target = self._pedestrian_targets.get(actor_id)
            if target is not None:
                location = actor.get_location()
                if self._location_distance(location, target) <= 1.5:
                    controller = self._pedestrian_controllers[actor_id]
                    if actor_id in self._relocating_pedestrians:
                        if controller.is_alive:
                            controller.stop()
                        self._relocating_pedestrians.discard(actor_id)
                        self._active_pedestrians.discard(actor_id)
                        route = self._pedestrian_route_by_id.get(actor_id)
                        if route is not None:
                            self._pedestrian_targets[actor_id] = route[1]
                        state.mode = PedestrianMode.WAITING
                    elif actor_id in self._active_pedestrians:
                        next_route = self._next_route_from_location(location)
                        if next_route is None:
                            next_target = self._select_pedestrian_target(actor)
                            if self._location_distance(location, next_target) > 1.5:
                                self._pedestrian_targets[actor_id] = next_target
                                self._relocating_pedestrians.add(actor_id)
                                if controller.is_alive:
                                    controller.start()
                                    controller.go_to_location(next_target)
                                state.mode = PedestrianMode.WAITING
                            else:
                                if controller.is_alive:
                                    controller.stop()
                                self._active_pedestrians.discard(actor_id)
                                state.mode = PedestrianMode.COMPLETED
                        else:
                            self._pedestrian_route_by_id[actor_id] = next_route
                            self._pedestrian_targets[actor_id] = next_route[0]
                            self._relocating_pedestrians.add(actor_id)
                            if controller.is_alive:
                                controller.start()
                                controller.go_to_location(next_route[0])
                            state.mode = PedestrianMode.WAITING
            if state.mode is PedestrianMode.WAITING and state.speed_mps > 0.1:
                state.mode = PedestrianMode.CROSSING

    def _try_spawn_vehicle_from_map(self, blueprint: Any) -> Any | None:
        candidate_indices = [
            index
            for index in range(len(self._vehicle_spawn_points))
            if index not in self._used_vehicle_spawn_points
        ]
        existing_locations = [
            actor.get_location()
            for actor in self._vehicle_actors.values()
            if actor.is_alive
        ]
        if existing_locations:
            candidate_indices.sort(
                key=lambda index: self._minimum_spawn_distance(
                    self._vehicle_spawn_points[index], existing_locations
                ),
                reverse=True,
            )
        for index in candidate_indices:
            if existing_locations and self._minimum_spawn_distance(
                self._vehicle_spawn_points[index], existing_locations
            ) < 8.0:
                continue
            actor = self.carla_world.try_spawn_actor(blueprint, self._vehicle_spawn_points[index])
            if actor is not None:
                self._used_vehicle_spawn_points.add(index)
                return actor
        for index in candidate_indices:
            actor = self.carla_world.try_spawn_actor(blueprint, self._vehicle_spawn_points[index])
            if actor is not None:
                self._used_vehicle_spawn_points.add(index)
                return actor
        return None

    @staticmethod
    def _minimum_spawn_distance(transform: Any, locations: list[Any]) -> float:
        return min(
            math.hypot(
                transform.location.x - location.x,
                transform.location.y - location.y,
            )
            for location in locations
        )

    @classmethod
    def _select_compact_spawn_points(
        cls,
        spawn_points: list[Any],
        required_count: int,
    ) -> list[Any]:
        """Keep a dense local group of map spawn points for visual demos."""
        if len(spawn_points) <= required_count:
            return spawn_points
        for radius_m in (60.0, 100.0, 160.0, 240.0):
            best_cluster: list[Any] = []
            for seed in spawn_points:
                cluster = [
                    candidate
                    for candidate in spawn_points
                    if cls._location_distance(seed.location, candidate.location) <= radius_m
                ]
                if len(cluster) > len(best_cluster):
                    best_cluster = cluster
            if len(best_cluster) >= required_count:
                return best_cluster
        return spawn_points

    @staticmethod
    def _location_distance(first: Any, second: Any) -> float:
        return math.hypot(first.x - second.x, first.y - second.y)

    def _try_spawn_walker_from_navigation(
        self,
        blueprint: Any,
        heading: Any,
        preferred_location: Any | None = None,
    ) -> Any | None:
        existing_locations = [
            actor.get_location()
            for actor in self._pedestrian_actors.values()
            if actor.is_alive
        ]
        candidate_locations: list[Any] = []
        if preferred_location is not None:
            candidate_locations.append(preferred_location)
        for _ in range(100):
            candidate_locations.append(self.carla_world.get_random_location_from_navigation())
        for location in candidate_locations:
            if location is None:
                continue
            if existing_locations and self._minimum_location_distance(
                location, existing_locations
            ) < 1.5:
                continue
            actor = self.carla_world.try_spawn_actor(
                blueprint,
                self._carla.Transform(
                    location,
                    self._carla.Rotation(yaw=math.degrees(math.atan2(heading.y, heading.x))),
                ),
            )
            if actor is not None:
                return actor
        return None

    @staticmethod
    def _minimum_location_distance(location: Any, locations: list[Any]) -> float:
        return min(
            math.hypot(
                location.x - other.x,
                location.y - other.y,
            )
            for other in locations
        )

    def _update_vehicle_state(self, state: VehicleState) -> None:
        actor = self._vehicle_actors.get(state.actor_id)
        if actor is None or not actor.is_alive:
            return
        transform = actor.get_transform()
        velocity = actor.get_velocity()
        state.position = self._vec2(transform.location.x, transform.location.y)
        state.heading = self._vec2(
            math.cos(math.radians(transform.rotation.yaw)),
            math.sin(math.radians(transform.rotation.yaw)),
        )
        state.speed_mps = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

    def _update_pedestrian_state(self, state: PedestrianState) -> None:
        actor = self._pedestrian_actors.get(state.actor_id)
        if actor is None or not actor.is_alive:
            return
        transform = actor.get_transform()
        velocity = actor.get_velocity()
        state.position = self._vec2(transform.location.x, transform.location.y)
        state.heading = self._vec2(
            math.cos(math.radians(transform.rotation.yaw)),
            math.sin(math.radians(transform.rotation.yaw)),
        )
        state.speed_mps = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

    def _transform_for(self, x: float, y: float, heading: Any) -> Any:
        yaw = math.degrees(math.atan2(heading.y, heading.x))
        return self._carla.Transform(
            self._carla.Location(x=x, y=y, z=0.5),
            self._carla.Rotation(yaw=yaw),
        )

    def _actor_speed_mps(self, actor: Any) -> float:
        velocity = actor.get_velocity()
        return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

    def _traffic_light_requires_stop(self, actor: Any) -> bool:
        """Stop manually controlled vehicles for red or yellow CARLA lights."""
        if (
            not hasattr(actor, "is_at_traffic_light")
            or not hasattr(actor, "get_traffic_light_state")
            or not actor.is_at_traffic_light()
        ):
            return False
        state = actor.get_traffic_light_state()
        return state in {
            self._carla.TrafficLightState.Red,
            self._carla.TrafficLightState.Yellow,
        }

    def _safe_target_speed(self, actor: Any, target_speed_mps: float) -> float:
        """Reduce speed before junctions and sharp waypoint changes."""
        target = self._lane_following_target(actor)
        if target is None:
            return target_speed_mps
        waypoint = self._driving_waypoint(actor.get_location())
        if waypoint is not None and waypoint.is_junction:
            target_speed_mps = min(target_speed_mps, 4.0)
        yaw_error = self._waypoint_yaw_error(actor, target)
        if abs(yaw_error) > 35.0:
            target_speed_mps = min(target_speed_mps, 3.0)
        return target_speed_mps

    def _imminent_vehicle_conflicts(self) -> set[str]:
        """Return vehicles that need a physics-level emergency brake.

        The backend-independent policy only sees the abstract scenario. CARLA
        actors can approach one another on map lanes or across intersections,
        so this last-resort guard uses live positions and velocities to stop a
        vehicle before a likely collision. It does not replace the policy; it
        only overrides throttle when the physical world is becoming unsafe.
        """
        live_vehicles = [
            (actor_id, actor)
            for actor_id, actor in self._vehicle_actors.items()
            if actor.is_alive
        ]
        emergency_brakes: set[str] = set()
        for index, (first_id, first_actor) in enumerate(live_vehicles):
            for second_id, second_actor in live_vehicles[index + 1 :]:
                emergency_brakes.update(
                    self._vehicle_conflict_brakes(
                        first_id,
                        first_actor,
                        second_id,
                        second_actor,
                    )
                )
        return emergency_brakes

    def _vehicle_conflict_brakes(
        self,
        first_id: str,
        first_actor: Any,
        second_id: str,
        second_actor: Any,
    ) -> set[str]:
        first_location = first_actor.get_location()
        second_location = second_actor.get_location()
        first_waypoint = self._driving_waypoint(first_location)
        second_waypoint = self._driving_waypoint(second_location)
        first_forward = first_actor.get_transform().get_forward_vector()
        second_forward = second_actor.get_transform().get_forward_vector()
        heading_alignment = abs(
            first_forward.x * second_forward.x + first_forward.y * second_forward.y
        )
        relative_x = second_location.x - first_location.x
        relative_y = second_location.y - first_location.y
        lateral_distance = abs(
            relative_x * -first_forward.y + relative_y * first_forward.x
        )

        if self._same_lane(first_waypoint, second_waypoint) or (
            heading_alignment >= 0.8 and lateral_distance <= 2.2
        ):
            return self._same_lane_brakes(
                first_id,
                first_actor,
                second_id,
                second_actor,
            )

        # Cars on parallel lanes of the same road are not a conflict. At a
        # junction, however, their predicted paths may cross.
        if (
            first_waypoint is not None
            and second_waypoint is not None
            and first_waypoint.road_id == second_waypoint.road_id
            and not first_waypoint.is_junction
            and not second_waypoint.is_junction
        ):
            return set()

        first_velocity = first_actor.get_velocity()
        second_velocity = second_actor.get_velocity()
        relative_position = (
            second_location.x - first_location.x,
            second_location.y - first_location.y,
        )
        relative_velocity = (
            second_velocity.x - first_velocity.x,
            second_velocity.y - first_velocity.y,
        )
        relative_speed_squared = (
            relative_velocity[0] ** 2 + relative_velocity[1] ** 2
        )
        if math.hypot(relative_position[0], relative_position[1]) <= 6.0:
            return {first_id, second_id}
        if relative_speed_squared <= 1e-6:
            return set()

        time_to_closest = -(
            relative_position[0] * relative_velocity[0]
            + relative_position[1] * relative_velocity[1]
        ) / relative_speed_squared
        if not 0.0 < time_to_closest <= 4.0:
            return set()

        closest_x = relative_position[0] + relative_velocity[0] * time_to_closest
        closest_y = relative_position[1] + relative_velocity[1] * time_to_closest
        if math.hypot(closest_x, closest_y) > 6.0:
            return set()
        return {first_id, second_id}

    def _same_lane_brakes(
        self,
        first_id: str,
        first_actor: Any,
        second_id: str,
        second_actor: Any,
    ) -> set[str]:
        first_transform = first_actor.get_transform()
        first_forward = first_transform.get_forward_vector()
        first_location = first_transform.location
        second_location = second_actor.get_location()
        relative_x = second_location.x - first_location.x
        relative_y = second_location.y - first_location.y
        longitudinal = relative_x * first_forward.x + relative_y * first_forward.y
        if longitudinal >= 0:
            leader = second_actor
            follower_id, follower = first_id, first_actor
        else:
            leader = first_actor
            follower_id, follower = second_id, second_actor

        follower_location = follower.get_location()
        leader_location = leader.get_location()
        follower_transform = follower.get_transform()
        follower_forward = follower_transform.get_forward_vector()
        center_distance = (
            (leader_location.x - follower_location.x) * follower_forward.x
            + (leader_location.y - follower_location.y) * follower_forward.y
        )
        gap = center_distance - 4.5
        if gap <= 6.0:
            return {follower_id}

        leader_speed = self._actor_speed_mps(leader)
        follower_speed = self._actor_speed_mps(follower)
        closing_speed = follower_speed - leader_speed
        if closing_speed <= 0:
            return set()
        time_to_hard_gap = (gap - 6.0) / closing_speed
        if time_to_hard_gap <= 4.0:
            return {follower_id}
        return set()

    def _driving_waypoint(self, location: Any) -> Any | None:
        return self.carla_world.get_map().get_waypoint(
            location,
            project_to_road=True,
            lane_type=self._carla.LaneType.Driving,
        )

    @staticmethod
    def _same_lane(first_waypoint: Any | None, second_waypoint: Any | None) -> bool:
        return (
            first_waypoint is not None
            and second_waypoint is not None
            and first_waypoint.road_id == second_waypoint.road_id
            and first_waypoint.lane_id == second_waypoint.lane_id
        )

    def _lane_following_target(self, actor: Any) -> Any | None:
        """Choose a forward lane branch without taking a U-turn."""
        waypoint = self._driving_waypoint(actor.get_location())
        if waypoint is None:
            return None
        next_waypoints = waypoint.next(8.0)
        if not next_waypoints:
            return waypoint
        current_forward = actor.get_transform().get_forward_vector()
        forward_candidates = [
            candidate
            for candidate in next_waypoints
            if (
                current_forward.x * candidate.transform.get_forward_vector().x
                + current_forward.y * candidate.transform.get_forward_vector().y
            )
            >= -0.5
        ]
        candidates = forward_candidates or next_waypoints
        return min(
            candidates,
            key=lambda candidate: abs(self._waypoint_yaw_error(actor, candidate)),
        )

    @staticmethod
    def _waypoint_yaw_error(actor: Any, waypoint: Any) -> float:
        current_yaw = actor.get_transform().rotation.yaw
        target_yaw = waypoint.transform.rotation.yaw
        return (target_yaw - current_yaw + 180.0) % 360.0 - 180.0

    def _lane_following_steer(self, actor: Any) -> float:
        """Steer toward a forward short-horizon CARLA lane waypoint."""
        target = self._lane_following_target(actor)
        if target is None:
            return 0.0
        yaw_error = self._waypoint_yaw_error(actor, target)
        return max(-1.0, min(1.0, yaw_error / 45.0))

    def _update_spectator(self) -> None:
        if self._spectator_follow_id is None:
            return
        actor = self._vehicle_actors.get(self._spectator_follow_id)
        if actor is None or not actor.is_alive:
            return
        transform = actor.get_transform()
        # CARLA exposes the forward vector on Transform, not on Actor.
        forward = transform.get_forward_vector()
        camera_distance = 24.0 if self._compact_traffic else 12.0
        camera_height = 14.0 if self._compact_traffic else 7.0
        camera_location = self._carla.Location(
            x=transform.location.x - forward.x * camera_distance,
            y=transform.location.y - forward.y * camera_distance,
            z=transform.location.z + camera_height,
        )
        camera_rotation = self._carla.Rotation(
            pitch=-20.0,
            yaw=transform.rotation.yaw,
        )
        self.carla_world.get_spectator().set_transform(
            self._carla.Transform(camera_location, camera_rotation)
        )

    @staticmethod
    def _vec2(x: float, y: float) -> Any:
        from .geometry import Vec2

        return Vec2(x, y)

