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
    _vehicle_actors: dict[str, Any] = field(default_factory=dict, repr=False)
    _pedestrian_actors: dict[str, Any] = field(default_factory=dict, repr=False)
    _pedestrian_controllers: dict[str, Any] = field(default_factory=dict, repr=False)
    _traffic_lights: dict[str, list[Any]] = field(default_factory=dict, repr=False)

    @classmethod
    def connect(cls, config: CarlaConnectionConfig | None = None) -> CarlaBackend:
        """Connect to a running CARLA server and configure stepping."""
        connection = config or CarlaConnectionConfig()
        try:
            carla = importlib.import_module("carla")
        except ModuleNotFoundError as error:
            raise CarlaUnavailableError(
                "CARLA is not installed; install the client matching the server release"
            ) from error
        client = carla.Client(connection.host, connection.port)
        client.set_timeout(connection.timeout_s)
        world = client.get_world()
        backend = cls(client, world, connection, _carla=carla)
        backend._configure_world()
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
        actor = self.carla_world.try_spawn_actor(
            blueprint,
            self._transform_for(vehicle.position.x, vehicle.position.y, vehicle.heading),
        )
        if actor is None:
            raise CarlaSpawnError(f"CARLA rejected vehicle spawn: {vehicle.actor_id}")
        self._vehicle_actors[vehicle.actor_id] = actor
        self.state.vehicles[vehicle.actor_id] = vehicle

    def spawn_pedestrian(self, pedestrian: PedestrianState) -> None:
        """Spawn a walker and its AI controller in CARLA."""
        if pedestrian.actor_id in self._pedestrian_actors:
            raise ValueError(f"pedestrian already exists: {pedestrian.actor_id}")
        blueprint_library = self.carla_world.get_blueprint_library()
        blueprints = blueprint_library.filter("walker.pedestrian.*")
        if not blueprints:
            raise CarlaSpawnError("CARLA has no pedestrian blueprints")
        walker = self.carla_world.try_spawn_actor(
            blueprints[0],
            self._transform_for(pedestrian.position.x, pedestrian.position.y, pedestrian.heading),
        )
        if walker is None:
            raise CarlaSpawnError(f"CARLA rejected pedestrian spawn: {pedestrian.actor_id}")
        controller_blueprint = blueprint_library.find("controller.ai.walker")
        controller = self.carla_world.spawn_actor(
            controller_blueprint,
            self._carla.Transform(),
            attach_to=walker,
        )
        if controller is None:
            walker.destroy()
            raise CarlaSpawnError(f"CARLA rejected walker controller: {pedestrian.actor_id}")
        controller.start()
        target = self._carla.Location(
            x=pedestrian.position.x + pedestrian.heading.x * 25.0,
            y=pedestrian.position.y + pedestrian.heading.y * 25.0,
            z=pedestrian.position.z if hasattr(pedestrian.position, "z") else 0.0,
        )
        controller.set_max_speed(pedestrian.speed_mps)
        controller.go_to_location(target)
        self._pedestrian_actors[pedestrian.actor_id] = walker
        self._pedestrian_controllers[pedestrian.actor_id] = controller
        self.state.pedestrians[pedestrian.actor_id] = pedestrian

    def step(self, delta_s: float, decision: PolicyDecision) -> None:
        """Apply policy commands and advance the CARLA world one frame."""
        if delta_s <= 0:
            raise ValueError("CARLA step delta must be positive")
        for actor_id, command in decision.vehicle_commands.items():
            actor = self._vehicle_actors.get(actor_id)
            if actor is None or not actor.is_alive:
                continue
            speed_mps = self._actor_speed_mps(actor)
            speed_error = command.target_speed_mps - speed_mps
            throttle = max(0.0, min(1.0, speed_error / 5.0))
            brake = 1.0 if command.should_stop else max(0.0, min(1.0, -speed_error / 5.0))
            actor.apply_control(self._carla.VehicleControl(throttle=throttle, brake=brake))
        if self.config.synchronous_mode:
            self.carla_world.tick()
        else:
            self.carla_world.wait_for_tick()
        self.state.time_s += delta_s
        self._refresh_state()

    def sync_signals(self) -> None:
        """Apply domain signal phases to CARLA traffic-light actors."""
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

    def _configure_world(self) -> None:
        settings = self.carla_world.get_settings()
        settings.synchronous_mode = self.config.synchronous_mode
        settings.fixed_delta_seconds = (
            self.config.fixed_delta_s if self.config.synchronous_mode else None
        )
        self.carla_world.apply_settings(settings)

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
            transform = actor.get_transform()
            velocity = actor.get_velocity()
            state.position = self._vec2(transform.location.x, transform.location.y)
            state.heading = self._vec2(
                math.cos(math.radians(transform.rotation.yaw)),
                math.sin(math.radians(transform.rotation.yaw)),
            )
            state.speed_mps = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        for actor_id, actor in self._pedestrian_actors.items():
            if not actor.is_alive:
                continue
            state = self.state.pedestrians[actor_id]
            transform = actor.get_transform()
            velocity = actor.get_velocity()
            state.position = self._vec2(transform.location.x, transform.location.y)
            state.heading = self._vec2(
                math.cos(math.radians(transform.rotation.yaw)),
                math.sin(math.radians(transform.rotation.yaw)),
            )
            state.speed_mps = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            if state.mode is PedestrianMode.WAITING and state.speed_mps > 0.1:
                state.mode = PedestrianMode.CROSSING

    def _transform_for(self, x: float, y: float, heading: Any) -> Any:
        yaw = math.degrees(math.atan2(heading.y, heading.x))
        return self._carla.Transform(
            self._carla.Location(x=x, y=y, z=0.5),
            self._carla.Rotation(yaw=yaw),
        )

    def _actor_speed_mps(self, actor: Any) -> float:
        velocity = actor.get_velocity()
        return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

    @staticmethod
    def _vec2(x: float, y: float) -> Any:
        from .geometry import Vec2

        return Vec2(x, y)

