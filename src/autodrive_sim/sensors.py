"""Optional CARLA sensor suite for recorded observations and safety events."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .events import EventSink, EventType, SimulationEvent


@dataclass(frozen=True, slots=True)
class SensorConfig:
    """Recording settings for the ego vehicle sensor suite."""

    output_dir: Path = Path("artifacts/sensors")
    image_width: int = 1280
    image_height: int = 720
    camera_fov: float = 90.0

    def __post_init__(self) -> None:
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("sensor image dimensions must be positive")
        if not 1.0 <= self.camera_fov <= 179.0:
            raise ValueError("camera field of view must be between 1 and 179 degrees")


@dataclass(slots=True)
class CarlaSensorManager:
    """Attach and clean up a small, reusable CARLA sensor suite."""

    carla_world: Any
    carla_module: Any
    config: SensorConfig = field(default_factory=SensorConfig)
    event_sink: EventSink | None = None
    _sensors: list[Any] = field(default_factory=list, repr=False)

    def attach_front_camera(self, vehicle_actor: Any, actor_id: str) -> Any:
        """Attach an RGB camera and save each frame under the output directory."""
        self.config.output_dir.mkdir(parents=True, exist_ok=True)
        blueprint = self.carla_world.get_blueprint_library().find("sensor.camera.rgb")
        blueprint.set_attribute("image_size_x", str(self.config.image_width))
        blueprint.set_attribute("image_size_y", str(self.config.image_height))
        blueprint.set_attribute("fov", str(self.config.camera_fov))
        transform = self.carla_module.Transform(
            self.carla_module.Location(x=1.5, z=1.6),
            self.carla_module.Rotation(pitch=-5.0),
        )
        sensor = self.carla_world.spawn_actor(blueprint, transform, attach_to=vehicle_actor)
        if sensor is None:
            raise RuntimeError("CARLA could not attach the RGB camera")
        sensor.listen(lambda image: self._save_image(image, actor_id))
        self._sensors.append(sensor)
        return sensor

    def attach_collision_sensor(self, vehicle_actor: Any, actor_id: str) -> Any:
        """Attach a collision sensor that emits structured safety events."""
        blueprint = self.carla_world.get_blueprint_library().find("sensor.other.collision")
        sensor = self.carla_world.spawn_actor(
            blueprint,
            self.carla_module.Transform(),
            attach_to=vehicle_actor,
        )
        if sensor is None:
            raise RuntimeError("CARLA could not attach the collision sensor")
        sensor.listen(lambda event: self._record_collision(event, actor_id))
        self._sensors.append(sensor)
        return sensor

    def attach_lane_invasion_sensor(self, vehicle_actor: Any, actor_id: str) -> Any:
        """Attach a lane-invasion sensor that emits a safety event."""
        blueprint = self.carla_world.get_blueprint_library().find("sensor.other.lane_invasion")
        sensor = self.carla_world.spawn_actor(
            blueprint,
            self.carla_module.Transform(),
            attach_to=vehicle_actor,
        )
        if sensor is None:
            raise RuntimeError("CARLA could not attach the lane-invasion sensor")
        sensor.listen(lambda event: self._record_lane_invasion(event, actor_id))
        self._sensors.append(sensor)
        return sensor

    def close(self) -> None:
        """Stop and destroy every sensor attached by this manager."""
        for sensor in self._sensors:
            if sensor.is_alive:
                sensor.stop()
                sensor.destroy()
        self._sensors.clear()

    def _save_image(self, image: Any, actor_id: str) -> None:
        path = self.config.output_dir / f"{actor_id}_rgb_{image.frame:08d}.png"
        image.save_to_disk(str(path))

    def _record_collision(self, event: Any, actor_id: str) -> None:
        if self.event_sink is None:
            return
        impulse = event.normal_impulse
        self.event_sink.record(
            SimulationEvent(
                time_s=float(getattr(event, "timestamp", 0.0)),
                event_type=EventType.SAFETY_VIOLATION,
                actor_id=actor_id,
                payload={
                    "kind": "carla_collision",
                    "other_actor_id": getattr(getattr(event, "other_actor", None), "id", None),
                    "normal_impulse": {
                        "x": float(impulse.x),
                        "y": float(impulse.y),
                        "z": float(impulse.z),
                    },
                },
            )
        )

    def _record_lane_invasion(self, event: Any, actor_id: str) -> None:
        if self.event_sink is None:
            return
        markings = [str(marking.type) for marking in getattr(event, "crossed_lane_markings", [])]
        self.event_sink.record(
            SimulationEvent(
                time_s=float(getattr(event, "timestamp", 0.0)),
                event_type=EventType.SAFETY_VIOLATION,
                actor_id=actor_id,
                payload={"kind": "lane_invasion", "markings": markings},
            )
        )

