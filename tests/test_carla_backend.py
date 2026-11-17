import pytest

import autodrive_sim.carla_backend as carla_backend
from autodrive_sim.carla_backend import CarlaBackend, CarlaConnectionConfig, CarlaUnavailableError
from autodrive_sim.geometry import Vec2


def test_carla_module_is_optional(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(carla_backend.importlib, "import_module", lambda _: _raise_module_error())

    with pytest.raises(CarlaUnavailableError):
        CarlaBackend.connect()


def test_carla_connection_config_validates_port() -> None:
    with pytest.raises(ValueError):
        CarlaConnectionConfig(port=70_000)


def test_heading_is_converted_to_expected_yaw() -> None:
    assert Vec2(1, 0).normalized() == Vec2(1, 0)


def test_spectator_reads_forward_vector_from_transform() -> None:
    class FakeTransform:
        location = type("Location", (), {"x": 10.0, "y": 20.0, "z": 1.0})()
        rotation = type("Rotation", (), {"yaw": 90.0})()

        def get_forward_vector(self) -> object:
            return type("Vector", (), {"x": 0.0, "y": 1.0})()

    class FakeActor:
        is_alive = True

        def get_transform(self) -> FakeTransform:
            return FakeTransform()

    class FakeSpectator:
        transform: object | None = None

        def set_transform(self, transform: object) -> None:
            self.transform = transform

    class FakeWorld:
        def __init__(self) -> None:
            self.spectator = FakeSpectator()

        def get_spectator(self) -> FakeSpectator:
            return self.spectator

    class FakeLocation:
        def __init__(self, *, x: float, y: float, z: float) -> None:
            self.x = x
            self.y = y
            self.z = z

    class FakeRotation:
        def __init__(self, *, pitch: float, yaw: float) -> None:
            self.pitch = pitch
            self.yaw = yaw

    class FakeCarlaTransform:
        def __init__(self, location: FakeLocation, rotation: FakeRotation) -> None:
            self.location = location
            self.rotation = rotation

    class FakeCarla:
        Location = FakeLocation
        Rotation = FakeRotation
        Transform = FakeCarlaTransform

    world = FakeWorld()
    backend = CarlaBackend(
        client=None,
        carla_world=world,
        config=CarlaConnectionConfig(),
        _carla=FakeCarla,
        _vehicle_actors={"car-0000": FakeActor()},
        _spectator_follow_id="car-0000",
    )

    backend._update_spectator()

    assert world.spectator.transform is not None
    assert world.spectator.transform.location.x == 10.0
    assert world.spectator.transform.location.y == 8.0
    assert world.spectator.transform.location.z == 8.0


def _raise_module_error() -> None:
    raise ModuleNotFoundError("carla")

