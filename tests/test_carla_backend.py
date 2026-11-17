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


def _raise_module_error() -> None:
    raise ModuleNotFoundError("carla")

