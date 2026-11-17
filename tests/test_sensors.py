from pathlib import Path

import pytest

from autodrive_sim.sensors import SensorConfig


def test_sensor_config_has_portfolio_friendly_defaults() -> None:
    config = SensorConfig()
    assert config.output_dir == Path("artifacts/sensors")
    assert config.image_width == 1280
    assert config.image_height == 720


def test_sensor_config_validates_field_of_view() -> None:
    with pytest.raises(ValueError):
        SensorConfig(camera_fov=180)

