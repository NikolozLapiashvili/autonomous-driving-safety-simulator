from pathlib import Path

import pytest

from autodrive_sim.config import ScenarioConfig


def test_scenario_config_rejects_unknown_fields() -> None:
    with pytest.raises(ValueError, match="unknown scenario fields"):
        ScenarioConfig.from_mapping({"vehicle_count": 3, "unsupported": True})


def test_scenario_config_loads_toml(tmp_path: Path) -> None:
    path = tmp_path / "scenario.toml"
    path.write_text(
        "[scenario]\n"
        "name = 'test-crossing'\n"
        "seed = 21\n"
        "vehicle_count = 4\n"
        "pedestrian_count = 2\n",
        encoding="utf-8",
    )

    config = ScenarioConfig.from_toml(path)

    assert config.name == "test-crossing"
    assert config.seed == 21
    assert config.vehicle_count == 4
    assert config.pedestrian_count == 2


def test_scenario_config_limits_total_actor_count() -> None:
    with pytest.raises(ValueError):
        ScenarioConfig(vehicle_count=10_000, pedestrian_count=1)

