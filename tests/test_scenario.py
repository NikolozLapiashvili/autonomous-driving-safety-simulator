from autodrive_sim.config import ScenarioConfig
from autodrive_sim.scenario import ScenarioFactory


def test_factory_spawns_requested_actor_counts() -> None:
    scenario = ScenarioFactory().build(
        ScenarioConfig(seed=10, vehicle_count=25, pedestrian_count=13)
    )
    assert len(scenario.world.state.vehicles) == 25
    assert len(scenario.world.state.pedestrians) == 13
    assert scenario.world.state.crosswalks["cw-main"].signal_id == "signal-main"


def test_factory_is_deterministic_for_same_seed() -> None:
    config = ScenarioConfig(seed=99, vehicle_count=8, pedestrian_count=5)
    first = ScenarioFactory().build(config).world.state
    second = ScenarioFactory().build(config).world.state
    assert first.vehicles == second.vehicles
    assert first.pedestrians == second.pedestrians


def test_factory_changes_spawn_layout_for_different_seed() -> None:
    first = ScenarioFactory().build(ScenarioConfig(seed=1, vehicle_count=4)).world.state
    second = ScenarioFactory().build(ScenarioConfig(seed=2, vehicle_count=4)).world.state
    assert first.vehicles["car-0000"].position != second.vehicles["car-0000"].position

