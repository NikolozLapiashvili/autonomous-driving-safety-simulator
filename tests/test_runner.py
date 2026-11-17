import pytest

from autodrive_sim.clock import SimulationClock
from autodrive_sim.config import ScenarioConfig
from autodrive_sim.events import EventType, MemoryEventSink
from autodrive_sim.runner import SimulationRunner
from autodrive_sim.scenario import ScenarioFactory


def test_runner_advances_world_and_clock_together() -> None:
    scenario = ScenarioFactory().build(
        ScenarioConfig(seed=4, vehicle_count=1, pedestrian_count=0)
    )
    runner = SimulationRunner(
        world=scenario.world,
        clock=SimulationClock(fixed_delta_s=0.1),
        signal_controllers=scenario.signal_controllers,
    )
    initial_position = scenario.world.state.vehicles["car-0000"].position

    frames = runner.run_steps(3)

    assert len(frames) == 3
    assert frames[-1].frame == 3
    assert frames[-1].time_s == pytest.approx(0.3)
    assert scenario.world.state.time_s == pytest.approx(0.3)
    assert scenario.world.state.vehicles["car-0000"].position != initial_position


def test_runner_rejects_non_integral_duration() -> None:
    scenario = ScenarioFactory().build(ScenarioConfig(vehicle_count=0, pedestrian_count=0))
    runner = SimulationRunner(world=scenario.world, clock=SimulationClock(0.1))
    with pytest.raises(ValueError, match="divisible"):
        runner.run_for(0.15)


def test_runner_emits_policy_events() -> None:
    scenario = ScenarioFactory().build(
        ScenarioConfig(seed=4, vehicle_count=1, pedestrian_count=1)
    )
    sink = MemoryEventSink()
    runner = SimulationRunner(
        world=scenario.world,
        clock=SimulationClock(fixed_delta_s=0.1),
        signal_controllers=scenario.signal_controllers,
        event_sink=sink,
    )

    runner.step()

    assert len(sink.events) == 2
    assert all(event.event_type is EventType.POLICY_DECISION for event in sink.events)
    assert {event.actor_id for event in sink.events} == {"car-0000", "ped-0000"}

