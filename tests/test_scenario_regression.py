import pytest

from autodrive_sim.config import ScenarioConfig
from autodrive_sim.events import CompositeEventSink
from autodrive_sim.metrics import MetricsCollector
from autodrive_sim.runner import SimulationRunner
from autodrive_sim.safety import SafetyMonitor
from autodrive_sim.scenario import ScenarioFactory


@pytest.mark.parametrize("seed", range(10))
def test_baseline_scenario_stays_collision_free_across_seeds(seed: int) -> None:
    scenario = ScenarioFactory().build(
        ScenarioConfig(seed=seed, vehicle_count=12, pedestrian_count=6)
    )
    metrics = MetricsCollector()
    runner = SimulationRunner(
        world=scenario.world,
        signal_controllers=scenario.signal_controllers,
        event_sink=CompositeEventSink((metrics,)),
        metrics=metrics,
        safety_monitor=SafetyMonitor(),
    )

    runner.run_steps(100)

    assert metrics.report().safety_violations == 0


def test_full_signal_cycle_allows_pedestrians_without_conflicts() -> None:
    scenario = ScenarioFactory().build(
        ScenarioConfig(seed=42, vehicle_count=12, pedestrian_count=6)
    )
    metrics = MetricsCollector()
    runner = SimulationRunner(
        world=scenario.world,
        signal_controllers=scenario.signal_controllers,
        event_sink=CompositeEventSink((metrics,)),
        metrics=metrics,
        safety_monitor=SafetyMonitor(),
    )

    runner.run_steps(600)

    report = metrics.report()
    assert report.signal_phase_changes >= 3
    assert report.vehicle_stop_decisions > 0
    assert report.safety_violations == 0

