import pytest

from autodrive_sim.events import EventType, SimulationEvent
from autodrive_sim.geometry import Vec2
from autodrive_sim.metrics import MetricsCollector
from autodrive_sim.models import PedestrianMode, PedestrianState, WorldState


def test_metrics_count_policy_and_signal_events() -> None:
    metrics = MetricsCollector()
    metrics.record(
        SimulationEvent(
            0,
            EventType.POLICY_DECISION,
            {"actor_kind": "vehicle", "should_stop": True},
            "car-1",
        )
    )
    metrics.record(SimulationEvent(1, EventType.SIGNAL_PHASE_CHANGED))

    report = metrics.report()

    assert report.policy_decisions == 1
    assert report.vehicle_stop_decisions == 1
    assert report.signal_phase_changes == 1


def test_metrics_accumulate_pedestrian_wait_time() -> None:
    metrics = MetricsCollector()
    state = WorldState(
        pedestrians={
            "ped-1": PedestrianState("ped-1", Vec2(0, 0), Vec2(0, 1)),
            "ped-2": PedestrianState(
                "ped-2", Vec2(1, 0), Vec2(0, 1), mode=PedestrianMode.CROSSING
            ),
        }
    )

    metrics.observe(state, 0.5)

    assert metrics.report().pedestrian_wait_time_s == pytest.approx(0.5)

