import pytest

from autodrive_sim.models import TrafficPhase, TrafficSignalState, WorldState
from autodrive_sim.signals import SignalController, SignalTiming


def test_signal_controller_advances_through_safe_sequence() -> None:
    state = WorldState(
        signals={"sig-1": TrafficSignalState("sig-1", TrafficPhase.VEHICLE_GREEN)}
    )
    timing = SignalTiming(
        vehicle_green_s=2,
        vehicle_yellow_s=1,
        all_red_s=0.5,
        pedestrian_walk_s=1,
    )
    controller = SignalController("sig-1", timing)

    controller.tick(state, 2.0)
    assert state.signals["sig-1"].phase is TrafficPhase.VEHICLE_YELLOW
    controller.tick(state, 1.0)
    assert state.signals["sig-1"].phase is TrafficPhase.ALL_RED
    controller.tick(state, 0.5)
    assert state.signals["sig-1"].phase is TrafficPhase.PEDESTRIAN_WALK


def test_signal_controller_handles_multiple_transitions() -> None:
    state = WorldState(
        signals={"sig-1": TrafficSignalState("sig-1", TrafficPhase.VEHICLE_GREEN)}
    )
    timing = SignalTiming(
        vehicle_green_s=1,
        vehicle_yellow_s=1,
        all_red_s=1,
        pedestrian_walk_s=1,
    )
    controller = SignalController("sig-1", timing)
    controller.tick(state, 5.25)
    assert state.signals["sig-1"].phase is TrafficPhase.VEHICLE_GREEN
    assert state.signals["sig-1"].elapsed_s == pytest.approx(0.25)


def test_signal_controller_rejects_unknown_signal() -> None:
    with pytest.raises(KeyError):
        SignalController("missing").tick(WorldState(), 0.1)

