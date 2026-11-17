from autodrive_sim.geometry import Vec2
from autodrive_sim.models import PedestrianState, VehicleState, WorldState
from autodrive_sim.safety import SafetyMonitor


def test_safety_monitor_detects_collision() -> None:
    state = WorldState(
        vehicles={
            "car-1": VehicleState("car-1", Vec2(0, 0), Vec2(1, 0), 0),
            "car-2": VehicleState("car-2", Vec2(1, 0), Vec2(-1, 0), 0),
        }
    )

    events = SafetyMonitor().inspect(state)

    assert len(events) == 1
    assert events[0].payload["kind"] == "collision"


def test_safety_monitor_detects_near_miss() -> None:
    state = WorldState(
        vehicles={
            "car-1": VehicleState("car-1", Vec2(0, 0), Vec2(1, 0), 0),
        },
        pedestrians={
            "ped-1": PedestrianState("ped-1", Vec2(2.75, 0), Vec2(0, 1), 0),
        },
    )

    events = SafetyMonitor().inspect(state)

    assert len(events) == 1
    assert events[0].payload["kind"] == "near_miss"

