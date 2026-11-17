from autodrive_sim.geometry import Vec2
from autodrive_sim.models import PedestrianState, VehicleState, WorldState
from autodrive_sim.safety import SafetyMonitor, SafetyMonitorConfig


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


def test_safety_monitor_does_not_repeat_active_conflict_every_frame() -> None:
    state = WorldState(
        vehicles={
            "car-1": VehicleState("car-1", Vec2(0, 0), Vec2(1, 0), 0),
            "car-2": VehicleState("car-2", Vec2(1, 0), Vec2(-1, 0), 0),
        }
    )
    monitor = SafetyMonitor()

    assert len(monitor.inspect(state)) == 1
    assert monitor.inspect(state) == []


def test_safety_monitor_ignores_pedestrian_pedestrian_overlap_by_default() -> None:
    state = WorldState(
        pedestrians={
            "ped-1": PedestrianState("ped-1", Vec2(0, 0), Vec2(1, 0), 0),
            "ped-2": PedestrianState("ped-2", Vec2(0, 0), Vec2(-1, 0), 0),
        }
    )

    assert SafetyMonitor().inspect(state) == []


def test_safety_monitor_can_include_pedestrian_pedestrian_overlap() -> None:
    state = WorldState(
        pedestrians={
            "ped-1": PedestrianState("ped-1", Vec2(0, 0), Vec2(1, 0), 0),
            "ped-2": PedestrianState("ped-2", Vec2(0, 0), Vec2(-1, 0), 0),
        }
    )

    events = SafetyMonitor(
        SafetyMonitorConfig(include_pedestrian_pedestrian=True)
    ).inspect(state)

    assert len(events) == 1
    assert events[0].payload["kind"] == "collision"

