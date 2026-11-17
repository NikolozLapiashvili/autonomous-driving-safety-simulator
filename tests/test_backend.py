import pytest

from autodrive_sim.backend import KinematicWorld
from autodrive_sim.geometry import Rect, Vec2
from autodrive_sim.models import (
    Crosswalk,
    PedestrianCommand,
    PedestrianDecisionReason,
    PedestrianMode,
    PedestrianState,
    PolicyDecision,
    VehicleCommand,
    VehicleDecisionReason,
    VehicleState,
)


def test_vehicle_brakes_toward_policy_target() -> None:
    world = KinematicWorld()
    vehicle = VehicleState("car-1", Vec2(0, 0), Vec2(1, 0), speed_mps=10)
    world.spawn_vehicle(vehicle)
    decision = PolicyDecision(
        vehicle_commands={
            "car-1": VehicleCommand(0, True, VehicleDecisionReason.SIGNAL_STOP),
        }
    )

    world.step(0.5, decision)

    assert vehicle.speed_mps == pytest.approx(7)
    assert vehicle.position == Vec2(3.5, 0)


def test_pedestrian_starts_crossing_only_when_allowed() -> None:
    crosswalk = Crosswalk("cw-1", Rect(Vec2(0, 0), Vec2(3, 2)), Vec2(0, 1), "sig-1")
    pedestrian = PedestrianState(
        "ped-1", Vec2(0, -4), Vec2(0, 1), crosswalk_id="cw-1"
    )
    world = KinematicWorld()
    world.add_crosswalk(crosswalk)
    world.spawn_pedestrian(pedestrian)
    waiting = PolicyDecision(
        pedestrian_commands={
            "ped-1": PedestrianCommand(False, PedestrianDecisionReason.WAIT_FOR_SIGNAL),
        }
    )
    world.step(1.0, waiting)
    assert pedestrian.mode is PedestrianMode.WAITING

    walk = PolicyDecision(
        pedestrian_commands={
            "ped-1": PedestrianCommand(True, PedestrianDecisionReason.WALK_SIGNAL),
        }
    )
    world.step(1.0, walk)
    assert pedestrian.mode is PedestrianMode.CROSSING
    assert pedestrian.position == Vec2(0, -2.6)


def test_duplicate_actor_ids_are_rejected() -> None:
    world = KinematicWorld()
    world.spawn_vehicle(VehicleState("car-1", Vec2(0, 0), Vec2(1, 0), 1))
    with pytest.raises(ValueError):
        world.spawn_vehicle(VehicleState("car-1", Vec2(1, 0), Vec2(1, 0), 1))

