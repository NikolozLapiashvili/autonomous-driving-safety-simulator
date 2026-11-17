from autodrive_sim.geometry import Rect, Vec2
from autodrive_sim.models import (
    Crosswalk,
    PedestrianDecisionReason,
    PedestrianMode,
    PedestrianState,
    TrafficPhase,
    TrafficSignalState,
    VehicleDecisionReason,
    VehicleState,
    WorldState,
)
from autodrive_sim.policy import SafetyPolicy


def make_world(
    *,
    phase: TrafficPhase,
    vehicle_position: Vec2 = Vec2(-10, 0),
    vehicle_speed_mps: float = 5,
    pedestrian: PedestrianState | None = None,
) -> WorldState:
    crosswalk = Crosswalk(
        crosswalk_id="cw-1",
        conflict_zone=Rect(Vec2(0, 0), Vec2(3, 2)),
        crossing_direction=Vec2(0, 1),
        signal_id="sig-1",
    )
    world = WorldState(
        vehicles={
            "car-1": VehicleState(
                actor_id="car-1",
                position=vehicle_position,
                heading=Vec2(1, 0),
                speed_mps=vehicle_speed_mps,
            )
        },
        crosswalks={"cw-1": crosswalk},
        signals={"sig-1": TrafficSignalState("sig-1", phase)},
    )
    if pedestrian is not None:
        world.pedestrians[pedestrian.actor_id] = pedestrian
    return world


def test_red_signal_stops_approaching_vehicle() -> None:
    world = make_world(phase=TrafficPhase.VEHICLE_YELLOW)
    decision = SafetyPolicy().evaluate(world)
    command = decision.vehicle_commands["car-1"]
    assert command.should_stop
    assert command.reason is VehicleDecisionReason.SIGNAL_STOP


def test_green_signal_allows_clear_vehicle() -> None:
    world = make_world(phase=TrafficPhase.VEHICLE_GREEN)
    decision = SafetyPolicy().evaluate(world)
    command = decision.vehicle_commands["car-1"]
    assert not command.should_stop
    assert command.reason is VehicleDecisionReason.CLEAR_TO_PROCEED


def test_pedestrian_in_zone_stops_approaching_vehicle() -> None:
    pedestrian = PedestrianState(
        actor_id="ped-1",
        position=Vec2(0, 0),
        heading=Vec2(0, 1),
        mode=PedestrianMode.CROSSING,
        crosswalk_id="cw-1",
    )
    world = make_world(phase=TrafficPhase.VEHICLE_GREEN, pedestrian=pedestrian)
    command = SafetyPolicy().evaluate(world).vehicle_commands["car-1"]
    assert command.should_stop
    assert command.reason is VehicleDecisionReason.PEDESTRIAN_IN_CONFLICT_ZONE


def test_walk_signal_allows_pedestrian_when_vehicle_is_far_away() -> None:
    pedestrian = PedestrianState(
        actor_id="ped-1",
        position=Vec2(0, -4),
        heading=Vec2(0, 1),
        crosswalk_id="cw-1",
    )
    world = make_world(
        phase=TrafficPhase.PEDESTRIAN_WALK,
        vehicle_position=Vec2(-30, 0),
        vehicle_speed_mps=2,
        pedestrian=pedestrian,
    )
    command = SafetyPolicy().evaluate(world).pedestrian_commands["ped-1"]
    assert command.may_enter_crosswalk
    assert command.reason is PedestrianDecisionReason.WALK_SIGNAL


def test_walk_signal_denies_pedestrian_with_fast_vehicle_conflict() -> None:
    pedestrian = PedestrianState(
        actor_id="ped-1",
        position=Vec2(0, -4),
        heading=Vec2(0, 1),
        crosswalk_id="cw-1",
    )
    world = make_world(
        phase=TrafficPhase.PEDESTRIAN_WALK,
        vehicle_position=Vec2(-10, 0),
        vehicle_speed_mps=5,
        pedestrian=pedestrian,
    )
    command = SafetyPolicy().evaluate(world).pedestrian_commands["ped-1"]
    assert not command.may_enter_crosswalk
    assert command.reason is PedestrianDecisionReason.VEHICLE_CONFLICT


def test_vehicle_inside_zone_is_allowed_to_clear_it() -> None:
    world = make_world(
        phase=TrafficPhase.PEDESTRIAN_WALK,
        vehicle_position=Vec2(0, 0),
        vehicle_speed_mps=2,
    )
    command = SafetyPolicy().evaluate(world).vehicle_commands["car-1"]
    assert not command.should_stop
    assert command.reason is VehicleDecisionReason.CLEAR_TO_PROCEED


def test_vehicle_follows_a_stopped_leader() -> None:
    world = make_world(phase=TrafficPhase.VEHICLE_GREEN, vehicle_position=Vec2(-25, 0))
    world.vehicles["car-1"].speed_mps = 6
    world.vehicles["car-2"] = VehicleState(
        actor_id="car-2",
        position=Vec2(-15, 0),
        heading=Vec2(1, 0),
        speed_mps=0,
    )

    command = SafetyPolicy().evaluate(world).vehicle_commands["car-1"]

    assert command.should_stop
    assert command.reason is VehicleDecisionReason.FOLLOWING_TRAFFIC


def test_vehicle_does_not_follow_a_car_in_an_adjacent_lane() -> None:
    world = make_world(phase=TrafficPhase.VEHICLE_GREEN, vehicle_position=Vec2(-25, 0))
    world.vehicles["car-2"] = VehicleState(
        actor_id="car-2",
        position=Vec2(-15, 3.5),
        heading=Vec2(1, 0),
        speed_mps=0,
    )

    command = SafetyPolicy().evaluate(world).vehicle_commands["car-1"]

    assert not command.should_stop

