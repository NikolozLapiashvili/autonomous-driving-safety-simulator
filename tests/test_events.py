import json
from pathlib import Path

from autodrive_sim.events import (
    CompositeEventSink,
    EventType,
    JsonlEventSink,
    MemoryEventSink,
    SimulationEvent,
)


def test_memory_event_sink_preserves_event_order() -> None:
    sink = MemoryEventSink()
    first = SimulationEvent(0.0, EventType.ACTOR_SPAWNED, {"kind": "vehicle"}, "car-1")
    second = SimulationEvent(0.1, EventType.POLICY_DECISION, {"stop": True}, "car-1")

    sink.record(first)
    sink.record(second)

    assert sink.events == [first, second]


def test_jsonl_event_sink_writes_structured_record(tmp_path: Path) -> None:
    path = tmp_path / "events.jsonl"
    sink = JsonlEventSink(path)
    sink.record(SimulationEvent(1.25, EventType.SAFETY_VIOLATION, {"kind": "collision"}))

    record = json.loads(path.read_text(encoding="utf-8"))
    assert record == {
        "actor_id": None,
        "event_type": "safety_violation",
        "payload": {"kind": "collision"},
        "time_s": 1.25,
    }


def test_composite_event_sink_fans_out() -> None:
    first = MemoryEventSink()
    second = MemoryEventSink()
    event = SimulationEvent(0.0, EventType.ACTOR_SPAWNED)

    CompositeEventSink((first, second)).record(event)

    assert first.events == [event]
    assert second.events == [event]

