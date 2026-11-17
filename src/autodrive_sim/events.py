"""Structured simulation events and pluggable event sinks."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
import json
from pathlib import Path
from typing import Any, Protocol


class EventType(StrEnum):
    """Event categories emitted by the simulation."""

    POLICY_DECISION = "policy_decision"
    ACTOR_SPAWNED = "actor_spawned"
    SIGNAL_PHASE_CHANGED = "signal_phase_changed"
    SAFETY_VIOLATION = "safety_violation"


@dataclass(frozen=True, slots=True)
class SimulationEvent:
    """One timestamped, JSON-serializable simulation event."""

    time_s: float
    event_type: EventType
    payload: dict[str, Any] = field(default_factory=dict)
    actor_id: str | None = None

    def as_dict(self) -> dict[str, Any]:
        """Return a stable representation for logs and downstream analysis."""
        return {
            "time_s": self.time_s,
            "event_type": self.event_type.value,
            "actor_id": self.actor_id,
            "payload": self.payload,
        }


class EventSink(Protocol):
    """Minimal interface for memory, JSONL, or future streaming sinks."""

    def record(self, event: SimulationEvent) -> None:
        """Persist one event."""


@dataclass(slots=True)
class MemoryEventSink:
    """Collect events for assertions and in-process analysis."""

    events: list[SimulationEvent] = field(default_factory=list)

    def record(self, event: SimulationEvent) -> None:
        self.events.append(event)


@dataclass(slots=True)
class CompositeEventSink:
    """Fan one event out to multiple independent sinks."""

    sinks: tuple[EventSink, ...]

    def record(self, event: SimulationEvent) -> None:
        for sink in self.sinks:
            sink.record(event)


@dataclass(slots=True)
class JsonlEventSink:
    """Append one JSON object per line for replay and analysis tools."""

    path: Path

    def record(self, event: SimulationEvent) -> None:
        with self.path.open("a", encoding="utf-8") as file:
            json.dump(event.as_dict(), file, sort_keys=True)
            file.write("\n")

