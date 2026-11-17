"""Simulation orchestration for deterministic and CARLA-compatible backends."""

from __future__ import annotations

from dataclasses import dataclass, field

from .backend import WorldBackend
from .clock import SimulationClock
from .events import EventSink, EventType, SimulationEvent
from .metrics import MetricsCollector
from .models import PolicyDecision
from .policy import SafetyPolicy
from .safety import SafetyMonitor
from .signals import SignalController


@dataclass(frozen=True, slots=True)
class SimulationFrame:
    """The decision produced and time reached by one simulation step."""

    frame: int
    time_s: float
    decision: PolicyDecision


@dataclass(slots=True)
class SimulationRunner:
    """Coordinate policy, backend, signals, and fixed-step time."""

    world: WorldBackend
    policy: SafetyPolicy = field(default_factory=SafetyPolicy)
    clock: SimulationClock = field(default_factory=SimulationClock)
    signal_controllers: list[SignalController] = field(default_factory=list)
    event_sink: EventSink | None = None
    metrics: MetricsCollector | None = None
    safety_monitor: SafetyMonitor | None = None

    def step(self) -> SimulationFrame:
        """Evaluate and apply exactly one fixed simulation step."""
        state = self.world.snapshot()
        decision = self.policy.evaluate(state)
        self._record_policy_decisions(state.time_s, decision)
        phases_before = {
            controller.signal_id: state.signals[controller.signal_id].phase.value
            for controller in self.signal_controllers
        }
        self.world.step(self.clock.fixed_delta_s, decision)
        for controller in self.signal_controllers:
            controller.tick(self.world.snapshot(), self.clock.fixed_delta_s)
        if self.metrics is not None:
            self.metrics.observe(self.world.snapshot(), self.clock.fixed_delta_s)
        if self.event_sink is not None and self.safety_monitor is not None:
            for event in self.safety_monitor.inspect(self.world.snapshot()):
                self.event_sink.record(event)
        self._record_signal_changes(phases_before, self.world.snapshot().time_s)
        time_s = self.clock.tick()
        return SimulationFrame(
            frame=self.clock.frame,
            time_s=time_s,
            decision=decision,
        )

    def run_steps(self, count: int) -> list[SimulationFrame]:
        """Run a fixed number of steps and return their decisions."""
        if count < 0:
            raise ValueError("simulation step count cannot be negative")
        return [self.step() for _ in range(count)]

    def run_for(self, duration_s: float) -> list[SimulationFrame]:
        """Run an exact duration that is divisible by the fixed step."""
        if duration_s < 0:
            raise ValueError("simulation duration cannot be negative")
        steps = duration_s / self.clock.fixed_delta_s
        rounded_steps = round(steps)
        if abs(steps - rounded_steps) > 1e-9:
            raise ValueError("duration must be divisible by the fixed simulation step")
        return self.run_steps(rounded_steps)

    def _record_policy_decisions(self, time_s: float, decision: PolicyDecision) -> None:
        if self.event_sink is None:
            return
        for actor_id, command in decision.vehicle_commands.items():
            self.event_sink.record(
                SimulationEvent(
                    time_s=time_s,
                    event_type=EventType.POLICY_DECISION,
                    actor_id=actor_id,
                    payload={
                        "actor_kind": "vehicle",
                        "should_stop": command.should_stop,
                        "target_speed_mps": command.target_speed_mps,
                        "reason": command.reason.value,
                    },
                )
            )
        for actor_id, command in decision.pedestrian_commands.items():
            self.event_sink.record(
                SimulationEvent(
                    time_s=time_s,
                    event_type=EventType.POLICY_DECISION,
                    actor_id=actor_id,
                    payload={
                        "actor_kind": "pedestrian",
                        "may_enter_crosswalk": command.may_enter_crosswalk,
                        "reason": command.reason.value,
                    },
                )
            )

    def _record_signal_changes(self, phases_before: dict[str, str], time_s: float) -> None:
        if self.event_sink is None:
            return
        state = self.world.snapshot()
        for signal_id, phase_before in phases_before.items():
            phase_after = state.signals[signal_id].phase.value
            if phase_before == phase_after:
                continue
            self.event_sink.record(
                SimulationEvent(
                    time_s=time_s,
                    event_type=EventType.SIGNAL_PHASE_CHANGED,
                    actor_id=signal_id,
                    payload={"from": phase_before, "to": phase_after},
                )
            )

