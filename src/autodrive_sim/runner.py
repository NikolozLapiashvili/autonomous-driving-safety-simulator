"""Simulation orchestration for deterministic and CARLA-compatible backends."""

from __future__ import annotations

from dataclasses import dataclass, field

from .backend import WorldBackend
from .clock import SimulationClock
from .models import PolicyDecision
from .policy import SafetyPolicy
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

    def step(self) -> SimulationFrame:
        """Evaluate and apply exactly one fixed simulation step."""
        decision = self.policy.evaluate(self.world.snapshot())
        self.world.step(self.clock.fixed_delta_s, decision)
        for controller in self.signal_controllers:
            controller.tick(self.world.snapshot(), self.clock.fixed_delta_s)
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

