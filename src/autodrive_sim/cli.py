"""Command-line entry point for Autonomy Traffic Lab."""

from __future__ import annotations

import argparse
import importlib.util
import json
import platform
import sys
from collections.abc import Sequence
from pathlib import Path

from . import __version__
from .carla_backend import CarlaBackend, CarlaConnectionConfig, CarlaSpawnError, CarlaUnavailableError
from .clock import SimulationClock
from .config import ScenarioConfig
from .events import CompositeEventSink, EventSink, JsonlEventSink
from .metrics import MetricsCollector
from .policy import SafetyPolicy
from .runner import SimulationRunner
from .safety import SafetyMonitor
from .scenario import ScenarioFactory


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line parser."""
    parser = argparse.ArgumentParser(
        prog="autodrive-sim",
        description="Run and inspect the Autonomy Traffic Lab simulation.",
    )
    parser.add_argument("--version", action="version", version=__version__)

    subparsers = parser.add_subparsers(dest="command")
    subparsers.add_parser(
        "doctor",
        help="Check the local Python environment and optional simulator dependencies.",
    )
    run_parser = subparsers.add_parser("run", help="Run a configured traffic scenario.")
    run_parser.add_argument(
        "--scenario",
        type=Path,
        default=Path("scenarios/urban_crosswalk.toml"),
        help="Path to a TOML scenario file.",
    )
    run_parser.add_argument("--steps", type=int, default=100, help="Number of fixed steps to run.")
    run_parser.add_argument(
        "--delta-s",
        type=float,
        default=0.05,
        help="Fixed simulation step in seconds.",
    )
    run_parser.add_argument(
        "--backend",
        choices=("kinematic", "carla"),
        default="kinematic",
        help="Simulation backend.",
    )
    run_parser.add_argument(
        "--events",
        type=Path,
        help="Optional JSONL event output path.",
    )
    run_parser.add_argument("--host", default="127.0.0.1", help="CARLA server host.")
    run_parser.add_argument("--port", type=int, default=2000, help="CARLA server port.")
    return parser


def _run_doctor() -> int:
    """Print environment diagnostics without requiring CARLA to be installed."""
    carla_available = importlib.util.find_spec("carla") is not None
    print(f"Autonomy Traffic Lab {__version__}")
    print(f"Python: {sys.version.split()[0]}")
    print(f"Platform: {platform.platform()}")
    print(f"CARLA Python API: {'available' if carla_available else 'not installed'}")
    print("Core simulator: ready")
    return 0


def _run_scenario(args: argparse.Namespace) -> int:
    if args.steps < 0:
        raise ValueError("--steps cannot be negative")
    config = ScenarioConfig.from_toml(args.scenario)
    built = ScenarioFactory().build(config)
    metrics = MetricsCollector()
    sinks: list[EventSink] = [metrics]
    if args.events is not None:
        args.events.parent.mkdir(parents=True, exist_ok=True)
        sinks.append(JsonlEventSink(args.events))
    event_sink = CompositeEventSink(tuple(sinks))

    backend = built.world
    carla_backend: CarlaBackend | None = None
    if args.backend == "carla":
        carla_backend = CarlaBackend.connect(
            CarlaConnectionConfig(host=args.host, port=args.port, fixed_delta_s=args.delta_s)
        )
        carla_backend.state.crosswalks.update(built.world.state.crosswalks)
        carla_backend.state.signals.update(built.world.state.signals)
        for vehicle in built.world.state.vehicles.values():
            carla_backend.spawn_vehicle(vehicle)
        for pedestrian in built.world.state.pedestrians.values():
            carla_backend.spawn_pedestrian(pedestrian)
        backend = carla_backend

    try:
        runner = SimulationRunner(
            world=backend,
            policy=SafetyPolicy(),
            clock=SimulationClock(fixed_delta_s=args.delta_s),
            signal_controllers=built.signal_controllers,
            event_sink=event_sink,
            metrics=metrics,
            safety_monitor=SafetyMonitor(),
        )
        runner.run_steps(args.steps)
        report = {
            "backend": args.backend,
            "scenario": config.name,
            "steps": args.steps,
            "metrics": metrics.report().as_dict(),
        }
        print(json.dumps(report, sort_keys=True))
    finally:
        if carla_backend is not None:
            carla_backend.close()
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    """Run the CLI and return a process exit code."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command == "doctor":
        return _run_doctor()
    if args.command == "run":
        try:
            return _run_scenario(args)
        except (CarlaSpawnError, CarlaUnavailableError, ValueError) as error:
            print(f"error: {error}", file=sys.stderr)
            return 2
    parser.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

