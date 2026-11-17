"""Command-line entry point for Autonomy Traffic Lab."""

from __future__ import annotations

import argparse
import importlib.util
import json
import platform
import sys
import time
from collections.abc import Sequence
from pathlib import Path

from . import __version__
from .carla_backend import (
    CarlaBackend,
    CarlaConnectionConfig,
    CarlaSpawnError,
    CarlaUnavailableError,
)
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
        "--live",
        action="store_true",
        help="Run CARLA in real time so the simulation can be watched live.",
    )
    run_parser.add_argument(
        "--compact-traffic",
        action="store_true",
        help="Place CARLA vehicles in a nearby group for live visual demos.",
    )
    run_parser.add_argument(
        "--traffic-manager",
        action="store_true",
        help="Use CARLA Traffic Manager for map-aware vehicle AI and yielding.",
    )
    run_parser.add_argument(
        "--tm-port",
        type=int,
        default=8000,
        help="CARLA Traffic Manager port.",
    )
    run_parser.add_argument(
        "--duration-s",
        type=float,
        default=60.0,
        help="Live-mode duration in seconds; use 0 to run until Ctrl+C.",
    )
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


def _run_live(runner: SimulationRunner, duration_s: float) -> None:
    """Advance a runner at wall-clock speed until the requested duration."""
    deadline = None if duration_s == 0 else time.monotonic() + duration_s
    next_tick = time.monotonic()
    while deadline is None or time.monotonic() < deadline:
        runner.step()
        next_tick += runner.clock.fixed_delta_s
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            # CARLA or the host fell behind. Reset the pacing origin instead
            # of trying to execute a burst of catch-up ticks.
            next_tick = time.monotonic()


def _run_scenario(args: argparse.Namespace) -> int:
    if args.steps < 0:
        raise ValueError("--steps cannot be negative")
    if args.duration_s < 0:
        raise ValueError("--duration-s cannot be negative")
    if args.live and args.backend != "carla":
        raise ValueError("--live requires --backend carla")
    if args.traffic_manager and args.backend != "carla":
        raise ValueError("--traffic-manager requires --backend carla")
    if not 1 <= args.tm_port <= 65_535:
        raise ValueError("--tm-port must be between 1 and 65535")
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
    try:
        if args.backend == "carla":
            carla_backend = CarlaBackend.connect(
                CarlaConnectionConfig(host=args.host, port=args.port, fixed_delta_s=args.delta_s),
                compact_traffic=args.compact_traffic,
                vehicle_count=config.vehicle_count,
                traffic_manager=args.traffic_manager,
                traffic_manager_port=args.tm_port,
                traffic_manager_seed=config.seed,
            )
            carla_backend.state.crosswalks.update(built.world.state.crosswalks)
            carla_backend.state.signals.update(built.world.state.signals)
            for vehicle in built.world.state.vehicles.values():
                carla_backend.spawn_vehicle(vehicle)
            for pedestrian in built.world.state.pedestrians.values():
                carla_backend.spawn_pedestrian(pedestrian)
            backend = carla_backend

        runner = SimulationRunner(
            world=backend,
            policy=SafetyPolicy(),
            clock=SimulationClock(fixed_delta_s=args.delta_s),
            signal_controllers=built.signal_controllers,
            event_sink=event_sink,
            metrics=metrics,
            safety_monitor=SafetyMonitor(),
        )
        if args.live:
            duration_text = "until Ctrl+C" if args.duration_s == 0 else f"for {args.duration_s:g}s"
            print(f"Live CARLA mode active {duration_text}. Press Ctrl+C to stop.")
            try:
                _run_live(runner, args.duration_s)
            except KeyboardInterrupt:
                print("\nLive CARLA run stopped.")
        else:
            runner.run_steps(args.steps)
        report = {
            "backend": args.backend,
            "scenario": config.name,
            "steps": runner.clock.frame,
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

