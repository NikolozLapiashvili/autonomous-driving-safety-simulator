"""Command-line entry point for Autonomy Traffic Lab."""

from __future__ import annotations

import argparse
import importlib.util
import platform
import sys
from collections.abc import Sequence

from . import __version__


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


def main(argv: Sequence[str] | None = None) -> int:
    """Run the CLI and return a process exit code."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command == "doctor":
        return _run_doctor()
    parser.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

