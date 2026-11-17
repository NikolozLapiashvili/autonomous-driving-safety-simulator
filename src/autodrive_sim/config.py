"""Scenario configuration loading and validation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import tomllib
from typing import Any


@dataclass(frozen=True, slots=True)
class ScenarioConfig:
    """All inputs needed to construct a deterministic traffic scenario."""

    name: str = "signalized_crosswalk"
    seed: int = 7
    vehicle_count: int = 12
    pedestrian_count: int = 6
    vehicle_speed_mps: float = 8.0
    vehicle_spacing_m: float = 12.0
    pedestrian_wait_distance_m: float = 4.0

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("scenario name cannot be empty")
        if self.seed < 0:
            raise ValueError("scenario seed cannot be negative")
        if self.vehicle_count < 0 or self.pedestrian_count < 0:
            raise ValueError("actor counts cannot be negative")
        if self.vehicle_count + self.pedestrian_count > 10_000:
            raise ValueError("scenario cannot contain more than 10,000 actors")
        if self.vehicle_speed_mps <= 0:
            raise ValueError("vehicle speed must be positive")
        if self.vehicle_spacing_m <= 0 or self.pedestrian_wait_distance_m <= 0:
            raise ValueError("spawn spacing values must be positive")

    @classmethod
    def from_mapping(cls, values: dict[str, Any]) -> ScenarioConfig:
        """Construct a config from a TOML ``[scenario]`` mapping."""
        allowed = {
            "name",
            "seed",
            "vehicle_count",
            "pedestrian_count",
            "vehicle_speed_mps",
            "vehicle_spacing_m",
            "pedestrian_wait_distance_m",
        }
        unknown = set(values) - allowed
        if unknown:
            unknown_names = ", ".join(sorted(unknown))
            raise ValueError(f"unknown scenario fields: {unknown_names}")
        return cls(**values)

    @classmethod
    def from_toml(cls, path: str | Path) -> ScenarioConfig:
        """Load a scenario from a TOML file."""
        scenario_path = Path(path)
        with scenario_path.open("rb") as file:
            document = tomllib.load(file)
        values = document.get("scenario")
        if not isinstance(values, dict):
            raise ValueError("TOML scenario file must contain a [scenario] table")
        return cls.from_mapping(values)

