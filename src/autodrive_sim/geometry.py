"""Small 2D geometry primitives used by the deterministic simulation core."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True, slots=True)
class Vec2:
    """A two-dimensional vector with the operations needed by the core model."""

    x: float
    y: float

    def __add__(self, other: Vec2) -> Vec2:
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vec2) -> Vec2:
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> Vec2:
        return Vec2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> Vec2:
        return self * scalar

    def __truediv__(self, scalar: float) -> Vec2:
        if scalar == 0:
            raise ZeroDivisionError("cannot divide a vector by zero")
        return Vec2(self.x / scalar, self.y / scalar)

    def dot(self, other: Vec2) -> float:
        return self.x * other.x + self.y * other.y

    def length(self) -> float:
        return math.hypot(self.x, self.y)

    def distance_to(self, other: Vec2) -> float:
        return (self - other).length()

    def normalized(self) -> Vec2:
        magnitude = self.length()
        if magnitude == 0:
            raise ValueError("the zero vector has no direction")
        return self / magnitude


@dataclass(frozen=True, slots=True)
class Rect:
    """An axis-aligned rectangle represented by a center and half-extents."""

    center: Vec2
    half_extents: Vec2

    def __post_init__(self) -> None:
        if self.half_extents.x < 0 or self.half_extents.y < 0:
            raise ValueError("rectangle half-extents must be non-negative")

    def contains(self, point: Vec2) -> bool:
        offset = point - self.center
        return (
            abs(offset.x) <= self.half_extents.x
            and abs(offset.y) <= self.half_extents.y
        )

    def expanded(self, margin: float) -> Rect:
        if margin < 0:
            raise ValueError("rectangle expansion margin must be non-negative")
        return Rect(
            center=self.center,
            half_extents=Vec2(
                self.half_extents.x + margin,
                self.half_extents.y + margin,
            ),
        )

    def intersects(self, other: Rect) -> bool:
        center_offset = self.center - other.center
        return (
            abs(center_offset.x) <= self.half_extents.x + other.half_extents.x
            and abs(center_offset.y) <= self.half_extents.y + other.half_extents.y
        )

