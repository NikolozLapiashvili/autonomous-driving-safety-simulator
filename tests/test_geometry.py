import pytest

from autodrive_sim.geometry import Rect, Vec2


def test_vec2_operations_are_predictable() -> None:
    vector = Vec2(3, 4)
    assert vector.length() == 5
    assert vector.normalized() == Vec2(0.6, 0.8)
    assert vector + Vec2(1, 2) == Vec2(4, 6)
    assert vector.dot(Vec2(1, 0)) == 3


def test_rect_contains_and_intersects() -> None:
    first = Rect(center=Vec2(0, 0), half_extents=Vec2(2, 1))
    second = Rect(center=Vec2(3, 0), half_extents=Vec2(1, 1))
    assert first.contains(Vec2(2, 1))
    assert first.intersects(second)
    assert first.expanded(1).contains(Vec2(3, 0))


def test_zero_vector_has_no_direction() -> None:
    with pytest.raises(ValueError):
        Vec2(0, 0).normalized()

