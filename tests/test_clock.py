import pytest

from autodrive_sim.clock import SimulationClock


def test_clock_advances_fixed_steps() -> None:
    clock = SimulationClock(fixed_delta_s=0.1)
    assert clock.tick() == pytest.approx(0.1)
    assert clock.tick(steps=2) == pytest.approx(0.3)
    assert clock.frame == 3


def test_clock_rejects_invalid_steps() -> None:
    with pytest.raises(ValueError):
        SimulationClock(fixed_delta_s=0).tick()
    with pytest.raises(ValueError):
        SimulationClock().tick(steps=0)

