import pytest

from gripper_mcp.units import (
    GripperGeometry,
    knuckle_rad_to_opening_mm,
    opening_mm_to_fraction,
    opening_mm_to_knuckle_rad,
)

GEOMETRY_2F_85 = GripperGeometry(
    max_opening_mm=85.0,
    min_opening_mm=0.0,
    knuckle_rad_open=0.0,
    knuckle_rad_closed=0.8,
)


def test_fully_open_maps_to_zero_knuckle_angle():
    assert opening_mm_to_knuckle_rad(85.0, GEOMETRY_2F_85) == pytest.approx(0.0)


def test_fully_closed_maps_to_the_joint_upper_limit():
    assert opening_mm_to_knuckle_rad(0.0, GEOMETRY_2F_85) == pytest.approx(0.8)


def test_half_opening_is_half_the_fraction():
    assert opening_mm_to_fraction(42.5, GEOMETRY_2F_85) == pytest.approx(0.5)


def test_opening_beyond_the_stroke_clamps_to_the_ends():
    assert opening_mm_to_knuckle_rad(120.0, GEOMETRY_2F_85) == pytest.approx(0.0)
    assert opening_mm_to_knuckle_rad(-5.0, GEOMETRY_2F_85) == pytest.approx(0.8)


def test_knuckle_angle_beyond_the_stroke_clamps_to_the_ends():
    assert knuckle_rad_to_opening_mm(1.2, GEOMETRY_2F_85) == pytest.approx(0.0)
    assert knuckle_rad_to_opening_mm(-0.1, GEOMETRY_2F_85) == pytest.approx(85.0)


@pytest.mark.parametrize("opening_mm", [0.0, 12.5, 42.5, 70.0, 85.0])
def test_opening_round_trips_through_the_knuckle_angle(opening_mm):
    knuckle_rad = opening_mm_to_knuckle_rad(opening_mm, GEOMETRY_2F_85)

    assert knuckle_rad_to_opening_mm(knuckle_rad, GEOMETRY_2F_85) == pytest.approx(
        opening_mm
    )
