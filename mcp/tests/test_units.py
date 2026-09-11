import pytest

from gripper_mcp.units import (
    GripperGeometry,
    JointGeometry,
    Stroke,
    knuckle_rad_to_opening_mm,
    opening_mm_to_fraction,
    opening_mm_to_knuckle_rad,
)

STROKE_2F_85 = Stroke(max_opening_mm=85.0, closed_tolerance_mm=1.5)
KNUCKLE_2F_85 = JointGeometry(
    name="robotiq_85_left_knuckle_joint", rad_open=0.0, rad_closed=0.8
)
GEOMETRY_2F_85 = GripperGeometry.of(STROKE_2F_85, KNUCKLE_2F_85)


def test_a_geometry_is_the_datasheet_stroke_on_the_robot_joint():
    assert GEOMETRY_2F_85 == GripperGeometry(
        max_opening_mm=85.0,
        min_opening_mm=0.0,
        knuckle_rad_open=0.0,
        knuckle_rad_closed=0.8,
    )


def test_a_stroke_with_no_travel_is_rejected():
    with pytest.raises(ValueError, match="max_opening_mm"):
        Stroke(max_opening_mm=0.0, closed_tolerance_mm=1.5)


def test_a_joint_with_no_travel_is_rejected():
    with pytest.raises(ValueError, match="rad_closed"):
        JointGeometry(name="finger_joint", rad_open=0.7, rad_closed=0.7)


def test_a_joint_without_a_name_is_rejected():
    with pytest.raises(ValueError, match="name"):
        JointGeometry(name="", rad_open=0.0, rad_closed=0.7)


def test_a_geometry_with_no_stroke_is_rejected():
    with pytest.raises(ValueError, match="max_opening_mm"):
        GripperGeometry(
            max_opening_mm=0.0,
            min_opening_mm=0.0,
            knuckle_rad_open=0.0,
            knuckle_rad_closed=0.8,
        )


def test_a_geometry_with_no_joint_travel_is_rejected():
    with pytest.raises(ValueError, match="knuckle_rad_closed"):
        GripperGeometry(
            max_opening_mm=85.0,
            min_opening_mm=0.0,
            knuckle_rad_open=0.8,
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


@pytest.mark.parametrize("tolerance_mm", [0.0, -1.0, 85.0])
def test_a_closed_tolerance_outside_the_stroke_is_rejected(tolerance_mm):
    with pytest.raises(ValueError, match="closed_tolerance_mm"):
        Stroke(max_opening_mm=85.0, closed_tolerance_mm=tolerance_mm)


def test_the_fingers_have_met_within_the_closed_tolerance():
    assert STROKE_2F_85.fingers_met(1.4)
    assert not STROKE_2F_85.fingers_met(1.6)
