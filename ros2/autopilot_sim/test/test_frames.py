import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from autopilot_sim import frames  # noqa: E402


def q_from_euler(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_level_north_is_ros_yaw_90():
    q = frames.quat_ned_frd_to_enu_flu((1, 0, 0, 0))
    assert abs(frames.yaw_enu_deg(q) - 90.0) < 1e-6


def test_facing_east_is_ros_yaw_0():
    q = frames.quat_ned_frd_to_enu_flu(q_from_euler(0, 0, math.radians(90)))
    assert abs(frames.yaw_enu_deg(q)) < 1e-6


def test_body_x_axis_maps_to_same_world_direction():
    # nose 30 deg up, heading 40 deg east of north: the nose direction must agree in both conventions
    q_ned = q_from_euler(math.radians(10), math.radians(30), math.radians(40))
    nose_ned = [row[0] for row in frames.quat_to_matrix(q_ned)]
    q_enu = frames.quat_ned_frd_to_enu_flu(q_ned)
    nose_enu = [row[0] for row in frames.quat_to_matrix(q_enu)]
    assert close(frames.ned_to_enu(nose_ned), nose_enu)
    # and the body "right" axis (FRD +y) is FLU -y
    right_ned = [row[1] for row in frames.quat_to_matrix(q_ned)]
    left_enu = [row[1] for row in frames.quat_to_matrix(q_enu)]
    assert close(frames.ned_to_enu(right_ned), [-c for c in left_enu])


def test_accelerometer_at_rest_points_up_in_flu():
    assert frames.frd_to_flu((0, 0, -9.81)) == (0, 0, 9.81)


def test_world_velocity_into_body():
    q_enu = frames.quat_ned_frd_to_enu_flu((1, 0, 0, 0))  # facing north
    v_body = frames.enu_vector_to_flu_body(q_enu, frames.ned_to_enu((2.0, 0.0, 0.0)))  # moving north
    assert close(v_body, (2.0, 0.0, 0.0))
