"""Frame conversions between the autopilot (NED world, FRD body) and ROS (ENU world, FLU body, REP 103).

Pure Python, no ROS imports, so it can be unit tested anywhere.
"""
import math

# NED -> ENU swaps x/y and flips z; FRD -> FLU flips y and z.
_NED_TO_ENU = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
_FLU_TO_FRD = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))


def ned_to_enu(v):
    """Vector in the NED world frame -> ENU."""
    return (v[1], v[0], -v[2])


def frd_to_flu(v):
    """Vector in the FRD body frame -> FLU."""
    return (v[0], -v[1], -v[2])


def _matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _transpose(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def quat_to_matrix(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def matrix_to_quat(m):
    """Rotation matrix -> quaternion (w, x, y, z), w >= 0."""
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)
    if q[0] < 0:
        q = tuple(-c for c in q)
    return q


def quat_ned_frd_to_enu_flu(q):
    """Attitude quaternion body(FRD)->NED (w,x,y,z) -> body(FLU)->ENU (w,x,y,z)."""
    r = _matmul(_matmul(_NED_TO_ENU, quat_to_matrix(q)), _FLU_TO_FRD)
    return matrix_to_quat(r)


def enu_vector_to_flu_body(q_enu_flu, v_enu):
    """Express a world (ENU) vector in the FLU body frame of attitude q_enu_flu."""
    rt = _transpose(quat_to_matrix(q_enu_flu))
    return tuple(sum(rt[i][k] * v_enu[k] for k in range(3)) for i in range(3))


def yaw_enu_deg(q_enu_flu):
    """Heading in ENU (0 = facing east, counter-clockwise positive), degrees."""
    m = quat_to_matrix(q_enu_flu)
    return math.degrees(math.atan2(m[1][0], m[0][0]))
