"""Pose and controller mapping shared by the XRoboToolkit ROS bridge.

The public XRoboToolkit Python SDK returns controller poses as
``[x, y, z, qx, qy, qz, qw]``.  The default coordinate conversion is the
headset-to-robot mapping used by both the old RemoteCtrl Quest reader and the
official XRoboToolkit teleoperation examples.
"""

import math


DEFAULT_HEADSET_TO_ROBOT = (
    0.0, 0.0, -1.0,
    -1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
)

BUTTON_ORDER = (
    "X", "Y", "A", "B", "LThU", "RThU",
    "LJ", "RJ", "LG", "RG", "LTr", "RTr",
)


def _clamp(value, low, high):
    return max(low, min(high, float(value)))


def _mat_vec(matrix, vector):
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def _mat_mul(a, b):
    return tuple(
        tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )


def _transpose(matrix):
    return tuple(tuple(matrix[col][row] for col in range(3)) for row in range(3))


def _determinant(matrix):
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


def matrix_3x3(values):
    """Build and validate a proper 3x3 rotation matrix from nine values."""
    if len(values) == 3 and all(len(row) == 3 for row in values):
        values = tuple(value for row in values for value in row)
    values = tuple(float(value) for value in values)
    if len(values) != 9:
        raise ValueError("mapping must contain exactly 9 row-major values")
    matrix = (values[0:3], values[3:6], values[6:9])
    if not all(math.isfinite(value) for value in values):
        raise ValueError("mapping contains a non-finite value")
    if abs(_determinant(matrix) - 1.0) > 1e-4:
        raise ValueError("mapping must be a right-handed rotation matrix")
    transpose_product = _mat_mul(_transpose(matrix), matrix)
    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    if any(
        abs(transpose_product[row][col] - identity[row][col]) > 1e-4
        for row in range(3)
        for col in range(3)
    ):
        raise ValueError("mapping must be orthonormal")
    return matrix


def normalize_quaternion(quaternion):
    """Return a unit quaternion in ``(w, x, y, z)`` order."""
    values = tuple(float(value) for value in quaternion)
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must contain four finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    return tuple(value / norm for value in values)


def quaternion_conjugate(quaternion):
    w, x, y, z = quaternion
    return (w, -x, -y, -z)


def quaternion_multiply(first, second):
    """Hamilton product ``first * second`` for ``(w, x, y, z)`` quaternions."""
    aw, ax, ay, az = first
    bw, bx, by, bz = second
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quaternion_from_matrix(matrix):
    """Convert a proper 3x3 rotation matrix to ``(w, x, y, z)``."""
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        return normalize_quaternion((
            0.25 * scale,
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
        ))
    if m00 > m11 and m00 > m22:
        scale = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        return normalize_quaternion((
            (m21 - m12) / scale,
            0.25 * scale,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
        ))
    if m11 > m22:
        scale = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        return normalize_quaternion((
            (m02 - m20) / scale,
            (m01 + m10) / scale,
            0.25 * scale,
            (m12 + m21) / scale,
        ))
    scale = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
    return normalize_quaternion((
        (m10 - m01) / scale,
        (m02 + m20) / scale,
        (m12 + m21) / scale,
        0.25 * scale,
    ))


def quaternion_to_matrix(quaternion):
    """Convert a ``(w, x, y, z)`` quaternion to a 3x3 rotation matrix."""
    w, x, y, z = normalize_quaternion(quaternion)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
        (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
        (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
    )


def raw_pose_is_valid(raw_pose):
    """Reject the all-zero sentinel returned before a controller is tracked."""
    try:
        values = tuple(float(value) for value in raw_pose)
    except (TypeError, ValueError):
        return False
    if len(values) < 7 or not all(math.isfinite(value) for value in values[:7]):
        return False
    quaternion_norm = math.sqrt(sum(value * value for value in values[3:7]))
    return quaternion_norm > 0.5


def transform_pose(
    raw_pose,
    mapping=DEFAULT_HEADSET_TO_ROBOT,
    scale=1.0,
    base_translation=(0.0, 0.0, 0.0),
    base_rotation=(1.0, 0.0, 0.0, 0.0),
):
    """Map an XRoboToolkit pose to the robot base without filtering it."""
    if not raw_pose_is_valid(raw_pose):
        raise ValueError("invalid or untracked controller pose")
    matrix = matrix_3x3(mapping)
    position = tuple(float(value) * float(scale) for value in raw_pose[0:3])
    source_quaternion = normalize_quaternion(
        (raw_pose[6], raw_pose[3], raw_pose[4], raw_pose[5])
    )

    mapped_position = _mat_vec(matrix, position)
    mapping_quaternion = quaternion_from_matrix(matrix)
    mapped_quaternion = quaternion_multiply(
        quaternion_multiply(mapping_quaternion, source_quaternion),
        quaternion_conjugate(mapping_quaternion),
    )

    base_matrix = quaternion_to_matrix(base_rotation)
    rotated_position = _mat_vec(base_matrix, mapped_position)
    output_position = tuple(
        rotated_position[index] + float(base_translation[index])
        for index in range(3)
    )
    output_quaternion = quaternion_multiply(
        normalize_quaternion(base_rotation),
        mapped_quaternion,
    )
    return output_position, normalize_quaternion(output_quaternion)


def smooth_pose(previous, position, quaternion, alpha):
    """Apply the same first-order low-pass behaviour as the old Quest reader."""
    alpha = _clamp(alpha, 0.0, 1.0)
    quaternion = normalize_quaternion(quaternion)
    if previous is None or alpha <= 0.0:
        return tuple(position), quaternion

    previous_position, previous_quaternion = previous
    if sum(a * b for a, b in zip(previous_quaternion, quaternion)) < 0.0:
        quaternion = tuple(-value for value in quaternion)
    filtered_position = tuple(
        old + alpha * (new - old)
        for old, new in zip(previous_position, position)
    )
    filtered_quaternion = normalize_quaternion(tuple(
        old + alpha * (new - old)
        for old, new in zip(previous_quaternion, quaternion)
    ))
    return filtered_position, filtered_quaternion


def _axis_pair(value):
    try:
        return (_clamp(value[0], -1.0, 1.0), _clamp(value[1], -1.0, 1.0))
    except (IndexError, TypeError, ValueError):
        return (0.0, 0.0)


def joy_payload(left, right, grip_threshold=0.5):
    """Create the stable Joy layout consumed by the copied JAKA and AMR nodes."""
    left_grip = _clamp(left.get("grip", 0.0), 0.0, 1.0)
    right_grip = _clamp(right.get("grip", 0.0), 0.0, 1.0)
    left_trigger = _clamp(left.get("trigger", 0.0), 0.0, 1.0)
    right_trigger = _clamp(right.get("trigger", 0.0), 0.0, 1.0)
    left_axis = _axis_pair(left.get("axis", (0.0, 0.0)))
    right_axis = _axis_pair(right.get("axis", (0.0, 0.0)))
    axes = [
        left_grip,
        right_grip,
        left_trigger,
        right_trigger,
        left_axis[0],
        left_axis[1],
        right_axis[0],
        right_axis[1],
    ]

    button_values = {
        "X": left.get("X", False),
        "Y": left.get("Y", False),
        "A": right.get("A", False),
        "B": right.get("B", False),
        "LThU": left.get("menu", False),
        "RThU": right.get("menu", False),
        "LJ": left.get("axis_click", False),
        "RJ": right.get("axis_click", False),
        "LG": left_grip > grip_threshold,
        "RG": right_grip > grip_threshold,
        "LTr": left_trigger > grip_threshold,
        "RTr": right_trigger > grip_threshold,
    }
    buttons = [1 if button_values[name] else 0 for name in BUTTON_ORDER]
    return axes, buttons
