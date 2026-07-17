"""Small geometry helpers for eye-in-hand tree observation."""

import math


def transform_point(point, translation, quat_xyzw):
    """Apply a rigid transform to a point without a tf2_geometry dependency."""
    px, py, pz = (float(value) for value in point)
    tx, ty, tz = (float(value) for value in translation)
    qx, qy, qz, qw = (float(value) for value in quat_xyzw)
    values = (px, py, pz, tx, ty, tz, qx, qy, qz, qw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('transform values must be finite')
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-9:
        raise ValueError('transform quaternion is invalid')
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return (
        tx + (1.0 - 2.0 * (yy + zz)) * px + 2.0 * (xy - wz) * py + 2.0 * (xz + wy) * pz,
        ty + 2.0 * (xy + wz) * px + (1.0 - 2.0 * (xx + zz)) * py + 2.0 * (yz - wx) * pz,
        tz + 2.0 * (xz - wy) * px + 2.0 * (yz + wx) * py + (1.0 - 2.0 * (xx + yy)) * pz,
    )


def camera_look_at_pose(
        tree_root, aim_height, camera_height, observation_distance,
        azimuth_offset_degrees=0.0):
    """Return a high camera pose whose optical +Z aims at the crown."""
    tx, ty, tz = (float(value) for value in tree_root)
    aim_height = float(aim_height)
    camera_height = float(camera_height)
    observation_distance = float(observation_distance)
    azimuth_offset_degrees = float(azimuth_offset_degrees)
    if not all(math.isfinite(value) for value in (
            tx, ty, tz, aim_height, camera_height, observation_distance,
            azimuth_offset_degrees)):
        raise ValueError('observation values must be finite')
    if aim_height <= 0.0 or camera_height <= 0.0 or observation_distance <= 0.0:
        raise ValueError('observation heights and distance must be positive')
    horizontal_distance = math.hypot(tx, ty)
    if horizontal_distance <= observation_distance + 0.05:
        raise ValueError('tree hint is too close for the requested observation distance')

    bearing = math.atan2(ty, tx) + math.radians(azimuth_offset_degrees)
    forward_x = math.cos(bearing)
    forward_y = math.sin(bearing)
    camera = (
        tx - observation_distance * forward_x,
        ty - observation_distance * forward_y,
        tz + camera_height,
    )
    target_delta = (
        tx - camera[0],
        ty - camera[1],
        tz + aim_height - camera[2],
    )
    target_distance = math.sqrt(sum(value * value for value in target_delta))
    optical_z = tuple(value / target_distance for value in target_delta)
    optical_x = (forward_y, -forward_x, 0.0)
    optical_y = (
        optical_z[1] * optical_x[2] - optical_z[2] * optical_x[1],
        optical_z[2] * optical_x[0] - optical_z[0] * optical_x[2],
        optical_z[0] * optical_x[1] - optical_z[1] * optical_x[0],
    )
    # Optical frame: +Z forward, +X right, +Y down. Matrix columns are the
    # optical X/Y/Z axes expressed in the arm-base frame.
    matrix = (
        (optical_x[0], optical_y[0], optical_z[0]),
        (optical_x[1], optical_y[1], optical_z[1]),
        (optical_x[2], optical_y[2], optical_z[2]),
    )
    return camera, quaternion_from_matrix(matrix)


def tool_pose_from_camera_pose(
        camera_position, camera_quat_xyzw,
        tool_to_camera_translation, tool_to_camera_quat_xyzw):
    """Convert an optical-camera goal into its parent tool0 goal.

    MoveIt plans the SRDF arm tip (tool0), while the fixed C10 frame remains
    the frame that defines the required observation direction.
    """
    camera_position = tuple(float(value) for value in camera_position)
    camera_quat_xyzw = normalize_quaternion(camera_quat_xyzw)
    tool_to_camera_translation = tuple(
        float(value) for value in tool_to_camera_translation)
    tool_to_camera_quat_xyzw = normalize_quaternion(tool_to_camera_quat_xyzw)
    camera_to_tool_quat = quaternion_conjugate(tool_to_camera_quat_xyzw)
    camera_to_tool_translation = rotate_vector(
        tuple(-value for value in tool_to_camera_translation),
        camera_to_tool_quat)
    tool_position = tuple(
        camera_position[index] + rotate_vector(
            camera_to_tool_translation, camera_quat_xyzw)[index]
        for index in range(3))
    return tool_position, quaternion_multiply(
        camera_quat_xyzw, camera_to_tool_quat)


def normalize_quaternion(quat_xyzw):
    values = tuple(float(value) for value in quat_xyzw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('quaternion values must be finite')
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        raise ValueError('quaternion is invalid')
    return tuple(value / norm for value in values)


def quaternion_conjugate(quat_xyzw):
    x, y, z, w = quat_xyzw
    return -x, -y, -z, w


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def yaw_rotate_quaternion(quat_xyzw, yaw_degrees):
    """Rotate an optical-camera orientation around the arm-base vertical axis."""
    yaw = math.radians(float(yaw_degrees)) / 2.0
    return quaternion_multiply(
        (0.0, 0.0, math.sin(yaw), math.cos(yaw)),
        quat_xyzw)


def rotate_vector(vector, quat_xyzw):
    x, y, z, w = normalize_quaternion(quat_xyzw)
    vx, vy, vz = (float(value) for value in vector)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz)) * vx + 2.0 * (xy - wz) * vy + 2.0 * (xz + wy) * vz,
        2.0 * (xy + wz) * vx + (1.0 - 2.0 * (xx + zz)) * vy + 2.0 * (yz - wx) * vz,
        2.0 * (xz - wy) * vx + 2.0 * (yz + wx) * vy + (1.0 - 2.0 * (xx + yy)) * vz,
    )


def quaternion_from_matrix(matrix):
    """Convert a proper 3x3 rotation matrix to an XYZW quaternion."""
    (m00, m01, m02), (m10, m11, m12), (m20, m21, m22) = matrix
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (m21 - m12) / scale
        qy = (m02 - m20) / scale
        qz = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / scale
        qx = 0.25 * scale
        qy = (m01 + m10) / scale
        qz = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / scale
        qx = (m01 + m10) / scale
        qy = 0.25 * scale
        qz = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / scale
        qx = (m02 + m20) / scale
        qy = (m12 + m21) / scale
        qz = 0.25 * scale
    return qx, qy, qz, qw
