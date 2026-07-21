"""OpenCV hand-eye solving with explicit ROS-TF to OpenCV conversion.

The ROS C10 workflow stores ``base -> tool0`` and
``camera_color_optical_frame -> marker``.  Those are the forward-pose
conventions accepted by the OpenCV Python binding in this workflow, which
returns the deployment transform directly as
``tool0 -> camera_color_optical_frame``.  The adapter keeps that convention
explicit and independent of easy_handeye2's service implementation.
"""

import math

import cv2
import numpy as np

from .calibration_quality import transform_components


_METHODS = {
    'OpenCV/Park': cv2.CALIB_HAND_EYE_PARK,
    'OpenCV/Horaud': cv2.CALIB_HAND_EYE_HORAUD,
    'OpenCV/Tsai-Lenz': cv2.CALIB_HAND_EYE_TSAI,
}


def _unit_quaternion(quaternion):
    values = np.asarray(quaternion, dtype=float)
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError('quaternion is invalid')
    return values / norm


def quaternion_matrix(quaternion):
    """Return a 3x3 rotation matrix from ROS ``(x, y, z, w)`` quaternion."""
    x, y, z, w = _unit_quaternion(quaternion)
    return np.array((
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)),
    ), dtype=float)


def matrix_quaternion(matrix):
    """Return a normalized ROS ``(x, y, z, w)`` quaternion from a matrix."""
    matrix = np.asarray(matrix, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        quaternion = (
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        )
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            scale = 2.0 * math.sqrt(max(0.0, 1.0 + matrix[0, 0]
                                        - matrix[1, 1] - matrix[2, 2]))
            quaternion = (
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
            )
        elif axis == 1:
            scale = 2.0 * math.sqrt(max(0.0, 1.0 + matrix[1, 1]
                                        - matrix[0, 0] - matrix[2, 2]))
            quaternion = (
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
            )
        else:
            scale = 2.0 * math.sqrt(max(0.0, 1.0 + matrix[2, 2]
                                        - matrix[0, 0] - matrix[1, 1]))
            quaternion = (
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            )
    return tuple(float(value) for value in _unit_quaternion(quaternion))


def invert_transform(transform):
    """Invert a ``((x, y, z), (qx, qy, qz, qw))`` rigid transform."""
    translation, quaternion = transform
    rotation = quaternion_matrix(quaternion)
    inverse_rotation = rotation.T
    inverse_translation = -inverse_rotation @ np.asarray(translation, dtype=float)
    return (
        tuple(float(value) for value in inverse_translation),
        matrix_quaternion(inverse_rotation),
    )


def solve_handeye(samples, algorithm_names):
    """Return ``{algorithm: tool0_to_camera}`` from ROS forward-pose samples."""
    if len(samples) < 3:
        raise ValueError('at least three samples are required for hand-eye solving')
    gripper_to_base_rotations, gripper_to_base_translations = [], []
    target_to_camera_rotations, target_to_camera_translations = [], []
    for sample in samples:
        base_to_tool = transform_components(sample.robot)
        camera_to_marker = transform_components(sample.tracking)
        gripper_to_base_rotations.append(quaternion_matrix(base_to_tool[1]))
        gripper_to_base_translations.append(np.asarray(base_to_tool[0], dtype=float))
        target_to_camera_rotations.append(quaternion_matrix(camera_to_marker[1]))
        target_to_camera_translations.append(
            np.asarray(camera_to_marker[0], dtype=float))

    results = {}
    for algorithm in algorithm_names:
        algorithm = str(algorithm)
        try:
            method = _METHODS[algorithm]
        except KeyError as error:
            raise ValueError(f'unsupported hand-eye algorithm: {algorithm}') from error
        tool_to_camera_rotation, tool_to_camera_translation = cv2.calibrateHandEye(
            gripper_to_base_rotations,
            gripper_to_base_translations,
            target_to_camera_rotations,
            target_to_camera_translations,
            method=method)
        results[algorithm] = (
            tuple(float(value) for value in np.asarray(
                tool_to_camera_translation, dtype=float).reshape(3)),
            matrix_quaternion(tool_to_camera_rotation),
        )
    return results
