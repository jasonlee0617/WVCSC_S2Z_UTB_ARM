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


def _components_matrix(transform):
    """Return a homogeneous matrix from a ROS-transform component tuple."""
    translation, quaternion = transform
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = quaternion_matrix(quaternion)
    matrix[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return matrix


def _matrix_components(matrix):
    matrix = np.asarray(matrix, dtype=float)
    return (
        tuple(float(value) for value in matrix[:3, 3]),
        matrix_quaternion(matrix[:3, :3]),
    )


def _se3_increment(values):
    """Return the local six-vector increment used by the refinement loop."""
    values = np.asarray(values, dtype=float).reshape(6)
    matrix = np.eye(4, dtype=float)
    rotation, _jacobian = cv2.Rodrigues(values[3:].reshape(3, 1))
    matrix[:3, :3] = rotation
    matrix[:3, 3] = values[:3]
    return matrix


def _rotation_vector(matrix):
    vector, _jacobian = cv2.Rodrigues(np.asarray(matrix, dtype=float))
    return np.asarray(vector, dtype=float).reshape(3)


def refine_handeye_fixed_marker(
        samples, initial_transform, *, translation_sigma_m=0.001,
        rotation_sigma_deg=1.0, max_iterations=25):
    """Refine an OpenCV hand-eye seed using the fixed-marker constraint.

    For each sample, ``base -> tool -> camera -> marker`` must be the same
    unknown ``base -> marker`` transform.  OpenCV's closed-form hand-eye
    methods are useful deterministic seeds, but with correlated wrist motion
    their translation solution can be weakly observable even when every
    individual PnP pose is accurate.  This small Gauss-Newton loop minimizes
    that physical fixed-marker residual directly.  It never receives the
    simulation mount truth and is applicable to the real setup as well.
    """
    if len(samples) < 3:
        raise ValueError('at least three samples are required for refinement')
    translation_sigma = float(translation_sigma_m)
    rotation_sigma = math.radians(float(rotation_sigma_deg))
    iterations = int(max_iterations)
    if (not math.isfinite(translation_sigma) or translation_sigma <= 0.0
            or not math.isfinite(rotation_sigma) or rotation_sigma <= 0.0
            or iterations < 1):
        raise ValueError('fixed-marker refinement parameters are invalid')

    robots = [_components_matrix(transform_components(sample.robot))
              for sample in samples]
    trackings = [_components_matrix(transform_components(sample.tracking))
                 for sample in samples]
    camera_mount = _components_matrix(initial_transform)
    implied_markers = [
        robot @ camera_mount @ tracking
        for robot, tracking in zip(robots, trackings)]
    marker = np.eye(4, dtype=float)
    marker[:3, 3] = np.mean(
        [implied[:3, 3] for implied in implied_markers], axis=0)
    # The OpenCV seed is already close.  The first implied orientation avoids
    # adding another quaternion-average convention to the numerical solver.
    marker[:3, :3] = implied_markers[0][:3, :3]

    def residual(parameters):
        current_mount = camera_mount @ _se3_increment(parameters[:6])
        current_marker = marker @ _se3_increment(parameters[6:])
        values = []
        for robot, tracking in zip(robots, trackings):
            implied = robot @ current_mount @ tracking
            values.extend(
                (implied[:3, 3] - current_marker[:3, 3]) /
                translation_sigma)
            values.extend(_rotation_vector(
                current_marker[:3, :3].T @ implied[:3, :3]) /
                rotation_sigma)
        return np.asarray(values, dtype=float)

    parameters = np.zeros(12, dtype=float)
    initial_cost = float(residual(parameters) @ residual(parameters))
    final_cost = initial_cost
    completed_iterations = 0
    for iteration in range(iterations):
        current = residual(parameters)
        current_cost = float(current @ current)
        jacobian = np.empty((len(current), len(parameters)), dtype=float)
        for index in range(len(parameters)):
            epsilon = 1.0e-6 if index % 6 < 3 else 1.0e-5
            plus, minus = parameters.copy(), parameters.copy()
            plus[index] += epsilon
            minus[index] -= epsilon
            jacobian[:, index] = (
                residual(plus) - residual(minus)) / (2.0 * epsilon)
        step, _residuals, _rank, _singular = np.linalg.lstsq(
            jacobian, -current, rcond=None)
        accepted = False
        for scale in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125):
            trial = parameters + scale * step
            trial_values = residual(trial)
            trial_cost = float(trial_values @ trial_values)
            if trial_cost < current_cost:
                parameters, final_cost = trial, trial_cost
                completed_iterations = iteration + 1
                accepted = True
                break
        if not accepted or float(np.linalg.norm(scale * step)) < 1.0e-8:
            break

    refined = camera_mount @ _se3_increment(parameters[:6])
    return _matrix_components(refined), {
        'initial_cost': initial_cost,
        'final_cost': final_cost,
        'iterations': completed_iterations,
    }


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
