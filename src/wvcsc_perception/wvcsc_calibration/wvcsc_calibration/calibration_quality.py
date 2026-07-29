"""Pure quality gates for Alicia-M automatic hand-eye calibration."""

from dataclasses import dataclass
import math
import statistics


@dataclass(frozen=True)
class MarkerObservation:
    center_px: tuple
    margin_px: float
    side_px: float
    translation: tuple
    rotation_vector: tuple
    received_monotonic: float


def _norm(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


def quaternion_angle_deg(left, right):
    left_norm, right_norm = _norm(left), _norm(right)
    if left_norm <= 1.0e-12 or right_norm <= 1.0e-12:
        raise ValueError('quaternion is invalid')
    dot = abs(sum(
        float(a) * float(b) for a, b in zip(left, right)) /
        (left_norm * right_norm))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def transform_error(actual, expected):
    """Return translation metres and shortest rotation degrees between poses.

    Both inputs are ``((x, y, z), (qx, qy, qz, qw))`` transforms with the
    same parent/child convention.  The quaternion helper intentionally treats
    ``q`` and ``-q`` as the same rotation, which is required for deterministic
    simulation ground-truth checks.
    """
    translation = _norm(tuple(
        float(actual[0][index]) - float(expected[0][index])
        for index in range(3)))
    rotation = quaternion_angle_deg(actual[1], expected[1])
    return translation, rotation


def stable_marker_window(
        observations, *, required_frames, min_distance_m, max_distance_m,
        minimum_margin_px, maximum_center_std_px,
        maximum_depth_std_m, maximum_angle_std_deg,
        minimum_marker_side_px=0.0):
    """Validate the latest consecutive ArUco observations."""
    required = int(required_frames)
    window = list(observations)[-required:]
    if required <= 0 or len(window) < required:
        return False, 'insufficient stable marker frames'
    if any(obs.margin_px < float(minimum_margin_px) for obs in window):
        return False, 'marker is too close to the image edge'
    if any(obs.side_px < float(minimum_marker_side_px) for obs in window):
        return False, 'marker is too small in the image'
    distances = [_norm(obs.translation) for obs in window]
    if min(distances) < float(min_distance_m) or max(distances) > float(max_distance_m):
        return False, (
            'marker distance is outside the calibration range: '
            f'observed=[{min(distances):.3f}, {max(distances):.3f}]m '
            f'required=[{float(min_distance_m):.3f}, '
            f'{float(max_distance_m):.3f}]m')
    center_u = [obs.center_px[0] for obs in window]
    center_v = [obs.center_px[1] for obs in window]
    center_std = max(statistics.pstdev(center_u), statistics.pstdev(center_v))
    if center_std > float(maximum_center_std_px):
        return False, f'marker centre is unstable: {center_std:.3f}px'
    depth_std = statistics.pstdev(obs.translation[2] for obs in window)
    if depth_std > float(maximum_depth_std_m):
        return False, f'marker depth is unstable: {depth_std:.6f}m'
    mean_rotation = tuple(
        statistics.fmean(obs.rotation_vector[index] for obs in window)
        for index in range(3))
    angle_std = math.degrees(math.sqrt(statistics.fmean(
        _norm(tuple(
            obs.rotation_vector[index] - mean_rotation[index]
            for index in range(3))) ** 2
        for obs in window)))
    if angle_std > float(maximum_angle_std_deg):
        return False, f'marker angle is unstable: {angle_std:.3f}deg'
    return True, 'marker observation is stable'


def pose_is_diverse(
        candidate_translation, candidate_quaternion, accepted_poses,
        minimum_translation_delta_m, minimum_rotation_delta_deg):
    if not accepted_poses:
        return True
    for translation, quaternion in accepted_poses:
        translation_delta = _norm(tuple(
            float(candidate_translation[index]) - float(translation[index])
            for index in range(3)))
        rotation_delta = quaternion_angle_deg(candidate_quaternion, quaternion)
        if (translation_delta < float(minimum_translation_delta_m)
                and rotation_delta < float(minimum_rotation_delta_deg)):
            return False
    return True


def sample_coverage(poses):
    """Return translation and orientation span for accepted robot poses."""
    poses = list(poses)
    if not poses:
        return 0.0, 0.0
    translations = [pose[0] for pose in poses]
    translation_span = max(
        _norm(tuple(a[index] - b[index] for index in range(3)))
        for a in translations for b in translations)
    quaternions = [pose[1] for pose in poses]
    rotation_span = max(
        quaternion_angle_deg(left, right)
        for left in quaternions for right in quaternions)
    return translation_span, rotation_span


def calibration_consensus(transforms):
    """Select the medoid result and return maximum inter-algorithm spreads."""
    transforms = list(transforms)
    if not transforms:
        raise ValueError('no calibration transforms were computed')

    def distance(left, right):
        translation = 100.0 * _norm(tuple(
            left[0][index] - right[0][index] for index in range(3)))
        rotation = quaternion_angle_deg(left[1], right[1])
        return translation + rotation

    selected = min(
        transforms,
        key=lambda item: sum(distance(item, other) for other in transforms))
    max_translation = max(
        _norm(tuple(left[0][index] - right[0][index] for index in range(3)))
        for left in transforms for right in transforms)
    max_rotation = max(
        quaternion_angle_deg(left[1], right[1])
        for left in transforms for right in transforms)
    return selected, max_translation, max_rotation


def transform_components(transform):
    return (
        (
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
        ),
        (
            float(transform.rotation.x),
            float(transform.rotation.y),
            float(transform.rotation.z),
            float(transform.rotation.w),
        ),
    )


def _normalize_quaternion(quaternion):
    norm = _norm(quaternion)
    if norm <= 1.0e-12:
        raise ValueError('quaternion is invalid')
    return tuple(float(value) / norm for value in quaternion)


def _quaternion_multiply(left, right):
    lx, ly, lz, lw = _normalize_quaternion(left)
    rx, ry, rz, rw = _normalize_quaternion(right)
    return _normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def _rotate(vector, quaternion):
    x, y, z, w = _normalize_quaternion(quaternion)
    vx, vy, vz = (float(value) for value in vector)
    return (
        (1 - 2 * (y * y + z * z)) * vx
        + 2 * (x * y - z * w) * vy
        + 2 * (x * z + y * w) * vz,
        2 * (x * y + z * w) * vx
        + (1 - 2 * (x * x + z * z)) * vy
        + 2 * (y * z - x * w) * vz,
        2 * (x * z - y * w) * vx
        + 2 * (y * z + x * w) * vy
        + (1 - 2 * (x * x + y * y)) * vz,
    )


def compose_transforms(left, right):
    """Compose ``A->B`` and ``B->C`` transform component tuples."""
    left_translation, left_quaternion = left
    right_translation, right_quaternion = right
    rotated = _rotate(right_translation, left_quaternion)
    return (
        tuple(left_translation[index] + rotated[index] for index in range(3)),
        _quaternion_multiply(left_quaternion, right_quaternion),
    )


def marker_pose_residuals(samples, handeye_transform):
    """Return per-sample residuals of the implied fixed ``base->marker``.

    The marker is stationary during an eye-in-hand session.  A robust median
    translation and quaternion medoid are used as the reference, so one bad
    ArUco/TF sample cannot move the reference enough to hide itself.
    """
    poses = []
    for sample in samples:
        robot = transform_components(sample.robot)
        tracking = transform_components(sample.tracking)
        poses.append(compose_transforms(
            compose_transforms(robot, handeye_transform), tracking))
    if not poses:
        raise ValueError('no samples are available for marker RMS')
    reference_translation = tuple(
        statistics.median(pose[0][index] for pose in poses)
        for index in range(3))
    reference_quaternion = min(
        (pose[1] for pose in poses),
        key=lambda candidate: sum(
            quaternion_angle_deg(candidate, other[1]) for other in poses))
    return tuple(
        (
            _norm(tuple(
                pose[0][index] - reference_translation[index]
                for index in range(3))),
            quaternion_angle_deg(pose[1], reference_quaternion),
        )
        for pose in poses)


def marker_pose_rms(samples, handeye_transform):
    """Evaluate constancy of ``base->marker`` implied by all samples."""
    residuals = marker_pose_residuals(samples, handeye_transform)
    translation_rms = math.sqrt(statistics.fmean(
        translation ** 2 for translation, _rotation in residuals))
    rotation_rms = math.sqrt(statistics.fmean(
        rotation ** 2 for _translation, rotation in residuals))
    return translation_rms, rotation_rms
