"""Alicia-M-specific, marker-relative hand-eye sampling geometry.

No absolute Fairino poses are reused.  Candidates are generated around the
marker currently seen by C10 and are later filtered by Alicia-M collision IK,
its URDF Jacobian condition number and joint-limit margins.
"""

from dataclasses import dataclass
import math

from wvcsc_arm_task.observation import (
    quaternion_from_matrix,
    rotate_vector,
    tool_pose_from_camera_pose,
)


@dataclass(frozen=True)
class CalibrationCandidate:
    candidate_id: str
    camera_position: tuple
    camera_quaternion: tuple
    tool_position: tuple
    tool_quaternion: tuple


def _unit(vector):
    values = tuple(float(value) for value in vector)
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm <= 1.0e-9:
        raise ValueError('calibration view vector is invalid')
    return tuple(value / norm for value in values)


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _camera_look_at(marker, camera, roll_deg, preferred_x):
    optical_z = _unit(tuple(
        float(marker[index]) - float(camera[index]) for index in range(3)))
    projected_x = tuple(
        preferred_x[index]
        - sum(preferred_x[i] * optical_z[i] for i in range(3)) * optical_z[index]
        for index in range(3))
    try:
        optical_x = _unit(projected_x)
    except ValueError:
        optical_x = _unit(_cross((0.0, 0.0, 1.0), optical_z))
    optical_y = _unit(_cross(optical_z, optical_x))
    roll = math.radians(float(roll_deg))
    cosine, sine = math.cos(roll), math.sin(roll)
    rolled_x = tuple(
        cosine * optical_x[index] + sine * optical_y[index]
        for index in range(3))
    rolled_y = tuple(
        -sine * optical_x[index] + cosine * optical_y[index]
        for index in range(3))
    matrix = tuple(zip(rolled_x, rolled_y, optical_z))
    return quaternion_from_matrix(matrix)


def generate_alicia_candidates(
        marker_position, current_camera_position, current_camera_quaternion,
        tool_to_camera_translation, tool_to_camera_quaternion):
    """Generate the requested 21 marker-relative views for Alicia-M."""
    marker = tuple(float(value) for value in marker_position)
    current = tuple(float(value) for value in current_camera_position)
    relative = tuple(current[index] - marker[index] for index in range(3))
    radius = math.sqrt(sum(value * value for value in relative))
    if not 0.20 <= radius <= 1.20:
        raise ValueError(
            f'initial camera-marker distance is unsafe: {radius:.3f} m')
    azimuth = math.atan2(relative[1], relative[0])
    elevation = math.asin(max(-1.0, min(1.0, relative[2] / radius)))
    preferred_x = rotate_vector((1.0, 0.0, 0.0), current_camera_quaternion)

    specifications = [('seed', 0.0, 0.0, 0.0, 0.0)]
    specifications.extend(
        (f'roll_{value:+g}', 0.0, 0.0, 0.0, value)
        for value in (-14.0, -8.0, 8.0, 14.0))
    specifications.extend(
        (f'radial_{value:+.3f}', value, 0.0, 0.0, 0.0)
        for value in (-0.045, -0.030, 0.030, 0.045))
    specifications.extend(
        (f'horizontal_{value:+g}', 0.0, value, 0.0, 0.0)
        for value in (-10.0, -6.0, 6.0, 10.0))
    specifications.extend(
        (f'vertical_{value:+g}', 0.0, 0.0, value, 0.0)
        for value in (-10.0, -6.0, 6.0, 10.0))
    specifications.extend((
        ('combo_left_low', 0.0, -6.0, -6.0, -5.0),
        ('combo_left_high', 0.0, -6.0, 6.0, 5.0),
        ('combo_right_low', 0.0, 6.0, -6.0, 5.0),
        ('combo_right_high', 0.0, 6.0, 6.0, -5.0),
    ))

    candidates = []
    for name, radial, horizontal_deg, vertical_deg, roll_deg in specifications:
        if name == 'seed':
            # 第一项必须是真实起始姿态，而不是重新构造的“近似 look-at”
            # 姿态。这样 q/Ctrl+C 回退基准和采样覆盖统计都与操作员确认的
            # 初始安全姿态一致。
            camera = current
            camera_quaternion = tuple(
                float(value) for value in current_camera_quaternion)
        else:
            candidate_radius = radius + radial
            candidate_azimuth = azimuth + math.radians(horizontal_deg)
            candidate_elevation = elevation + math.radians(vertical_deg)
            if (candidate_radius <= 0.20
                    or abs(candidate_elevation) >= math.radians(80.0)):
                continue
            horizontal_radius = candidate_radius * math.cos(candidate_elevation)
            camera = (
                marker[0] + horizontal_radius * math.cos(candidate_azimuth),
                marker[1] + horizontal_radius * math.sin(candidate_azimuth),
                marker[2] + candidate_radius * math.sin(candidate_elevation),
            )
            camera_quaternion = _camera_look_at(
                marker, camera, roll_deg, preferred_x)
        tool_position, tool_quaternion = tool_pose_from_camera_pose(
            camera, camera_quaternion,
            tool_to_camera_translation, tool_to_camera_quaternion)
        candidates.append(CalibrationCandidate(
            candidate_id=name,
            camera_position=camera,
            camera_quaternion=camera_quaternion,
            tool_position=tool_position,
            tool_quaternion=tool_quaternion,
        ))
    return candidates
