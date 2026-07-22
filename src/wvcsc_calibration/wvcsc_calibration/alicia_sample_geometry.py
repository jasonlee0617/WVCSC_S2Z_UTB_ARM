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


def generate_initial_anchor_candidates(
        marker_position, tool_to_camera_translation, tool_to_camera_quaternion,
        height_candidates, radial_backoff_candidates,
        tangential_offset_candidates, aim_yaw_deg=0.0, aim_pitch_deg=0.0):
    """Generate marker-prior views before the marker is visible in C10.

    The marker is fixed on the vehicle relative to ``alicia_base_link``.  The
    first view therefore needs no camera-to-marker TF, but it must still point
    C10 at the configured marker centre and pass the normal MoveIt safety gate
    in the caller.
    """
    marker = tuple(float(value) for value in marker_position)
    if len(marker) != 3 or not all(math.isfinite(value) for value in marker):
        raise ValueError('marker_position must contain three finite values')
    horizontal_distance = math.hypot(marker[0], marker[1])
    if horizontal_distance < 0.05:
        raise ValueError('marker_position must be offset from the arm base')

    radial = (marker[0] / horizontal_distance,
              marker[1] / horizontal_distance, 0.0)
    tangential = (-radial[1], radial[0], 0.0)
    candidates = []
    for height in height_candidates:
        height = float(height)
        if not math.isfinite(height) or height <= 0.0:
            raise ValueError('anchor heights must be finite and positive')
        for backoff in radial_backoff_candidates:
            backoff = float(backoff)
            if not math.isfinite(backoff) or backoff < 0.0:
                raise ValueError('anchor radial backoffs must be finite and non-negative')
            for offset in tangential_offset_candidates:
                offset = float(offset)
                if not math.isfinite(offset):
                    raise ValueError('anchor tangential offsets must be finite')
                camera = (
                    marker[0] - radial[0] * backoff + tangential[0] * offset,
                    marker[1] - radial[1] * backoff + tangential[1] * offset,
                    marker[2] + height,
                )
                camera_quaternion = _camera_look_at(
                    marker, camera, 0.0, (1.0, 0.0, 0.0))
                camera = _camera_position_for_aim(
                    marker, camera, camera_quaternion,
                    aim_yaw_deg, aim_pitch_deg)
                tool_position, tool_quaternion = tool_pose_from_camera_pose(
                    camera, camera_quaternion,
                    tool_to_camera_translation, tool_to_camera_quaternion)
                candidates.append(CalibrationCandidate(
                    candidate_id=(
                        f'initial_h{height:.3f}_r{backoff:.3f}_t{offset:+.3f}'),
                    camera_position=camera,
                    camera_quaternion=camera_quaternion,
                    tool_position=tool_position,
                    tool_quaternion=tool_quaternion,
                ))
    return tuple(candidates)


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


def _quaternion_multiply(left, right):
    """Compose parent-to-local rotations stored as ``(x, y, z, w)``."""
    lx, ly, lz, lw = (float(value) for value in left)
    rx, ry, rz, rw = (float(value) for value in right)
    return _unit((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def _local_tilt(quaternion, yaw_deg, pitch_deg):
    """Offset the optical axis while retaining the marker inside C10's FOV."""
    yaw = math.radians(float(yaw_deg)) * 0.5
    pitch = math.radians(float(pitch_deg)) * 0.5
    yaw_quaternion = (0.0, math.sin(yaw), 0.0, math.cos(yaw))
    pitch_quaternion = (math.sin(pitch), 0.0, 0.0, math.cos(pitch))
    return _quaternion_multiply(
        quaternion, _quaternion_multiply(yaw_quaternion, pitch_quaternion))


def _camera_position_for_aim(
        marker, camera, camera_quaternion, yaw_deg, pitch_deg):
    """Centre the target by translation while preserving wrist orientation."""
    yaw = math.radians(float(yaw_deg))
    pitch = math.radians(float(pitch_deg))
    if abs(math.cos(pitch)) <= 1.0e-6:
        raise ValueError('camera aim pitch is invalid')
    target_ray = _unit((
        -math.tan(yaw) / math.cos(pitch),
        math.tan(pitch),
        1.0,
    ))
    world_ray = rotate_vector(target_ray, camera_quaternion)
    distance = math.sqrt(sum(
        (float(marker[index]) - float(camera[index])) ** 2
        for index in range(3)))
    return tuple(
        float(marker[index]) - distance * world_ray[index]
        for index in range(3))


def generate_alicia_candidates(
        marker_position, current_camera_position, current_camera_quaternion,
        tool_to_camera_translation, tool_to_camera_quaternion,
        include_fine=False, aim_yaw_deg=0.0, aim_pitch_deg=0.0):
    """Generate baseline 21, or 49 excitation-expanded, marker-relative views.

    ``include_fine`` is intentionally opt-in.  The default set mirrors the
    documented 21 broad calibration views.  The collector only enables the
    expansion when the strict Alicia-M safety gate leaves too few candidates.
    The expansion first adds 16 wider, two-axis and off-axis views.  They add
    the rotational excitation that an eye-in-hand solve needs; the final
    twelve small perturbations are then only a reachability fallback.
    """
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
    if include_fine:
        # A horizontal desktop marker makes radial motion and camera roll alone
        # poorly conditioned for AX=XB.  Alicia-M cannot safely use every
        # vertical view, so first try reachable yaw+roll orbits around the
        # marker before consuming small near-duplicate views.
        excitation_specs = (
            ('wide_roll_-24', 0.0, 0.0, 0.0, -24.0),
            ('wide_roll_+24', 0.0, 0.0, 0.0, 24.0),
            ('wide_orbit_left', 0.0, -18.0, 0.0, -12.0),
            ('wide_orbit_right', 0.0, 18.0, 0.0, 12.0),
            ('wide_orbit_left_outer', 0.0, -24.0, 0.0, -16.0),
            ('wide_orbit_right_outer', 0.0, 24.0, 0.0, 16.0),
            ('wide_orbit_left_high', 0.0, -14.0, 4.0, -10.0),
            ('wide_orbit_right_high', 0.0, 14.0, 4.0, 10.0),
            ('wide_tilt_left', 0.0, 0.0, 0.0, 0.0, 12.0, 0.0),
            ('wide_tilt_right', 0.0, 0.0, 0.0, 0.0, -12.0, 0.0),
            ('wide_tilt_up', 0.0, 0.0, 0.0, 0.0, 0.0, 10.0),
            ('wide_tilt_down', 0.0, 0.0, 0.0, 0.0, 0.0, -10.0),
            ('wide_tilt_left_up', 0.0, 0.0, 0.0, -8.0, 10.0, 8.0),
            ('wide_tilt_right_up', 0.0, 0.0, 0.0, 8.0, -10.0, 8.0),
            ('wide_tilt_left_down', 0.0, 0.0, 0.0, 8.0, 10.0, -8.0),
            ('wide_tilt_right_down', 0.0, 0.0, 0.0, -8.0, -10.0, -8.0),
        )
        # The collector accepts candidates in this order.  Put the strong
        # excitation before radial fine-tuning so a session cannot fill with
        # near-parallel motions before reaching useful rotations.
        specifications[5:5] = excitation_specs
        specifications.extend(
            (f'fine_roll_{value:+g}', 0.0, 0.0, 0.0, value)
            for value in (-4.0, 4.0))
        specifications.extend(
            (f'fine_radial_{value:+.3f}', value, 0.0, 0.0, 0.0)
            for value in (-0.015, 0.015))
        specifications.extend(
            (f'fine_horizontal_{value:+g}', 0.0, value, 0.0, 0.0)
            for value in (-3.0, 3.0))
        specifications.extend(
            (f'fine_vertical_{value:+g}', 0.0, 0.0, value, 0.0)
            for value in (-3.0, 3.0))
        specifications.extend((
            ('fine_combo_left_low', 0.0, -3.0, -3.0, -3.0),
            ('fine_combo_left_high', 0.0, -3.0, 3.0, 3.0),
            ('fine_combo_right_low', 0.0, 3.0, -3.0, 3.0),
            ('fine_combo_right_high', 0.0, 3.0, 3.0, -3.0),
        ))

    candidates = []
    for specification in specifications:
        name, radial, horizontal_deg, vertical_deg, roll_deg, *tilt = specification
        yaw_deg, pitch_deg = (tilt + [0.0, 0.0])[:2]
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
            camera = _camera_position_for_aim(
                marker, camera, camera_quaternion,
                aim_yaw_deg, aim_pitch_deg)
            camera_quaternion = _local_tilt(
                camera_quaternion, yaw_deg, pitch_deg)
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
