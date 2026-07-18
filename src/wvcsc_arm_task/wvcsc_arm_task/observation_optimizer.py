"""Rank collision-free camera observation poses before visual servo starts."""

from dataclasses import dataclass
import math

import numpy as np
from urdf_parser_py.urdf import URDF

from .observation_pose import camera_look_at_pose, tool_pose_from_camera_pose


@dataclass
class ObservationCandidate:
    candidate_id: str
    distance_m: float
    camera_height_m: float
    azimuth_deg: float
    camera_position: tuple
    camera_quat: tuple
    tool_position: tuple
    tool_quat: tuple
    visible: bool
    visible_margin_px: float
    ik_joints: tuple = None
    condition_number: float = math.inf
    min_joint_margin_rad: float = 0.0
    joint_motion_norm: float = math.inf
    rejection_reason: str = ''


def _values(start, stop, step):
    count = int(round((float(stop) - float(start)) / float(step)))
    return tuple(float(start) + index * float(step) for index in range(count + 1))


def _rotation_from_rpy(rpy):
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


def _axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([
        [cosine + x * x * (1.0 - cosine), x * y * (1.0 - cosine) - z * sine,
         x * z * (1.0 - cosine) + y * sine],
        [y * x * (1.0 - cosine) + z * sine, cosine + y * y * (1.0 - cosine),
         y * z * (1.0 - cosine) - x * sine],
        [z * x * (1.0 - cosine) - y * sine, z * y * (1.0 - cosine) + x * sine,
         cosine + z * z * (1.0 - cosine)],
    ], dtype=float)


def _transform(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def _matrix_from_quaternion(quat_xyzw):
    x, y, z, w = (float(value) for value in quat_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


class ObservationOptimizer:
    """Use the actual URDF chain to keep observation poses servo-ready."""

    def __init__(self, robot_description, base_link, tip_link, joint_names, config):
        if not str(robot_description).strip():
            raise ValueError('robot_description is required for observation ranking')
        self._robot = URDF.from_xml_string(str(robot_description))
        self._base_link = str(base_link)
        self._tip_link = str(tip_link)
        self._joint_names = tuple(joint_names)
        self._config = dict(config)
        by_child = {joint.child: joint for joint in self._robot.joints}
        chain = []
        link = self._tip_link
        while link != self._base_link:
            joint = by_child.get(link)
            if joint is None:
                raise ValueError(
                    f'URDF has no chain from {self._base_link} to {self._tip_link}')
            chain.append(joint)
            link = joint.parent
        self._chain = tuple(reversed(chain))
        self._limits = {
            joint.name: (float(joint.limit.lower), float(joint.limit.upper))
            for joint in self._chain
            if joint.name in self._joint_names and joint.limit is not None
        }
        if set(self._limits) != set(self._joint_names):
            raise ValueError('URDF does not provide limits for every arm joint')

    def generate(self, tree_in_base, camera_mount, camera):
        tree_x, tree_y, _tree_z = (float(value) for value in tree_in_base)
        horizontal_distance = math.hypot(tree_x, tree_y)
        aim_height = (
            float(self._config['fruit_zone_height_min_m']) +
            float(self._config['fruit_zone_height_max_m'])) / 2.0
        candidates = []
        for distance in _values(
                self._config['distance_min_m'], self._config['distance_max_m'],
                self._config['distance_step_m']):
            if horizontal_distance <= distance + 0.05:
                continue
            for height in _values(
                    self._config['camera_height_min_m'],
                    self._config['camera_height_max_m'],
                    self._config['camera_height_step_m']):
                for azimuth in self._config['azimuth_offsets_deg']:
                    camera_position, camera_quat = camera_look_at_pose(
                        tree_in_base, aim_height, height, distance, azimuth)
                    visible, margin = self._envelope_visible(
                        tree_in_base, camera_position, camera_quat, camera)
                    tool_position, tool_quat = tool_pose_from_camera_pose(
                        camera_position, camera_quat, camera_mount[0], camera_mount[1])
                    candidates.append(ObservationCandidate(
                        candidate_id=(
                            f'd{distance:.2f}_h{height:.2f}_a{float(azimuth):+.0f}'),
                        distance_m=distance,
                        camera_height_m=height,
                        azimuth_deg=float(azimuth),
                        camera_position=camera_position,
                        camera_quat=camera_quat,
                        tool_position=tool_position,
                        tool_quat=tool_quat,
                        visible=visible,
                        visible_margin_px=margin,
                        rejection_reason='' if visible else 'fruit_zone_outside_camera',
                    ))
        return candidates

    def evaluate_ik(self, candidate, joints, current_joints):
        values = self._joint_vector(joints)
        condition = self.condition_number(values)
        margin = self.minimum_joint_margin(values)
        motion = float(np.linalg.norm(
            np.asarray(values, dtype=float) - np.asarray(current_joints, dtype=float)))
        candidate.ik_joints = values
        candidate.condition_number = condition
        candidate.min_joint_margin_rad = margin
        candidate.joint_motion_norm = motion
        if condition >= float(self._config['max_condition_number']):
            candidate.rejection_reason = 'near_singularity'
        elif margin < float(self._config['min_joint_margin_rad']):
            candidate.rejection_reason = 'joint_limit_margin'
        else:
            candidate.rejection_reason = ''
        return candidate

    def rank(self, candidates):
        preferred_margin = float(self._config.get(
            'preferred_joint_margin_rad',
            self._config['min_joint_margin_rad']))
        return sorted(
            (candidate for candidate in candidates
             if candidate.visible and not candidate.rejection_reason),
            key=lambda candidate: (
                candidate.min_joint_margin_rad < preferred_margin,
                candidate.condition_number,
                -candidate.min_joint_margin_rad,
                candidate.joint_motion_norm,
                candidate.candidate_id,
            ))

    def condition_number(self, joints):
        jacobian = self._jacobian(self._joint_vector(joints))
        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        smallest = float(singular_values[-1])
        return math.inf if smallest <= 1e-9 else float(singular_values[0] / smallest)

    def minimum_joint_margin(self, joints):
        values = self._joint_vector(joints)
        return min(
            min(value - self._limits[name][0], self._limits[name][1] - value)
            for name, value in zip(self._joint_names, values))

    def _joint_vector(self, joints):
        if isinstance(joints, dict):
            values = tuple(float(joints[name]) for name in self._joint_names)
        else:
            values = tuple(float(value) for value in joints)
        if len(values) != len(self._joint_names) or not all(
                math.isfinite(value) for value in values):
            raise ValueError('arm joint state is incomplete')
        return values

    def _jacobian(self, values):
        positions = dict(zip(self._joint_names, values))
        transform = np.eye(4)
        axes, origins = [], []
        for joint in self._chain:
            origin = joint.origin
            rotation = _rotation_from_rpy(origin.rpy if origin and origin.rpy else (0.0, 0.0, 0.0))
            translation = origin.xyz if origin and origin.xyz else (0.0, 0.0, 0.0)
            transform = transform @ _transform(rotation, translation)
            if joint.name in positions:
                local_axis = np.asarray(joint.axis if joint.axis else (0.0, 0.0, 1.0))
                axes.append(transform[:3, :3] @ local_axis)
                origins.append(transform[:3, 3].copy())
                transform = transform @ _transform(
                    _axis_rotation(local_axis, positions[joint.name]), (0.0, 0.0, 0.0))
        end = transform[:3, 3]
        linear = [np.cross(axis, end - origin) for axis, origin in zip(axes, origins)]
        return np.vstack((np.column_stack(linear), np.column_stack(axes)))

    def _envelope_visible(self, tree, camera_position, camera_quat, camera):
        fx, fy, cx, cy, width, height = camera
        rotation = _matrix_from_quaternion(camera_quat)
        camera_position = np.asarray(camera_position, dtype=float)
        border_x = float(width) * float(self._config['image_margin_ratio'])
        border_y = float(height) * float(self._config['image_margin_ratio'])
        margins = []
        for height_m in (
                float(self._config['fruit_zone_height_min_m']),
                float(self._config['fruit_zone_height_max_m'])):
            for angle in np.linspace(0.0, 2.0 * math.pi, 8, endpoint=False):
                point = np.array((
                    float(tree[0]) + float(self._config['fruit_zone_radius_m']) * math.cos(angle),
                    float(tree[1]) + float(self._config['fruit_zone_radius_m']) * math.sin(angle),
                    float(tree[2]) + height_m,
                ))
                optical = rotation.T @ (point - camera_position)
                if optical[2] <= 1e-6:
                    return False, -math.inf
                u = fx * optical[0] / optical[2] + cx
                v = fy * optical[1] / optical[2] + cy
                margins.append(min(u - border_x, width - border_x - u,
                                   v - border_y, height - border_y - v))
        margin = min(margins)
        return bool(margin >= 0.0), float(margin)
