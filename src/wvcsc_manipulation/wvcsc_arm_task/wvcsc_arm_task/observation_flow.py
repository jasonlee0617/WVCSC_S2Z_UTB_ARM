"""ROS 观察流程与共享候选执行。

IK 几何/运动学公开边界由 :mod:`ik_observation` 导出，固定关节扫描策略位于
:mod:`joint_preset_observation`。本模块只保留二者共用的 ROS 快照、候选执行、
目标重心、观察位切换和调试发布逻辑，避免两种模式重复实现安全恢复。

位置单位为米，角度单位为弧度或函数名明确标注的度，四元数统一使用 (x, y, z, w)。
相机 optical frame 遵循 ROS 约定：+Z 向前、+X 向右、+Y 向下。
"""

import json
import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from std_msgs.msg import String
from tf2_ros import TransformException
from urdf_parser_py.urdf import URDF

from .target_flow import target_pixel_error


def transform_point(point, translation, quat_xyzw):
    """对点应用刚体变换，避免在纯几何层引入 tf2_geometry 依赖。
    
    此函数是纯数学层面的坐标变换，用于将目标物体从局部坐标系
    变换到机械臂基座坐标系。在底层，通过旋转矩阵与平移向量实现。
    """
    px, py, pz = (float(value) for value in point)
    tx, ty, tz = (float(value) for value in translation)
    quaternion = tuple(float(value) for value in quat_xyzw)
    values = (px, py, pz, tx, ty, tz, *quaternion)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('transform values must be finite')
    rotated = rotate_vector((px, py, pz), quaternion)
    return (
        tx + rotated[0],
        ty + rotated[1],
        tz + rotated[2],
    )


def camera_look_at_pose(
        tree_root, aim_height, camera_height, observation_distance,
        azimuth_offset_degrees=0.0):
    """生成 optical ``+Z`` 指向树冠中心的相机位姿。

    ``tree_root`` 是病树根部在机械臂 base 下的位置。``camera_height`` 是相机
    光心相对机械臂 base 的 Z 高度；相机位于树与机械臂之间，``observation_distance``
    为水平离树距离。树过近时直接拒绝，防止生成穿过树干或机械臂自身不可达的候选位姿。
    
    **工程意义**：机械臂末端相机不是末端工具点（tool0），而是挂载的深度相机。此函数
    计算的是相机坐标系在机械臂基座下的绝对坐标和旋转，随后需要经过工具外参逆变换，
    才能得到真正的 `tool0` 期望位置。
    """
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

    # 计算方位角，并叠加扇扫偏置
    bearing = math.atan2(ty, tx) + math.radians(azimuth_offset_degrees)
    forward_x = math.cos(bearing)
    forward_y = math.sin(bearing)
    
    # 相机位于树与机械臂连线之间，保持固定作业距离
    camera = (
        tx - observation_distance * forward_x,
        ty - observation_distance * forward_y,
        camera_height,
    )
    target_delta = (
        tx - camera[0],
        ty - camera[1],
        tz + aim_height - camera[2],
    )
    target_distance = math.sqrt(sum(value * value for value in target_delta))
    
    # 构造相机的 Optical 姿态（+Z 指向目标）
    optical_z = tuple(value / target_distance for value in target_delta)
    optical_x = (forward_y, -forward_x, 0.0)
    optical_y = (
        optical_z[1] * optical_x[2] - optical_z[2] * optical_x[1],
        optical_z[2] * optical_x[0] - optical_z[0] * optical_x[2],
        optical_z[0] * optical_x[1] - optical_z[1] * optical_x[0],
    )
    # 旋转矩阵的三列分别是 optical X/Y/Z 轴在机械臂 base 坐标系中的表达。
    matrix = (
        (optical_x[0], optical_y[0], optical_z[0]),
        (optical_x[1], optical_y[1], optical_z[1]),
        (optical_x[2], optical_y[2], optical_z[2]),
    )
    return camera, quaternion_from_matrix(matrix)


def camera_pose_from_bearing(
        tree_root, camera_reach, camera_height, pitch_degrees,
        yaw_offset_degrees=0.0):
    """Create an IK observation pose without classifying tree/leaf height.

    The camera is placed on a short radial grid around ``alicia_base_link``.
    Its optical +Z points along the tree bearing with a distance-adaptive
    downward pitch.  Unlike the retired fruit-zone envelope this contains no
    target-height or fixed camera-to-tree acceptance gate.
    """
    tree_x, tree_y, _tree_z = (float(value) for value in tree_root)
    camera_reach = float(camera_reach)
    camera_height = float(camera_height)
    pitch = math.radians(float(pitch_degrees))
    yaw = math.atan2(tree_y, tree_x) + math.radians(float(yaw_offset_degrees))
    values = (tree_x, tree_y, camera_reach, camera_height, pitch, yaw)
    if (not all(math.isfinite(value) for value in values) or
            math.hypot(tree_x, tree_y) <= 0.0 or camera_reach <= 0.0):
        raise ValueError('bearing observation inputs are invalid')
    camera = (
        camera_reach * math.cos(yaw),
        camera_reach * math.sin(yaw),
        camera_height,
    )
    optical_z = (
        math.cos(pitch) * math.cos(yaw),
        math.cos(pitch) * math.sin(yaw),
        math.sin(pitch),
    )
    optical_x = (math.sin(yaw), -math.cos(yaw), 0.0)
    optical_y = (
        optical_z[1] * optical_x[2] - optical_z[2] * optical_x[1],
        optical_z[2] * optical_x[0] - optical_z[0] * optical_x[2],
        optical_z[0] * optical_x[1] - optical_z[1] * optical_x[0],
    )
    matrix = (
        (optical_x[0], optical_y[0], optical_z[0]),
        (optical_x[1], optical_y[1], optical_z[1]),
        (optical_x[2], optical_y[2], optical_z[2]),
    )
    return camera, quaternion_from_matrix(matrix)


def camera_orientation_for_pixel(
        camera_position, camera_quat_xyzw, target_point, camera_model,
        desired_u, desired_v):
    """Rotate the camera so ``target_point`` projects at the requested pixel."""
    fx, fy, cx, cy, width, height = camera_model
    values = (
        *camera_position, *target_point, fx, fy, cx, cy,
        width, height, desired_u, desired_v)
    if (not all(math.isfinite(float(value)) for value in values) or
            fx <= 0.0 or fy <= 0.0 or width <= 0 or height <= 0):
        raise ValueError('camera targeting inputs are invalid')
    desired_ray = _unit_vector((
        (float(desired_u) - float(cx)) / float(fx),
        (float(desired_v) - float(cy)) / float(fy),
        1.0,
    ))
    target_ray = _unit_vector(tuple(
        float(target_point[index]) - float(camera_position[index])
        for index in range(3)))
    current_world_ray = rotate_vector(desired_ray, camera_quat_xyzw)
    correction = _quaternion_between_vectors(current_world_ray, target_ray)
    return quaternion_multiply(correction, camera_quat_xyzw)


def tool_pose_from_camera_pose(
        camera_position, camera_quat_xyzw,
        tool_to_camera_translation, tool_to_camera_quat_xyzw):
    """把 C10 optical 相机目标换算成其父级 ``tool0`` 目标。

    MoveIt 规划 SRDF 中的末端 ``tool0``，而观察方向由固定安装的 C10 optical frame
    定义。这里显式求取固定 ``tool0 -> camera`` 外参的逆变换，保证相机目标不被
    错当成末端目标。
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
    camera_to_tool_in_base = rotate_vector(
        camera_to_tool_translation, camera_quat_xyzw)
    tool_position = tuple(
        camera_position[index] + camera_to_tool_in_base[index]
        for index in range(3))
    return tool_position, quaternion_multiply(
        camera_quat_xyzw, camera_to_tool_quat)


def recenter_camera_pose(
        camera_position, camera_quat_xyzw, camera_model, center_u, center_v,
        desired_u, desired_v, max_angle_degrees,
        residual_error_px=0.0):
    """保持相机位置不变，将姿态转向喷洒像素，并可给 IBVS 留出残差。

    输入像素是分割掩膜内部的安全瞄准点，而非检测框中心。若完整重定向角超过
    ``max_angle_degrees`` 会失败；``residual_error_px`` 可让 MoveIt 只完成大范围
    重心，剩余小误差由连续视觉伺服消除，避免在单次 IK 中逼近奇异位形。
    """
    camera_position = tuple(float(value) for value in camera_position)
    fx, fy, cx, cy, width, height = camera_model
    values = (*camera_position, fx, fy, cx, cy, center_u, center_v,
              desired_u, desired_v, max_angle_degrees,
              residual_error_px)
    if (not all(math.isfinite(float(value)) for value in values) or
            fx <= 0.0 or fy <= 0.0 or width <= 0 or height <= 0 or
            max_angle_degrees <= 0.0 or residual_error_px < 0.0):
        raise ValueError('target recenter inputs are invalid')
    
    # The desired point is the calibrated nozzle-axis projection, supplied by
    # VisualServo.  Do not recreate it from an image-centre offset here.
    desired_u = float(desired_u)
    desired_v = float(desired_v)
    error_u = float(center_u) - desired_u
    error_v = float(center_v) - desired_v
    maximum_error = max(abs(error_u), abs(error_v))
    
    # 如果设置了残差，将一次大角度运动切割为预留残差的亚运动
    if residual_error_px > 0.0 and maximum_error > residual_error_px:
        residual_scale = float(residual_error_px) / maximum_error
        desired_u += residual_scale * error_u
        desired_v += residual_scale * error_v
        
    # 计算当前视线向量和期望视线向量
    current_ray = _unit_vector((
        (float(center_u) - float(cx)) / float(fx),
        (float(center_v) - float(cy)) / float(fy),
        1.0,
    ))
    desired_ray = _unit_vector((
        (desired_u - float(cx)) / float(fx),
        (desired_v - float(cy)) / float(fy),
        1.0,
    ))
    angle = math.degrees(math.acos(max(-1.0, min(1.0, sum(
        left * right for left, right in zip(current_ray, desired_ray))))))
    if angle > float(max_angle_degrees) + 1e-9:
        raise ValueError(
            f'target recenter angle exceeds limit: required_angle={angle:.1f}deg '
            f'limit={float(max_angle_degrees):.1f}deg')
    camera_quat_xyzw = normalize_quaternion(camera_quat_xyzw)
    # 计算在世界坐标系下旋转视线所需的四元数
    world_rotation = _quaternion_between_vectors(
        rotate_vector(desired_ray, camera_quat_xyzw),
        rotate_vector(current_ray, camera_quat_xyzw))
    return (
        camera_position,
        quaternion_multiply(world_rotation, camera_quat_xyzw),
        angle,
    )


def _unit_vector(vector):
    values = tuple(float(value) for value in vector)
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        raise ValueError('vector is invalid')
    return tuple(value / norm for value in values)


def _quaternion_between_vectors(source, target):
    source = _unit_vector(source)
    target = _unit_vector(target)
    dot = max(-1.0, min(1.0, sum(left * right for left, right in zip(source, target))))
    if dot < -1.0 + 1e-9:
        axis = _unit_vector((0.0, -source[2], source[1]))
        return axis[0], axis[1], axis[2], 0.0
    cross = (
        source[1] * target[2] - source[2] * target[1],
        source[2] * target[0] - source[0] * target[2],
        source[0] * target[1] - source[1] * target[0],
    )
    return normalize_quaternion((*cross, 1.0 + dot))


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


def rotation_matrix_from_quaternion(quat_xyzw):
    """将 XYZW 四元数转换为只读语义的 3x3 行优先旋转矩阵。"""
    x, y, z, w = normalize_quaternion(quat_xyzw)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def rotate_vector(vector, quat_xyzw):
    vx, vy, vz = (float(value) for value in vector)
    if not all(math.isfinite(value) for value in (vx, vy, vz)):
        raise ValueError('vector values must be finite')
    matrix = rotation_matrix_from_quaternion(quat_xyzw)
    return tuple(
        row[0] * vx + row[1] * vy + row[2] * vz
        for row in matrix)


def quaternion_from_matrix(matrix):
    """把正交 3x3 旋转矩阵转换为 XYZW 四元数。"""
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

@dataclass
class ObservationCandidate:
    """一个观察候选及其从几何检查到 IK 排序的完整评估结果。"""
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
    visible_fraction: float = 0.0
    projected_bbox: tuple = ()
    target_u_px: float = 0.0
    target_v_px: float = 0.0
    ik_joints: tuple = None
    condition_number: float = math.inf
    min_joint_margin_rad: float = 0.0
    joint_motion_norm: float = math.inf
    rejection_reason: str = ''
    selection_phase: str = 'unranked'
    observation_mode: str = 'ik'
    joint_positions: tuple = ()


def _build_candidate(
        candidate_id, distance_m, camera_height_m, azimuth_deg,
        camera_position, camera_quat, tool_position, tool_quat,
        visible, visible_margin_px, visible_fraction=0.0,
        projected_bbox=(), target_u_px=0.0, target_v_px=0.0,
        rejection_reason='', observation_mode='ik', joint_positions=()):
    return ObservationCandidate(
        candidate_id=candidate_id,
        distance_m=distance_m,
        camera_height_m=camera_height_m,
        azimuth_deg=float(azimuth_deg),
        camera_position=camera_position,
        camera_quat=camera_quat,
        tool_position=tool_position,
        tool_quat=tool_quat,
        visible=bool(visible),
        visible_margin_px=float(visible_margin_px),
        visible_fraction=float(visible_fraction),
        projected_bbox=tuple(float(value) for value in projected_bbox),
        target_u_px=float(target_u_px),
        target_v_px=float(target_v_px),
        rejection_reason=rejection_reason,
        observation_mode=str(observation_mode),
        joint_positions=tuple(float(value) for value in joint_positions),
    )


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


class ObservationOptimizer:
    """使用真实 URDF 链筛选适合后续视觉伺服的观察位姿。

    实例在节点生命周期内只读保存 URDF、关节顺序和阈值，不启动 ROS 线程。
    ``generate`` 只做视野几何；调用方取得碰撞 IK 后调用 ``evaluate_ik``；最后
    ``rank`` 返回可见、非奇异且关节余量足够的候选。
    """

    def __init__(self, robot_description, base_link, tip_link, joint_names, config):
        if not str(robot_description).strip():
            raise ValueError('robot_description is required for observation ranking')
        self._robot = URDF.from_xml_string(str(robot_description))
        self._base_link = str(base_link)
        self._tip_link = str(tip_link)
        self._joint_names = tuple(joint_names)
        self._config = dict(config)
        
        # 从 URDF 中提取从 base_link 到 tip_link 的关节链
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
        
        # 提取关节限位（用于后续计算关节余量）
        self._limits = {
            joint.name: (float(joint.limit.lower), float(joint.limit.upper))
            for joint in self._chain
            if joint.name in self._joint_names and joint.limit is not None
        }
        if set(self._limits) != set(self._joint_names):
            raise ValueError('URDF does not provide limits for every arm joint')

    def generate(self, tree_in_base, camera_mount, camera):
        """Enumerate the radial IK grid without height-zone filtering."""
        tree_x, tree_y, _tree_z = (float(value) for value in tree_in_base)
        horizontal_distance = math.hypot(tree_x, tree_y)
        near_range = float(self._config['tree_range_min_m'])
        far_range = float(self._config['tree_range_max_m'])
        progress = min(1.0, max(
            0.0, (horizontal_distance - near_range) / (far_range - near_range)))
        pitch = (
            float(self._config['pitch_near_deg']) + progress * (
                float(self._config['pitch_far_deg']) -
                float(self._config['pitch_near_deg'])))
        candidates = []
        for reach in _values(
                self._config['camera_reach_min_m'],
                self._config['camera_reach_max_m'],
                self._config['camera_reach_step_m']):
            for height in _values(
                    self._config['camera_height_min_m'],
                    self._config['camera_height_max_m'],
                    self._config['camera_height_step_m']):
                for azimuth in self._config['azimuth_offsets_deg']:
                    camera_position, camera_quat = camera_pose_from_bearing(
                        tree_in_base, reach, height, pitch, azimuth)
                    tool_position, tool_quat = tool_pose_from_camera_pose(
                        camera_position, camera_quat, camera_mount[0], camera_mount[1])
                    optical_depth = (
                        (horizontal_distance - reach) /
                        max(1e-6, math.cos(math.radians(pitch))))
                    candidates.append(_build_candidate(
                        candidate_id=(
                            f'r{reach:.2f}_h{height:.2f}_a{float(azimuth):+.0f}'),
                        distance_m=optical_depth,
                        camera_height_m=height,
                        azimuth_deg=azimuth,
                        camera_position=camera_position,
                        camera_quat=camera_quat,
                        tool_position=tool_position,
                        tool_quat=tool_quat,
                        visible=True,
                        visible_margin_px=math.inf,
                        visible_fraction=1.0,
                        projected_bbox=(0.0, 0.0, 0.0, 0.0),
                        target_u_px=float(camera[4]) / 2.0,
                        target_v_px=float(camera[5]) / 2.0,
                        rejection_reason='',
                    ))
        return candidates

    def evaluate_ik(self, candidate, joints, current_joints):
        """写入候选的运动学指标；不满足安全阈值时记录明确拒绝原因。"""
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
        """按安全优先级排序：余量达标、条件数低、余量大、移动距离短。"""
        preferred_margin = float(self._config.get(
            'preferred_joint_margin_rad',
            self._config['min_joint_margin_rad']))
        # 排序逻辑：主要考量关节限位余量、雅可比矩阵条件数、距当前关节角距离
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

    def order_for_tree_scan(self, candidates):
        """中心视角优先；tree 未确认后才进入左右扇形扫描。"""
        ranked = self.rank(candidates)
        centers = [candidate for candidate in ranked
                   if math.isclose(candidate.azimuth_deg, 0.0, abs_tol=1e-6)]
        center_ids = {id(candidate) for candidate in centers}
        sides = [candidate for candidate in ranked if id(candidate) not in center_ids]
        if not centers:
            return self._order_lateral_scan(sides, initial_phase='center_unavailable_fallback')

        center = centers[0]
        center.selection_phase = 'center_initial'
        same_view_sides = [
            candidate for candidate in sides
            if math.isclose(candidate.distance_m, center.distance_m, abs_tol=1e-6)
            and math.isclose(candidate.camera_height_m, center.camera_height_m,
                             abs_tol=1e-6)
        ]
        fan = self._order_lateral_scan(same_view_sides)
        selected = {id(candidate) for candidate in fan}
        recovery = [candidate for candidate in sides if id(candidate) not in selected]
        for candidate in recovery:
            candidate.selection_phase = 'recovery'
        return [center, *fan, *recovery]

    @staticmethod
    def _order_lateral_scan(candidates, initial_phase=None):
        """每个方向只先尝试一个同组候选，剩余侧向候选留作恢复。"""
        selected = []
        used = set()
        for direction, phase in ((-1.0, 'fan_left'), (1.0, 'fan_right')):
            candidate = next(
                (item for item in candidates
                 if id(item) not in used and item.azimuth_deg * direction > 0.0),
                None)
            if candidate is None:
                continue
            candidate.selection_phase = initial_phase if not selected and initial_phase else phase
            selected.append(candidate)
            used.add(id(candidate))
        for candidate in candidates:
            if id(candidate) not in used:
                candidate.selection_phase = 'recovery'
                selected.append(candidate)
        return selected

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
        """沿 URDF 链前向累乘，构造 base 表达的 6xN 几何雅可比矩阵。"""
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
                # 在雅可比计算中，需要累积当前关节的旋转
                transform = transform @ _transform(
                    _axis_rotation(local_axis, positions[joint.name]), (0.0, 0.0, 0.0))
        end = transform[:3, 3]
        linear = [np.cross(axis, end - origin) for axis, origin in zip(axes, origins)]
        return np.vstack((np.column_stack(linear), np.column_stack(axes)))

class ObservationFlowMixin:
    # --------- 扫描与确认树 ---------
    def _scan_for_tree(self, cancel_requested, *, fan_only=False):
        while not self._aborted(cancel_requested):
            candidate = self._active_observation_candidate()
            self._reset_tree_tracking()
            self._set_inference_mode('tree')
            if self._wait_for_tree(cancel_requested):
                self._publish_observation_debug('tree_confirmed', candidate)
                return True
            self._publish_observation_debug(
                'tree_not_confirmed', candidate,
                rejection_reason='tree_not_confirmed')
            if fan_only:
                if not self._move_to_next_fan_observation():
                    return False
            elif not self._move_to_next_observation():
                return False
        return False

    def _move_to_next_fan_observation(self):
        """Advance center -> left fan -> right fan, never into recovery views."""
        next_index = self._observation_candidate_index + 1
        if next_index >= len(self._observation_candidates):
            return False
        phase = getattr(
            self._observation_candidates[next_index], 'selection_phase', '')
        if not str(phase).startswith('fan_'):
            return False
        return self._move_to_next_observation()

    def _active_observation_candidate(self):
        index = self._observation_candidate_index
        if 0 <= index < len(self._observation_candidates):
            return self._observation_candidates[index]
        return None

    def _wait_for_tree(self, cancel_requested):
        deadline = time.monotonic() + float(
            self.get_parameter('scan_pose_detection_timeout_sec').value)
        required = int(self.get_parameter('confirmation_frames').value)
        while time.monotonic() < deadline:
            if self._aborted(cancel_requested):
                return False
            with self._vision_mutex:
                if self._tree_frames >= required:
                    return True
            time.sleep(0.02)
        return False

    # --------- 目标重心与姿态修正 ---------
    def _motion_preflight(
            self, target, current_joints, *, source, error_norm_px, stage):
        """Verify the stationary state before a safety-critical next step.

        Collision validity comes from the successful MoveIt observation or
        recenter plan that placed the arm at this state.  Re-solving IK for the
        same pose could choose a different branch and would not prove that the
        measured current state is collision-free.  The missing stationary
        checks are therefore the Jacobian condition number and joint-limit
        margin, evaluated from the actual joint feedback.
        """
        stage_message = 'Servo' if stage == 'SERVO' else stage
        index = self._observation_candidate_index
        if index < 0 or index >= len(self._observation_candidates):
            return False, (
                f'no active observation candidate for {stage_message} preflight')
        observation = self._observation_candidates[index]
        if observation.rejection_reason:
            return False, (
                'active observation is not motion-safe: '
                f'{observation.rejection_reason}')
        preflight = _build_candidate(
            candidate_id=f'{observation.candidate_id}_servo_preflight',
            distance_m=observation.distance_m,
            camera_height_m=observation.camera_height_m,
            azimuth_deg=observation.azimuth_deg,
            camera_position=observation.camera_position,
            camera_quat=observation.camera_quat,
            tool_position=observation.tool_position,
            tool_quat=observation.tool_quat,
            visible=True,
            visible_margin_px=math.inf,
            observation_mode=getattr(observation, 'observation_mode', 'ik'),
            joint_positions=getattr(observation, 'joint_positions', ()),
        )
        try:
            self._observation_optimizer.evaluate_ik(
                preflight, current_joints, current_joints)
        except (KeyError, TypeError, ValueError):
            return False, f'incomplete joint state for {stage_message} preflight'
        if preflight.rejection_reason:
            return False, (
                f'{stage_message} preflight rejected: '
                f'{preflight.rejection_reason}')
        self.get_logger().info(
            f'[ARM][{stage}_PREFLIGHT] target={target.target_id} '
            f'mode={getattr(observation, "observation_mode", "ik")} '
            f'source={source} error={error_norm_px:.1f}px '
            f'condition={preflight.condition_number:.2f} '
            f'joint_margin={preflight.min_joint_margin_rad:.2f} '
            'collision=active_moveit_plan')
        return True, ''

    def _servo_handoff_preflight(
            self, target, current_joints, *, source, error_norm_px):
        """Verify the stationary observation state before starting Servo."""
        return self._motion_preflight(
            target, current_joints, source=source,
            error_norm_px=error_norm_px, stage='SERVO')

    def _confirm_servo_handoff(
            self, target, current_joints, desired_u, desired_v,
            cancel_requested, *, pre_error_u, pre_error_v, planned_angle_deg,
            event, source, candidate=None, confirmed=None):
        """Reconfirm a target, enforce the bounded Servo entry, then preflight."""
        if confirmed is None:
            if not self._wait_for_target_confirmation(
                    target.target_id, cancel_requested, require_workspace=False):
                return False, 'target was not freshly reconfirmed before visual servo'
            confirmed = self._latest_target()
        post_error_u, post_error_v = target_pixel_error(
            confirmed.center_u, confirmed.center_v, desired_u, desired_v)
        post_error_norm = math.hypot(post_error_u, post_error_v)
        entry_px = self._recenter_config['servo_entry_px']
        if post_error_norm > entry_px:
            self._publish_observation_debug(
                'target_recenter_failed', candidate,
                target_id=target.target_id,
                pre_error_u_px=pre_error_u,
                pre_error_v_px=pre_error_v,
                post_error_u_px=post_error_u,
                post_error_v_px=post_error_v,
                planned_angle_deg=planned_angle_deg,
                rejection_reason='servo_entry_tolerance_not_reached')
            return False, (
                f'target residual {post_error_norm:.1f}px exceeds Servo entry '
                f'tolerance {entry_px:.1f}px')
        inputs = self._wait_for_observation_inputs()
        if inputs is None:
            return False, 'camera or joint state unavailable for Servo preflight'
        _camera, current_joints = inputs
        preflight_ok, preflight_message = self._servo_handoff_preflight(
            confirmed, current_joints, source=source,
            error_norm_px=post_error_norm)
        if not preflight_ok:
            self._publish_observation_debug(
                'target_recenter_failed', candidate,
                target_id=target.target_id,
                pre_error_u_px=pre_error_u,
                pre_error_v_px=pre_error_v,
                post_error_u_px=post_error_u,
                post_error_v_px=post_error_v,
                planned_angle_deg=planned_angle_deg,
                rejection_reason=preflight_message)
            return False, preflight_message
        self._publish_observation_debug(
            event, candidate,
            target_id=target.target_id,
            pre_error_u_px=pre_error_u,
            pre_error_v_px=pre_error_v,
            post_error_u_px=post_error_u,
            post_error_v_px=post_error_v,
            planned_angle_deg=planned_angle_deg)
        return True, 'target reconfirmed for visual servo'

    def _recenter_target(self, target, attempt, cancel_requested):
        """使用掩膜安全瞄准点执行有限角度重心，并重新确认同一逻辑目标。

        重心只旋转相机姿态、保持位置和喷洒距离；每步都重新做碰撞 IK、条件数和
        关节余量检查。目标丢失、关联歧义或安全筛选失败时返回可恢复失败，由上层
        切换观察候选，绝不就近改选另一颗病果。
        """
        inputs = self._wait_for_observation_inputs()
        if inputs is None:
            return False, 'camera or joint state unavailable for target recenter'
        camera, current_joints = inputs
        desired_aim = self._active_aim_pixel(camera[4], camera[5])
        if desired_aim is None:
            return False, 'calibrated nozzle aim is unavailable for target recenter'
        desired_u, desired_v = desired_aim
        pre_error_u, pre_error_v = target_pixel_error(
            target.center_u, target.center_v, desired_u, desired_v)
        
        trigger_px = self._recenter_config['trigger_px']
        pre_error_norm = math.hypot(pre_error_u, pre_error_v)
        if pre_error_norm <= trigger_px:
            return self._confirm_servo_handoff(
                target, current_joints, desired_u, desired_v, cancel_requested,
                pre_error_u=pre_error_u, pre_error_v=pre_error_v,
                planned_angle_deg=0.0,
                event='target_recenter_not_required',
                source='inside_recenter_trigger')
        # 目标偏离多少像素不再是重心准入条件。真正的物理边界由单步/累计
        # 旋转角、碰撞 IK、关节余量及 MoveIt 规划共同决定；固定 128 px
        # 门限曾错误拒绝约 19°、但仍在 20°安全重心范围内的病果。
        index = self._observation_candidate_index
        if index < 0 or index >= len(self._observation_candidates):
            return False, 'no active observation candidate for target recenter'
        if index in attempt.recentered_observation_indices:
            return False, 'target recenter already used at this observation'
        attempt.recentered_observation_indices.add(index)
        observation = self._observation_candidates[index]
        
        # 获取真实的相机位姿作为初始起点（不是规划的终点）
        camera_pose = self._current_camera_pose()
        if camera_pose is None:
            return False, 'actual camera pose unavailable for target recenter'
        maximum_total_angle = self._recenter_config.get(
            'max_total_angle_deg', math.inf)
        candidate, angle_deg, rejection_reason = self._move_recenter_step(
            observation, target, camera, current_joints, camera_pose=camera_pose,
            max_angle_deg=min(
                self._recenter_config['max_angle_deg'], maximum_total_angle))
        if candidate is None:
            return False, f'target recenter rejected: {rejection_reason}'
        if self._aborted(cancel_requested):
            return False, 'spray goal canceled'
        self._reset_target_confirmation(target.target_id)
        if not self._wait_for_target_confirmation(
                target.target_id, cancel_requested, require_workspace=False):
            self._publish_observation_debug(
                'target_recenter_failed', candidate,
                target_id=target.target_id,
                pre_error_u_px=pre_error_u,
                pre_error_v_px=pre_error_v,
                planned_angle_deg=angle_deg,
                rejection_reason='target_not_reconfirmed')
            return False, 'target was not reconfirmed after recenter'
        
        confirmed = self._latest_target()
        post_error_u, post_error_v = target_pixel_error(
            confirmed.center_u, confirmed.center_v, desired_u, desired_v)
        total_angle_deg = angle_deg
        iterations = 1
        self.get_logger().info(
            f'[ARM][RECENTER_STEP] target={target.target_id} iteration=1 '
            f'angle={angle_deg:.1f}deg total={total_angle_deg:.1f}deg '
            f'error={math.hypot(pre_error_u, pre_error_v):.1f}px'
            f'→{math.hypot(post_error_u, post_error_v):.1f}px')
        
        # 循环细化重心，直到误差小于 refine_goal_px 或达到最大迭代次数
        while (
                iterations < self._recenter_config['max_iterations'] and
                total_angle_deg < maximum_total_angle and
                math.hypot(post_error_u, post_error_v) >
                self._recenter_config['refine_goal_px']):
            inputs = self._wait_for_observation_inputs()
            if inputs is None:
                break
            camera, current_joints = inputs
            desired_aim = self._active_aim_pixel(camera[4], camera[5])
            if desired_aim is None:
                break
            desired_u, desired_v = desired_aim
            camera_pose = self._current_camera_pose()
            if camera_pose is None:
                self._publish_observation_debug(
                    'target_recenter_refine_skipped', candidate,
                    target_id=target.target_id,
                    pre_error_u_px=pre_error_u,
                    pre_error_v_px=pre_error_v,
                    post_error_u_px=post_error_u,
                    post_error_v_px=post_error_v,
                    planned_angle_deg=total_angle_deg,
                    rejection_reason='actual_camera_pose_unavailable')
                break
            refined, refine_angle, rejection_reason = self._move_recenter_step(
                candidate, confirmed, camera, current_joints, camera_pose=camera_pose,
                suffix=f'_refine{iterations}', max_angle_deg=min(
                    self._recenter_config['max_angle_deg'],
                    maximum_total_angle - total_angle_deg))
            if refined is None:
                self._publish_observation_debug(
                    'target_recenter_refine_skipped', candidate,
                    target_id=target.target_id,
                    pre_error_u_px=pre_error_u,
                    pre_error_v_px=pre_error_v,
                    post_error_u_px=post_error_u,
                    post_error_v_px=post_error_v,
                    planned_angle_deg=total_angle_deg,
                    rejection_reason=rejection_reason)
                self.get_logger().warn(
                    f'[ARM][RECENTER_STEP] target={target.target_id} '
                    f'iteration={iterations + 1} rejected '
                    f'error={math.hypot(post_error_u, post_error_v):.1f}px '
                    f'reason={rejection_reason}')
                break
            candidate = refined
            total_angle_deg += refine_angle
            iterations += 1
            self._reset_target_confirmation(target.target_id)
            if not self._wait_for_target_confirmation(
                    target.target_id, cancel_requested,
                    require_workspace=False):
                self._publish_observation_debug(
                    'target_recenter_failed', candidate,
                    target_id=target.target_id,
                    pre_error_u_px=pre_error_u,
                    pre_error_v_px=pre_error_v,
                    planned_angle_deg=total_angle_deg,
                    rejection_reason='target_not_reconfirmed_after_refine')
                return False, 'target was not reconfirmed after recenter refinement'
            confirmed = self._latest_target()
            previous_error_norm = math.hypot(post_error_u, post_error_v)
            post_error_u, post_error_v = target_pixel_error(
                confirmed.center_u, confirmed.center_v, desired_u, desired_v)
            self.get_logger().info(
                f'[ARM][RECENTER_STEP] target={target.target_id} '
                f'iteration={iterations} angle={refine_angle:.1f}deg '
                f'total={total_angle_deg:.1f}deg '
                f'error={previous_error_norm:.1f}px'
                f'→{math.hypot(post_error_u, post_error_v):.1f}px')
        # 粗对准只负责把一个新鲜、有效的锁定目标送入 Servo 可控窗口；最终
        # 4px/0.5s 稳定性仍由 AlignTarget 闭环统一判定。
        handoff_ok, handoff_message = self._confirm_servo_handoff(
            target, current_joints, desired_u, desired_v, cancel_requested,
            pre_error_u=pre_error_u, pre_error_v=pre_error_v,
            planned_angle_deg=total_angle_deg,
            event='target_recenter_confirmed', source='safe_moveit_recenter',
            candidate=candidate, confirmed=confirmed)
        if not handoff_ok:
            return False, handoff_message
        self.get_logger().info(
            f'[ARM][RECENTER] target={target.target_id} '
            f'iterations={iterations} angle={total_angle_deg:.1f}deg '
            f'error=({pre_error_u:.1f},{pre_error_v:.1f})px'
            f'→({post_error_u:.1f},{post_error_v:.1f})px '
            f'condition={candidate.condition_number:.2f} '
            f'joint_margin={candidate.min_joint_margin_rad:.2f}')
        return True, 'target reconfirmed after recenter'

    def _move_recenter_step(
            self, observation, target, camera, current_joints, *, camera_pose=None,
            suffix='', max_angle_deg=None, residual_candidates=None):
        """用真实 C10 起点生成并执行一次安全的重心姿态。

        ``observation`` 只提供候选的距离、高度、方位和诊断身份；若调用方传入
        ``camera_pose``，它来自 ``base -> camera_color_optical_frame`` 的最新 TF。
        这样多次重心会根据真实执行终点继续修正，而不是在旧的计划终点附近重复
        计算同一个旋转。没有显式位姿时保留候选中的几何值，方便纯单元测试。
        """
        if camera_pose is None:
            start_camera_position = observation.camera_position
            start_camera_quat = observation.camera_quat
        else:
            start_camera_position, start_camera_quat = camera_pose
        rejection_reason = 'no partial recenter candidate was feasible'
        desired_aim = self._active_aim_pixel(camera[4], camera[5])
        if desired_aim is None:
            return None, 0.0, 'calibrated nozzle aim is unavailable'
        if max_angle_deg is None:
            max_angle_deg = self._recenter_config['max_angle_deg']
        if max_angle_deg <= 0.0:
            return None, 0.0, 'recenter total angle limit reached'
        # 闭环模式尝试多个残差；实机单次对准显式传入 (0.0,)。
        residual_candidates = (
            self._recenter_config['residual_candidates_px']
            if residual_candidates is None else tuple(residual_candidates))
        if not residual_candidates:
            return None, 0.0, 'no recenter residual candidates configured'
        for residual_px in residual_candidates:
            try:
                camera_position, camera_quat, angle_deg = recenter_camera_pose(
                    start_camera_position, start_camera_quat,
                    camera, target.center_u, target.center_v,
                    *desired_aim,
                    max_angle_deg,
                    residual_error_px=residual_px)
                tool_position, tool_quat = tool_pose_from_camera_pose(
                    camera_position, camera_quat,
                    self._camera_mount[0], self._camera_mount[1])
            except (TypeError, ValueError) as error:
                rejection_reason = str(error)
                continue
            trial = _build_candidate(
                candidate_id=(
                    f'{observation.candidate_id}_target_{target.target_id}'
                    f'_r{residual_px:g}{suffix}'),
                distance_m=observation.distance_m,
                camera_height_m=observation.camera_height_m,
                azimuth_deg=observation.azimuth_deg,
                camera_position=camera_position,
                camera_quat=camera_quat,
                tool_position=tool_position,
                tool_quat=tool_quat,
                visible=True,
                visible_margin_px=math.inf,
            )
            if self._move_to_recentered_pose(trial, current_joints):
                return trial, angle_deg, ''
            rejection_reason = trial.rejection_reason
            if (rejection_reason.startswith('actual_') or
                    rejection_reason in {
                        'moveit_execution_failed',
                        'joint_state_unavailable_after_motion',
                    }):
                break
        return None, 0.0, rejection_reason

    def _execute_candidate_motion(
            self, candidate, *, current_joints=None,
            tolerance_position=None, tolerance_orientation=None,
            validate_target_ik=False, prefix_actual_rejection=False):
        """执行候选位姿的统一安全流程。

        普通观察候选在生成阶段已经完成碰撞 IK，重心候选则必须在执行前重新
        计算 IK，因此通过 ``validate_target_ik`` 保持两条原有路径的语义差异。
        两条路径共享计划轨迹、执行、最新关节状态和实际安全指标复核逻辑。
        """
        # 1. 如果通过 IK 校验执行重心步骤，则计算当前状态下的 IK
        if validate_target_ik:
            ik = self.arm.compute_ik(
                candidate.tool_position, candidate.tool_quat, current_joints)
            if ik is None:
                candidate.rejection_reason = 'collision_ik_failed'
                self._publish_observation_debug('candidate_rejected', candidate)
                return False
            try:
                self._observation_optimizer.evaluate_ik(
                    candidate, dict(zip(ik.name, ik.position)), current_joints)
            except (KeyError, TypeError, ValueError):
                candidate.rejection_reason = 'incomplete_ik_state'
            if candidate.rejection_reason:
                self._publish_observation_debug('candidate_rejected', candidate)
                return False

        # 2. 设置姿态公差并规划轨迹
        if tolerance_position is None or tolerance_orientation is None:
            tolerance_position = self._recenter_config['position_tolerance_m']
            tolerance_orientation = self._recenter_config[
                'orientation_tolerance_rad']
        trajectory = self.arm.plan_pose(
            candidate.tool_position, candidate.tool_quat, frame_id=self._base_frame,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation)
        planned = self.arm.trajectory_final_positions(
            trajectory, self.arm_joint_names) if trajectory is not None else None
        if planned is None:
            candidate.rejection_reason = 'moveit_plan_failed'
            self._publish_observation_debug('candidate_rejected', candidate)
            return False
        
        # 3. 用规划预期的终点关节值评估运动学指标
        self._observation_optimizer.evaluate_ik(candidate, planned, planned)
        if candidate.rejection_reason:
            self._publish_observation_debug('candidate_rejected', candidate)
            return False
        
        # 4. 执行轨迹
        with self._state_mutex:
            joint_state_sequence = self._joint_state_sequence
        if not self.arm.execute_trajectory(trajectory):
            candidate.rejection_reason = 'moveit_execution_failed'
            self._publish_observation_debug('candidate_rejected', candidate)
            return False
            
        # 5. 执行完成后的实际状态复查（闭环安全确认）
        actual = self._wait_for_joint_state(joint_state_sequence)
        if actual is None:
            candidate.rejection_reason = 'joint_state_unavailable_after_motion'
        else:
            self._observation_optimizer.evaluate_ik(candidate, actual, actual)
            if candidate.rejection_reason:
                if prefix_actual_rejection:
                    candidate.rejection_reason = (
                        f'actual_{candidate.rejection_reason}')
        if candidate.rejection_reason:
            self._publish_observation_debug('candidate_rejected', candidate)
            return False
        return True

    def _move_to_recentered_pose(self, candidate, current_joints):
        """Apply the normal collision IK, singularity and joint-margin gates."""
        return self._execute_candidate_motion(
            candidate,
            current_joints=current_joints,
            validate_target_ik=True,
            prefix_actual_rejection=True)

    @staticmethod
    def _hint_available(tree_hint):
        if tree_hint is None or not str(tree_hint.header.frame_id).strip():
            return False
        point = tree_hint.point
        return all(math.isfinite(value) for value in (point.x, point.y, point.z))

    # --------- 从 hint 走到观察位 ---------
    def _move_to_observation(self, tree_hint):
        self._observation_failure_reason = ''
        if not self._hint_available(tree_hint):
            self._observation_failure_reason = 'tree_hint_unavailable'
            self.get_logger().error('[ARM] tree_hint is required for observation')
            return False
        try:
            # 将 tree_hint 从目标坐标系（如 map）转换到机械臂基座坐标系 (alicia_base_link)
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, tree_hint.header.frame_id, rclpy.time.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            tree_in_base = transform_point(
                (tree_hint.point.x, tree_hint.point.y, tree_hint.point.z),
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w))
        except (TransformException, ValueError) as error:
            self._observation_failure_reason = f'observation_tf_failed: {error}'
            self.get_logger().error(f'[ARM] cannot build observation pose: {error}')
            return False
        self._tree_in_base = tree_in_base

        if self._observation_mode == 'joint_presets':
            if not self._select_joint_preset_side(tree_in_base):
                return False
        else:
            range_m = math.hypot(tree_in_base[0], tree_in_base[1])
            minimum = float(self._observation_config['tree_range_min_m'])
            maximum = float(self._observation_config['tree_range_max_m'])
            if not minimum <= range_m <= maximum:
                self._observation_failure_reason = (
                    'ik_tree_range_out_of_range '
                    f'(range={range_m:.3f}m, allowed={minimum:.3f}-{maximum:.3f}m)')
                self._publish_observation_debug(
                    'search_failed', rejection_reason=self._observation_failure_reason)
                return False

        try:
            # 粗对准仍需已加载的 tool0 -> camera 外参；预设扫描本身不使用它求 IK。
            camera_transform = self._tf_buffer.lookup_transform(
                'tool0', self._camera_frame, rclpy.time.Time())
        except TransformException as error:
            self._observation_failure_reason = f'observation_tf_failed: {error}'
            self.get_logger().error(f'[ARM] cannot load camera mount: {error}')
            return False
        camera_translation = camera_transform.transform.translation
        camera_rotation = camera_transform.transform.rotation
        self._camera_mount = (
            (camera_translation.x, camera_translation.y, camera_translation.z),
            (camera_rotation.x, camera_rotation.y,
             camera_rotation.z, camera_rotation.w),
        )
        if self._observation_mode == 'joint_presets':
            if not self._prepare_joint_preset_observation_candidates():
                return False
            return self._move_to_next_observation()
        if not self._prepare_observation_candidates():
            return False
        return self._move_to_next_observation()

    def _prepare_observation_candidates(self):
        """生成观察网格，结合碰撞 IK 和 URDF 指标保留少量安全候选。"""
        inputs = self._wait_for_observation_inputs()
        if inputs is None:
            self._observation_failure_reason = 'camera_or_joint_state_unavailable'
            self._publish_observation_debug(
                'search_failed', rejection_reason='camera_or_joint_state_unavailable')
            return False
        camera, current_joints = inputs
        started = time.monotonic()
        candidates = self._observation_optimizer.generate(
            self._tree_in_base, self._camera_mount, camera)
        visible_count = len(candidates)
        for candidate in candidates:
            if not candidate.visible:
                self._publish_observation_debug('candidate_rejected', candidate)
                continue
            if time.monotonic() - started >= float(
                    self.get_parameter('observation_search_timeout_sec').value):
                candidate.rejection_reason = 'ik_search_timeout'
                self._publish_observation_debug('candidate_rejected', candidate)
                continue
            ik = self.arm.compute_ik(
                candidate.tool_position, candidate.tool_quat, current_joints)
            if ik is None:
                candidate.rejection_reason = 'collision_ik_failed'
                self._publish_observation_debug('candidate_rejected', candidate)
                continue
            try:
                self._observation_optimizer.evaluate_ik(
                    candidate, dict(zip(ik.name, ik.position)), current_joints)
            except (KeyError, TypeError, ValueError):
                candidate.rejection_reason = 'incomplete_ik_state'
            self._publish_observation_debug(
                'candidate_ranked' if not candidate.rejection_reason
                else 'candidate_rejected', candidate)
        self._observation_candidates = self._observation_optimizer.order_for_tree_scan(
            candidates)[:int(self.get_parameter('observation_max_plans').value)]
        ik_count = sum(candidate.ik_joints is not None for candidate in candidates)
        servo_safe_count = sum(
            candidate.visible and not candidate.rejection_reason
            for candidate in candidates)
        self.get_logger().info(
            f'[ARM][OBSERVE] tree_in_base=({self._tree_in_base[0]:.2f},'
            f'{self._tree_in_base[1]:.2f},{self._tree_in_base[2]:.2f}) '
            f'camera={camera[4]}x{camera[5]} fx={camera[0]:.1f} fy={camera[1]:.1f} '
            f'generated={len(candidates)} view_usable={visible_count} '
            f'ik_valid={ik_count} servo_safe={servo_safe_count}')
        self._observation_candidate_index = -1
        if not self._observation_candidates:
            if not candidates:
                reason = 'no_observation_candidate'
            elif visible_count == 0:
                reason = 'no_observation_candidate'
            elif ik_count == 0:
                reason = 'no_collision_free_ik_candidate'
            else:
                reason = 'no_servo_safe_candidate'
            self._observation_failure_reason = reason
            self._publish_observation_debug(
                'search_failed', rejection_reason=reason)
            return False
        if self._observation_candidates[0].selection_phase == \
                'center_unavailable_fallback':
            candidate = self._observation_candidates[0]
            self.get_logger().warn(
                '[ARM][OBSERVE] no safe center observation candidate; '
                f'falling back to azimuth={candidate.azimuth_deg:+.0f} deg')
            self._publish_observation_debug(
                'center_view_unavailable', candidate,
                rejection_reason='no_center_servo_safe_candidate')
        return True

    def _move_to_next_observation(self, excluded_indices=None):
        excluded = set(excluded_indices or ())
        while self._observation_candidate_index + 1 < len(
                self._observation_candidates):
            self._observation_candidate_index += 1
            if self._observation_candidate_index in excluded:
                continue
            candidate = self._observation_candidates[self._observation_candidate_index]
            if self._aborted(lambda: False):
                return False
            if getattr(candidate, 'observation_mode', 'ik') == 'joint_presets':
                if not self.arm.move_joints(candidate.joint_positions):
                    candidate.rejection_reason = 'joint_preset_move_failed'
                    self.get_logger().warn(
                        '[ARM][OBSERVE] mode=joint_presets failed '
                        f'index={self._observation_candidate_index} '
                        f'id={candidate.candidate_id}; trying next preset')
                    self._publish_observation_debug('candidate_rejected', candidate)
                    continue
                camera_pose = self._current_camera_pose()
                if camera_pose is not None:
                    candidate.camera_position, candidate.camera_quat = camera_pose
                    candidate.camera_height_m = candidate.camera_position[2]
                working_range, _error = self._dynamic_nozzle_range()
                self._observation_distance = working_range
                self._observation_pose = None
                joints_deg = ','.join(
                    f'{math.degrees(value):.1f}'
                    for value in candidate.joint_positions)
                self.get_logger().info(
                    '[ARM][OBSERVE] mode=joint_presets selected '
                    f'index={self._observation_candidate_index} '
                    f'id={candidate.candidate_id} joints_deg=[{joints_deg}]')
                self._publish_observation_debug('candidate_selected', candidate)
                return True
            if self._execute_candidate_motion(
                    candidate,
                    tolerance_position=self._observation_config[
                        'position_tolerance_m'],
                    tolerance_orientation=self._observation_config[
                        'orientation_tolerance_rad']):
                self._observation_distance = candidate.distance_m
                self._observation_pose = (candidate.tool_position, candidate.tool_quat)
                self.get_logger().info(
                    f'[ARM][ALIGN] selected observation candidate '
                    f'index={self._observation_candidate_index} '
                    f'id={candidate.candidate_id} distance={candidate.distance_m} m '
                    f'phase={getattr(candidate, "selection_phase", "recovery")} '
                    f'camera_height_in_base={candidate.camera_height_m:.2f} m '
                    f'camera_z_in_base={candidate.camera_position[2]:.2f} m '
                    f'condition={candidate.condition_number:.2f} '
                    f'joint_margin={candidate.min_joint_margin_rad:.2f}')
                self._publish_observation_debug('candidate_selected', candidate)
                return True
            if candidate.rejection_reason == 'moveit_execution_failed':
                self.get_logger().warn(
                    f'[ARM][ALIGN] planning failed for observation candidate '
                    f'index={self._observation_candidate_index} '
                    f'id={candidate.candidate_id}')
        self._observation_failure_reason = 'all_observation_motion_candidates_failed'
        return False

    def _wait_for_state(self, *, require_camera=False, after_sequence=None):
        """等待共享状态快照，统一相机/关节输入的超时与轮询语义。"""
        deadline = time.monotonic() + float(
            self.get_parameter('observation_input_timeout_sec').value)
        while time.monotonic() < deadline:
            with self._state_mutex:
                camera = self._camera_model
                joints = self._joint_positions
                sequence = self._joint_state_sequence
            if (joints is not None and
                    (not require_camera or camera is not None) and
                    (after_sequence is None or sequence > after_sequence)):
                return (camera, joints) if require_camera else joints
            time.sleep(0.02)
        return None

    def _wait_for_observation_inputs(self):
        return self._wait_for_state(require_camera=True)

    def _current_camera_pose(self):
        """返回机械臂 base 下 C10 optical frame 的最新实际 TF 位姿。

        MoveIt 的 ``SUCCEEDED`` 只说明控制器接受并完成了轨迹容差内的执行；视觉
        重心要求以真实相机轴为基准继续计算，因此不能复用候选生成阶段的理想位姿。
        该函数不等待、不重试：调用方已有任务级恢复路径，TF 短暂不可用时应安全地
        放弃当前观察候选，而不是依据陈旧位姿继续运动。
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, self._camera_frame, rclpy.time.Time())
        except TransformException as error:
            self.get_logger().debug(
                f'[ARM][RECENTER] actual camera TF unavailable: {error}')
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = (
            translation.x, translation.y, translation.z,
            rotation.x, rotation.y, rotation.z, rotation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().warn(
                '[ARM][RECENTER] actual camera TF contains non-finite values')
            return None
        return (
            (float(translation.x), float(translation.y), float(translation.z)),
            (float(rotation.x), float(rotation.y), float(rotation.z),
             float(rotation.w)),
        )

    def _wait_for_joint_state(self, after_sequence=None):
        return self._wait_for_state(after_sequence=after_sequence)

    def _publish_observation_debug(
            self, event, candidate=None, rejection_reason='', *, target_id='',
            pre_error_u_px=None, pre_error_v_px=None, post_error_u_px=None,
            post_error_v_px=None, planned_angle_deg=None):
        """发布候选/重心完整 JSON；终端仅在 DEBUG 级别显示候选明细。"""
        if candidate is None:
            candidate_id = ''
            distance = camera_height = azimuth = 0.0
            camera_z_in_base = 0.0
            selection_phase = 'none'
            visible = ik_valid = selected = False
            condition = math.inf
            margin = motion = 0.0
            visible_fraction = 0.0
            projected_bbox = ()
            target_u = target_v = None
            observation_mode = getattr(self, '_observation_mode', 'ik')
            joint_positions_deg = ()
        else:
            candidate_id = candidate.candidate_id
            distance = candidate.distance_m
            camera_height = candidate.camera_height_m
            azimuth = candidate.azimuth_deg
            camera_z_in_base = float(candidate.camera_position[2])
            selection_phase = getattr(candidate, 'selection_phase', 'recovery')
            visible = bool(candidate.visible)
            ik_valid = candidate.ik_joints is not None
            selected = event == 'candidate_selected'
            condition = candidate.condition_number
            margin = candidate.min_joint_margin_rad
            motion = candidate.joint_motion_norm
            visible_fraction = candidate.visible_fraction
            projected_bbox = candidate.projected_bbox
            target_u = candidate.target_u_px
            target_v = candidate.target_v_px
            rejection_reason = rejection_reason or candidate.rejection_reason
            observation_mode = getattr(candidate, 'observation_mode', 'ik')
            joint_positions_deg = tuple(
                round(math.degrees(value), 4)
                for value in getattr(candidate, 'joint_positions', ()))
        payload = {
            'event': event,
            'mission_id': self._active_mission,
            'tree_id': self._active_tree,
            'candidate_id': candidate_id,
            'distance_m': distance,
            'camera_height_m': camera_height,
            'camera_height_in_base_m': camera_height,
            'camera_z_in_base_m': camera_z_in_base,
            'azimuth_deg': azimuth,
            'observation_phase': selection_phase,
            'observation_mode': observation_mode,
            'selection_policy': (
                'joint_preset_center_then_fan'
                if observation_mode == 'joint_presets' else 'center_then_fan'),
            'joint_positions_deg': joint_positions_deg,
            'visible': visible,
            'visible_fraction': visible_fraction,
            # 候选不再以树/叶高度包络作为准入条件；最终可见性由实际 YOLO
            # 检测和锁定流程判定，因此这里明确标出该旧阈值不再适用。
            'required_visible_fraction': None,
            'projected_bbox_xyxy': projected_bbox,
            'target_u_px': target_u,
            'target_v_px': target_v,
            'ik_valid': ik_valid,
            'condition_number': None if not math.isfinite(condition) else condition,
            'min_joint_margin_rad': margin,
            'joint_motion_norm': motion,
            'rejection_reason': rejection_reason,
            'selected': selected,
            'target_id': target_id,
            'pre_error_u_px': pre_error_u_px,
            'pre_error_v_px': pre_error_v_px,
            'post_error_u_px': post_error_u_px,
            'post_error_v_px': post_error_v_px,
            'planned_angle_deg': planned_angle_deg,
        }
        self._observation_debug_pub.publish(String(data=json.dumps(
            payload, sort_keys=True, separators=(',', ':'))))
        if event != 'candidate_rejected' or visible:
            self.get_logger().debug(
                f'[ARM][OBSERVE] event={event} id={candidate_id or "-"} '
                f'visible={visible} coverage={visible_fraction:.3f} '
                f'camera_z_in_base={camera_z_in_base:.3f} ik={ik_valid} '
                f'condition={payload["condition_number"]} '
                f'joint_margin={margin:.3f} reason={rejection_reason or "-"}')

    def _return_to_observation(self):
        if self._abort.is_set():
            return False
        index = self._observation_candidate_index
        if (0 <= index < len(self._observation_candidates) and
                getattr(self._observation_candidates[index], 'observation_mode', 'ik')
                == 'joint_presets'):
            return self.arm.move_joints(
                self._observation_candidates[index].joint_positions)
        if self._observation_pose is None:
            return False
        position, quat = self._observation_pose
        return self._move_to_pose((position, quat))

    def _move_to_pose(self, pose):
        position, quat = pose
        return self.arm.move_pose(
            position, quat, frame_id=self._base_frame,
            tolerance_position=self._observation_config[
                'position_tolerance_m'],
            tolerance_orientation=self._observation_config[
                'orientation_tolerance_rad'])
