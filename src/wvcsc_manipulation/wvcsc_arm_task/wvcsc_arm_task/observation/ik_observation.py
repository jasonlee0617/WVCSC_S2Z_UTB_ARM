"""IK 观察位姿的纯几何与运动学实现。

本模块不依赖 ROS 节点状态，集中提供观察候选生成、相机/工具位姿换算与
URDF 雅可比安全评估。ROS 快照、候选执行和恢复流程位于
:mod:`observation_flow`。
"""

import math

import numpy as np
from urdf_parser_py.urdf import URDF

from .candidate import build_candidate


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
                    candidates.append(build_candidate(
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

__all__ = (
    'ObservationOptimizer',
    'camera_look_at_pose',
    'camera_orientation_for_pixel',
    'camera_pose_from_bearing',
    'normalize_quaternion',
    'quaternion_conjugate',
    'quaternion_from_matrix',
    'quaternion_multiply',
    'recenter_camera_pose',
    'rotate_vector',
    'rotation_matrix_from_quaternion',
    'tool_pose_from_camera_pose',
    'transform_point',
)
