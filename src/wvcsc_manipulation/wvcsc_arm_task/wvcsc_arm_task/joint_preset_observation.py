"""固定关节预设观察策略。

本模块只处理实机验证过的左右扫描序列。它不计算相机 look-at、IK 或雅可比；
这些职责留给 ``ik_observation`` 与 ``observation_flow``，因此切换
``observation_mode`` 不会改变 Action、取消或安全锁语义。
"""

import math

from .observation_flow import _build_candidate


DEFAULT_JOINT_PRESETS_DEG = {
    'center': (95.3, -136.9, -71.0, 7.7, 57.3, -4.4),
    'fan_left': (52.2, -131.7, -55.4, -58.9, 76.5, 18.2),
    'fan_right': (118.5, -129.4, -55.8, 47.6, 66.2, -17.1),
    'right_center': (-105.4, -127.8, -50.5, -15.4, 71.2, -4.9),
    'right_fan_left': (-139.8, -128.6, -57.3, -70.1, 79.7, 13.0),
    'right_fan_right': (-70.8, -126.8, -50.6, 32.2, 69.8, -12.1),
}


class JointPresetObservationMixin:
    """将配置的角度序列转换为左右侧候选，不参与 IK 观察位生成。"""

    def _joint_preset_parameters(self):
        mode = str(self.get_parameter('observation_mode').value).strip().lower()
        if mode not in {'ik', 'joint_presets'}:
            raise ValueError('observation_mode must be ik or joint_presets')
        if mode == 'ik':
            return mode, ()
        presets_by_side = {}
        for side, definitions in (
                ('left', (
                    ('center', 'joint_preset_center_deg'),
                    ('fan_left', 'joint_preset_fan_left_deg'),
                    ('fan_right', 'joint_preset_fan_right_deg'),
                )),
                ('right', (
                    ('center', 'joint_preset_right_center_deg'),
                    ('fan_left', 'joint_preset_right_fan_left_deg'),
                    ('fan_right', 'joint_preset_right_fan_right_deg'),
                ))):
            presets = []
            for name, parameter in definitions:
                try:
                    degrees = tuple(
                        float(value) for value in self.get_parameter(parameter).value)
                except (TypeError, ValueError) as error:
                    raise ValueError(
                        f'{parameter} must contain six finite degrees') from error
                if (len(degrees) != 6 or
                        not all(math.isfinite(value) for value in degrees)):
                    raise ValueError(f'{parameter} must contain six finite degrees')
                presets.append((name, tuple(math.radians(value) for value in degrees)))
            presets_by_side[side] = tuple(presets)
        return mode, presets_by_side

    def _select_joint_preset_side(self, tree_in_base):
        """Choose the fixed left/right scan set from the tree's arm-frame Y."""
        epsilon = self._joint_preset_side_epsilon_m
        if tree_in_base[1] > epsilon:
            self._joint_preset_side = 'left'
            return True
        if tree_in_base[1] < -epsilon:
            self._joint_preset_side = 'right'
            return True
        self._joint_preset_side = ''
        self._observation_failure_reason = (
            'joint_preset_tree_side_ambiguous '
            f'(tree_y_m={tree_in_base[1]:.3f}, requires_abs_y>{epsilon:.3f})')
        self.get_logger().error(
            '[ARM][OBSERVE] mode=joint_presets rejects tree too close '
            f'to the base Y axis: y={tree_in_base[1]:.3f} m, '
            f'epsilon={epsilon:.3f} m')
        self._publish_observation_debug(
            'search_failed', rejection_reason=self._observation_failure_reason)
        return False

    def _prepare_joint_preset_observation_candidates(self):
        """Prepare center then fan candidates for the selected physical side."""
        self._observation_candidates = []
        self._observation_candidate_index = -1
        presets = self._joint_preset_positions.get(self._joint_preset_side, ())
        if not presets:
            self._observation_failure_reason = (
                f'joint_preset_side_unconfigured ({self._joint_preset_side or "none"})')
            self.get_logger().error(
                '[ARM][OBSERVE] mode=joint_presets has no configured presets '
                f'for side={self._joint_preset_side or "none"}')
            self._publish_observation_debug(
                'search_failed', rejection_reason=self._observation_failure_reason)
            return False
        for index, (name, joints) in enumerate(presets):
            candidate = _build_candidate(
                candidate_id=f'joint_preset_{self._joint_preset_side}_{name}',
                distance_m=0.0,
                camera_height_m=0.0,
                azimuth_deg=(0.0, -1.0, 1.0)[index],
                camera_position=(0.0, 0.0, 0.0),
                camera_quat=(0.0, 0.0, 0.0, 1.0),
                tool_position=(0.0, 0.0, 0.0),
                tool_quat=(0.0, 0.0, 0.0, 1.0),
                visible=True,
                visible_margin_px=math.inf,
                visible_fraction=1.0,
                observation_mode='joint_presets',
                joint_positions=joints,
            )
            candidate.selection_phase = 'center_initial' if index == 0 else 'fan_scan'
            self._observation_candidates.append(candidate)
        self.get_logger().info(
            '[ARM][OBSERVE] mode=joint_presets tree_in_base='
            f'({self._tree_in_base[0]:.2f},{self._tree_in_base[1]:.2f},'
            f'{self._tree_in_base[2]:.2f}) side={self._joint_preset_side} prepared='
            f'{len(self._observation_candidates)}')
        return True
