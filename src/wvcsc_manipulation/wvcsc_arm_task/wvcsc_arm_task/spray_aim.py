"""喷嘴瞄准点服务与动态工距计算。

该混入类只负责从当前 C10 几何得到喷嘴投影目标；它不驱动 MoveIt、Servo 或喷洒
Action，因此仿真和实机可共用同一条补偿入口。
"""

import math
import time

from wvcsc_interfaces.srv import ComputeSprayAim

from .ik_observation import rotate_vector


WORKING_RANGE_MIN_M = 0.2
WORKING_RANGE_MAX_M = 2.0


class SprayAimMixin:
    def _request_spray_aim(self, cancel_requested):
        """读取标定喷嘴像素；缺失标定时拒绝进入重心或视觉伺服。"""
        working_range = float(getattr(self, '_working_range_override', 0.0))
        if working_range > 0.0:
            range_source = 'goal_override'
        else:
            working_range, range_error = self._dynamic_nozzle_range()
            if working_range is None:
                return False, range_error
            range_source = 'tree_geometry'
        deadline = time.monotonic() + float(
            self.get_parameter('aim_service_timeout_sec').value)
        while not self._aim_client.service_is_ready():
            if self._aborted(cancel_requested):
                return False, 'spray goal canceled'
            if time.monotonic() >= deadline:
                return False, 'nozzle aim service is unavailable'
            time.sleep(0.02)
        request = ComputeSprayAim.Request()
        request.working_range_m = float(working_range)
        future = self._aim_client.call_async(request)
        while not future.done():
            if self._aborted(cancel_requested):
                return False, 'spray goal canceled'
            if time.monotonic() >= deadline:
                return False, 'nozzle aim service timed out'
            time.sleep(0.02)
        try:
            response = future.result()
        except Exception as error:
            return False, f'nozzle aim service failed: {error}'
        if not response.success:
            return False, f'nozzle aim unavailable: {response.message}'
        values = (
            float(response.desired_u_px), float(response.desired_v_px),
            int(response.image_width), int(response.image_height),
            float(working_range),
        )
        if (not all(math.isfinite(value) for value in values[:2]) or
                values[2] <= 0 or values[3] <= 0 or
                not 0.0 <= values[0] < values[2] or
                not 0.0 <= values[1] < values[3]):
            return False, 'nozzle aim service returned an invalid image point'
        self._active_aim = values
        self.get_logger().info(
            '[ARM][AIM] calibrated nozzle target='
            f'({values[0]:.1f},{values[1]:.1f})px '
            f'aim_plane_range={values[4]:.2f}m source={range_source} '
            f'image={values[2]}x{values[3]}')
        return True, ''

    def _dynamic_nozzle_range(self):
        """Intersect current C10 optical axis with the tree's vertical plane."""
        if self._tree_in_base is None:
            return None, 'tree geometry is unavailable for dynamic nozzle aim'
        tree_x, tree_y, _tree_z = self._tree_in_base
        planar_range = math.hypot(tree_x, tree_y)
        if not math.isfinite(planar_range) or planar_range <= 1e-6:
            return None, 'tree planar range is invalid for dynamic nozzle aim'
        camera_pose = self._current_camera_pose()
        if camera_pose is None:
            return None, 'camera TF is unavailable for dynamic nozzle aim'
        origin, quaternion = camera_pose
        normal = (tree_x / planar_range, tree_y / planar_range, 0.0)
        optical_z = rotate_vector((0.0, 0.0, 1.0), quaternion)
        denominator = sum(normal[index] * optical_z[index] for index in range(3))
        numerator = planar_range - sum(
            normal[index] * origin[index] for index in range(3))
        if (not math.isfinite(denominator) or not math.isfinite(numerator) or
                denominator <= 0.2):
            return None, 'camera optical axis does not face the tree plane'
        working_range = numerator / denominator
        if (not math.isfinite(working_range) or
                not WORKING_RANGE_MIN_M <= working_range <=
                WORKING_RANGE_MAX_M):
            return None, (
                f'dynamic nozzle range is outside geometric bounds: '
                f'{working_range:.3f}m')
        return working_range, ''

    def _active_aim_pixel(self, image_width, image_height):
        """Scale one calibrated aim point to the current Target2D resolution."""
        aim = self._active_aim
        if aim is None or image_width <= 0 or image_height <= 0:
            return None
        desired_u, desired_v, aim_width, aim_height, _range_m = aim
        return (
            desired_u * float(image_width) / float(aim_width),
            desired_v * float(image_height) / float(aim_height),
        )
