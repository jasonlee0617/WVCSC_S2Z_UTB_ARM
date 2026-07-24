#!/usr/bin/env python3
"""Capture a stable AMCL docking pose and measured tree offset into YAML."""

import argparse
import math
from pathlib import Path
import sys
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from wvcsc_bringup.site_mission import (
    MAX_CAPTURE_POSITION_SPREAD_M,
    MAX_CAPTURE_POSITION_STDDEV_M,
    MAX_CAPTURE_YAW_SPREAD_RAD,
    MAX_CAPTURE_YAW_STDDEV_RAD,
    atomic_write_site,
    load_site_document,
    map_hashes,
    new_site_document,
    pose_sample_statistics,
    validate_site_document,
)
from wvcsc_bringup.field_route import (
    ARM_SPRAY_DURATION_SEC,
    INSPECT_POINT_IDS,
    ROUTE_POINT_IDS,
    load_field_route_document,
    new_field_route_document,
    validate_field_route_document,
)


class SitePoseCapture(Node):
    _INPUT_MAX_AGE_SEC = 1.0
    # AMCL may publish at roughly 1 Hz while the vehicle is stopped.  Allow
    # one publication-period of jitter and retry stale AMCL during sampling.
    _AMCL_MAX_AGE_SEC = 2.0
    _STOP_SETTLE_SEC = 1.0
    _NO_MOTION_UPDATE_PERIOD_SEC = 0.5
    # TF may need a short warm-up after AMCL starts publishing.  This is only
    # a retry window; the pose spread and covariance gates below are unchanged.
    _TF_CAPTURE_WINDOW_SEC = 8.0
    _TF_RETRY_PERIOD_SEC = 0.05
    _TF_LOG_PERIOD_SEC = 1.0

    def __init__(self):
        super().__init__('wvcsc_site_pose_capture')
        self._imu = None
        self._odom = None
        self._amcl = None
        self._last_amcl_quality = (0.0, 0.0)
        self._stable_since = None
        self._next_no_motion_update = 0.0
        self._no_motion_service_state = 'not checked'
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._no_motion_client = self.create_client(
            Empty, '/request_nomotion_update')
        self.create_subscription(Imu, '/imu', self._on_imu, 20)
        self.create_subscription(Odometry, '/ekf_odom', self._on_odom, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)

    @staticmethod
    def _yaw(quaternion):
        norm = math.sqrt(sum(value * value for value in (
            quaternion.x, quaternion.y, quaternion.z, quaternion.w)))
        if not math.isfinite(norm) or norm < 1e-6:
            raise ValueError('TF quaternion is invalid')
        x, y, z, w = (
            quaternion.x / norm, quaternion.y / norm,
            quaternion.z / norm, quaternion.w / norm)
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z))

    def _on_imu(self, _message):
        self._imu = time.monotonic()

    def _on_odom(self, message):
        now = time.monotonic()
        linear = math.hypot(
            message.twist.twist.linear.x, message.twist.twist.linear.y)
        angular = abs(message.twist.twist.angular.z)
        self._odom = now, linear, angular
        if linear <= 0.02 and angular <= 0.02:
            if self._stable_since is None:
                self._stable_since = now
        else:
            self._stable_since = None

    def _on_amcl(self, message):
        covariance = message.pose.covariance
        try:
            variances = (
                float(covariance[0]), float(covariance[7]),
                float(covariance[35]))
        except (IndexError, TypeError, ValueError):
            self._amcl = None
            return
        if not all(math.isfinite(value) and value >= 0.0 for value in variances):
            self._amcl = None
            return
        self._amcl = (
            time.monotonic(),
            max(math.sqrt(variances[0]), math.sqrt(variances[1])),
            math.sqrt(variances[2]),
        )
        self._last_amcl_quality = self._amcl[1:]

    def _request_no_motion_update(self, now):
        if now < self._next_no_motion_update:
            return
        self._next_no_motion_update = (
            now + self._NO_MOTION_UPDATE_PERIOD_SEC)
        if not self._no_motion_client.service_is_ready():
            self._no_motion_service_state = 'unavailable'
            return
        try:
            self._no_motion_client.call_async(Empty.Request())
        except Exception as error:  # rclpy reports transport errors here.
            self._no_motion_service_state = f'error: {error}'
            return
        self._no_motion_service_state = 'available'

    def _input_issues(self, now):
        issues = []
        if self._imu is None:
            issues.append('AHRS /imu has not published')
        elif now - self._imu > self._INPUT_MAX_AGE_SEC:
            issues.append(
                f'AHRS /imu is stale ({now - self._imu:.2f} s old)')

        if self._odom is None:
            issues.append('EKF odometry /ekf_odom has not published')
        else:
            age, linear, angular = (
                now - self._odom[0], self._odom[1], self._odom[2])
            if age > self._INPUT_MAX_AGE_SEC:
                issues.append(f'EKF odometry is stale ({age:.2f} s old)')
            if self._stable_since is None:
                issues.append(
                    f'vehicle is moving (linear={linear:.3f} m/s, '
                    f'angular={angular:.3f} rad/s)')
            elif now - self._stable_since < self._STOP_SETTLE_SEC:
                issues.append(
                    f'vehicle stop is settling '
                    f'({now - self._stable_since:.2f}/{self._STOP_SETTLE_SEC:.2f} s)')

        if self._amcl is None:
            issues.append('AMCL /amcl_pose has not published')
        elif now - self._amcl[0] > self._AMCL_MAX_AGE_SEC:
            issues.append(
                f'AMCL /amcl_pose is stale '
                f'({now - self._amcl[0]:.2f} s old, '
                f'limit={self._AMCL_MAX_AGE_SEC:.1f} s)')

        if self._no_motion_service_state != 'available':
            issues.append(
                'AMCL /request_nomotion_update service is '
                f'{self._no_motion_service_state}')
        return issues

    def _inputs_ready(self, now):
        return not self._input_issues(now)

    def _lookup_latest_transform(self):
        """Return the latest map pose, or a retryable TF exception."""
        try:
            return self._tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()), None
        except TransformException as error:
            return None, error

    @staticmethod
    def _quality_issues(quality):
        issues = []
        if quality['position_spread_m'] > MAX_CAPTURE_POSITION_SPREAD_M:
            issues.append(
                f"position spread {quality['position_spread_m']:.3f} m "
                f'exceeds {MAX_CAPTURE_POSITION_SPREAD_M:.2f} m')
        if quality['yaw_spread_rad'] > MAX_CAPTURE_YAW_SPREAD_RAD:
            issues.append(
                f"yaw spread {quality['yaw_spread_rad']:.3f} rad "
                f'exceeds {MAX_CAPTURE_YAW_SPREAD_RAD:.2f} rad')
        if quality['max_position_stddev_m'] > MAX_CAPTURE_POSITION_STDDEV_M:
            issues.append(
                'AMCL position standard deviation '
                f"{quality['max_position_stddev_m']:.3f} m exceeds "
                f'{MAX_CAPTURE_POSITION_STDDEV_M:.2f} m')
        if quality['max_yaw_stddev_rad'] > MAX_CAPTURE_YAW_STDDEV_RAD:
            issues.append(
                'AMCL yaw standard deviation '
                f"{quality['max_yaw_stddev_rad']:.3f} rad exceeds "
                f'{MAX_CAPTURE_YAW_STDDEV_RAD:.2f} rad')
        return issues

    @classmethod
    def _validate_quality(cls, quality):
        issues = cls._quality_issues(quality)
        if issues:
            raise RuntimeError(issues[0])

    def capture(self, timeout_sec=30.0, *, strict_capture=False,
                force_capture=False):
        # ``force_capture`` is retained as a compatibility alias for the
        # relaxed mode.  Strict validation is now opt-in for final acceptance.
        strict_capture = bool(strict_capture) and not force_capture
        deadline = time.monotonic() + timeout_sec
        if not strict_capture:
            self.get_logger().warning(
                '[SITE] relaxed capture: freshness, stop and quality gates '
                'are warnings only; initial sensor data and TF are required')
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                now = time.monotonic()
                self._request_no_motion_update(now)
                if self._imu is not None and self._odom is not None and self._amcl is not None:
                    break
            else:
                raise RuntimeError(
                    'timed out waiting for initial /imu, /ekf_odom and '
                    '/amcl_pose messages')
        else:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                now = time.monotonic()
                self._request_no_motion_update(now)
                if self._inputs_ready(now):
                    break
            else:
                raise RuntimeError('timed out waiting for safe capture inputs: ' +
                                   '; '.join(self._input_issues(time.monotonic())))

        samples = []
        position_stddevs = []
        yaw_stddevs = []
        next_sample = time.monotonic()
        sample_deadline = next_sample + self._TF_CAPTURE_WINDOW_SEC
        tf_wait_started = None
        tf_retry_count = 0
        last_tf_error = None
        next_tf_log = next_sample
        amcl_wait_started = None
        amcl_retry_count = 0
        last_amcl_issue = None
        next_amcl_log = next_sample
        while rclpy.ok() and len(samples) < 30 and time.monotonic() < sample_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            self._request_no_motion_update(now)
            if now < next_sample:
                continue
            if strict_capture:
                input_issues = self._input_issues(now)
                amcl_stale_issues = [
                    issue for issue in input_issues
                    if issue.startswith('AMCL /amcl_pose is stale')]
                if (amcl_stale_issues and
                        len(amcl_stale_issues) == len(input_issues)):
                    amcl_retry_count += 1
                    last_amcl_issue = amcl_stale_issues[-1]
                    if amcl_wait_started is None:
                        amcl_wait_started = now
                    if now >= next_amcl_log:
                        self.get_logger().warning(
                            '[SITE] waiting for fresh AMCL pose: '
                            f'{now - amcl_wait_started:.1f} s, '
                            f'retries={amcl_retry_count}, '
                            f'last_error={last_amcl_issue}')
                        next_amcl_log = now + self._TF_LOG_PERIOD_SEC
                    next_sample = now + self._TF_RETRY_PERIOD_SEC
                    continue
                if input_issues:
                    raise RuntimeError(
                        'capture interrupted: ' + '; '.join(input_issues))
            else:
                input_issues = self._input_issues(now)
                if input_issues and now >= next_tf_log:
                    self.get_logger().warning(
                        '[SITE] relaxed capture continues with: ' +
                        '; '.join(input_issues))
                    next_tf_log = now + self._TF_LOG_PERIOD_SEC
            transform, tf_error = self._lookup_latest_transform()
            if transform is None:
                tf_retry_count += 1
                last_tf_error = str(tf_error)
                if tf_wait_started is None:
                    tf_wait_started = now
                if now >= next_tf_log:
                    self.get_logger().warning(
                        '[SITE] waiting for map -> base_footprint TF: '
                        f'{now - tf_wait_started:.1f} s, '
                        f'retries={tf_retry_count}, last_error={last_tf_error}')
                    next_tf_log = now + self._TF_LOG_PERIOD_SEC
                next_sample = now + self._TF_RETRY_PERIOD_SEC
                continue
            translation = transform.transform.translation
            yaw = self._yaw(transform.transform.rotation)
            if not all(math.isfinite(value) for value in (
                    translation.x, translation.y, yaw)):
                tf_retry_count += 1
                last_tf_error = 'map -> base_footprint transform is non-finite'
                next_sample = now + self._TF_RETRY_PERIOD_SEC
                continue
            samples.append((translation.x, translation.y, yaw))
            position_stddevs.append(self._last_amcl_quality[0])
            yaw_stddevs.append(self._last_amcl_quality[1])
            next_sample = now + 0.1
        if len(samples) < 30:
            message = f'only captured {len(samples)}/30 valid samples'
            if tf_retry_count:
                message += (
                    f'; TF retries={tf_retry_count}, '
                    f'last_error={last_tf_error}')
            if amcl_retry_count:
                message += (
                    f'; AMCL retries={amcl_retry_count}, '
                    f'last_error={last_amcl_issue}')
            raise RuntimeError(message)
        x, y, yaw, position_spread, yaw_spread = pose_sample_statistics(samples)
        quality = {
            'samples': len(samples),
            'position_spread_m': float(position_spread),
            'yaw_spread_rad': float(yaw_spread),
            'max_position_stddev_m': float(max(position_stddevs)),
            'max_yaw_stddev_rad': float(max(yaw_stddevs)),
        }
        if strict_capture:
            self._validate_quality(quality)
        else:
            quality_issues = self._quality_issues(quality)
            if quality_issues:
                self.get_logger().warning(
                    '[SITE] relaxed capture quality warning: ' +
                    '; '.join(quality_issues))
            quality['validation_bypassed'] = True
        return (x, y, yaw), quality


def _arguments(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', default='~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/wvcsc_sites/corn_site.yaml')
    parser.add_argument('--map', required=True)
    operation = parser.add_mutually_exclusive_group(required=True)
    operation.add_argument('--capture-home', action='store_true')
    operation.add_argument('--target-id')
    operation.add_argument('--route-point', choices=ROUTE_POINT_IDS)
    parser.add_argument('--tree-x-m', type=float)
    parser.add_argument('--tree-y-m', type=float)
    parser.add_argument('--tree-id')
    parser.add_argument('--spray-duration', type=float, default=5.0)
    parser.add_argument('--arm-spray-duration', type=float,
                        default=ARM_SPRAY_DURATION_SEC)
    parser.add_argument('--site-id', default='corn_site')
    parser.add_argument('--mission-id', default='corn_measured_001')
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    parser.add_argument('--update', action='store_true')
    parser.add_argument(
        '--force-capture', action='store_true',
        help='compatibility alias for the default relaxed capture mode')
    parser.add_argument(
        '--strict-capture', action='store_true',
        help='require freshness, stop, quality and footprint gates')
    return parser.parse_args(argv)


def _document(args):
    path = Path(args.file).expanduser()
    if path.exists():
        document = (load_field_route_document(path) if args.route_point
                    else load_site_document(path))
        if document.get('map') != map_hashes(args.map):
            raise ValueError('existing site file is bound to a different map')
        return document
    return (new_field_route_document(args.site_id, args.mission_id, args.map)
            if args.route_point
            else new_site_document(args.site_id, args.mission_id, args.map))


def main():
    argv = remove_ros_args(args=sys.argv)[1:]
    if argv and argv[0] == '--':
        argv = argv[1:]
    args = _arguments(argv)
    strict_capture = bool(args.strict_capture and not args.force_capture)
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0.0:
        raise SystemExit('--timeout-sec must be finite and positive')
    if args.target_id and (
            args.tree_x_m is None or args.tree_y_m is None):
        raise SystemExit('target capture requires --tree-x-m and --tree-y-m')
    if args.route_point:
        inspecting = args.route_point in INSPECT_POINT_IDS
        if inspecting and (not str(args.tree_id or '').strip()
                           or args.tree_x_m is None or args.tree_y_m is None):
            raise SystemExit(
                'point_2 and point_3 require --tree-id --tree-x-m --tree-y-m')
        if not inspecting and any(value is not None for value in (
                args.tree_id, args.tree_x_m, args.tree_y_m)):
            raise SystemExit(
                'only point_2 and point_3 accept tree offset arguments')
        if not math.isclose(
                args.arm_spray_duration, ARM_SPRAY_DURATION_SEC, abs_tol=1e-9):
            raise SystemExit(
                f'--arm-spray-duration must be {ARM_SPRAY_DURATION_SEC:.1f}')

    try:
        document = _document(args)
    except ValueError as error:
        raise SystemExit(str(error)) from error

    rclpy.init(args=sys.argv)
    node = SitePoseCapture()
    try:
        pose, quality = node.capture(
            args.timeout_sec,
            strict_capture=strict_capture,
            force_capture=args.force_capture)
        pose_mapping = {'x': pose[0], 'y': pose[1], 'yaw': pose[2]}
        if args.route_point:
            route_steps = document['mission']['route_steps']
            step = next(item for item in route_steps
                        if item['point_id'] == args.route_point)
            if step.get('navigation_pose') is not None and not args.update:
                raise ValueError(
                    f'{args.route_point} already exists; pass --update to replace it')
            step['navigation_pose'] = pose_mapping
            step['capture_quality'] = quality
            if args.route_point in INSPECT_POINT_IDS:
                step['tree_id'] = str(args.tree_id).strip()
                step['tree_offset_arm_base_m'] = [
                    float(args.tree_x_m), float(args.tree_y_m)]
                step['tree_base_z_m'] = 0.0
                step['arm_spray_duration'] = ARM_SPRAY_DURATION_SEC
            complete = all(item.get('navigation_pose') is not None
                           for item in route_steps)
            if complete:
                validate_field_route_document(
                    document, args.map,
                    require_capture_quality=strict_capture,
                    require_free_space=strict_capture)
        else:
            mission = document['mission']
            if args.capture_home:
                mission['home_pose'] = pose_mapping
                mission['home_capture_quality'] = quality
            else:
                target_id = str(args.target_id).strip()
                if not target_id:
                    raise ValueError('target_id must be non-empty')
                targets = mission.setdefault('targets', [])
                existing = next(
                    (item for item in targets if item.get('target_id') == target_id),
                    None)
                if existing is not None and not args.update:
                    raise ValueError(
                        f'{target_id} already exists; pass --update to replace it')
                target = {
                    'target_id': target_id,
                    'docking_pose': pose_mapping,
                    'tree_offset_arm_base_m': {
                        'reference': 'alicia_base_link_xy',
                        'x_m': float(args.tree_x_m),
                        'y_m': float(args.tree_y_m),
                    },
                    'tree_base_z_m': 0.0,
                    'spray_duration': float(args.spray_duration),
                    'capture_quality': quality,
                }
                if existing is None:
                    targets.append(target)
                else:
                    targets[targets.index(existing)] = target
                validate_site_document(
                    document, args.map,
                    require_capture_quality=strict_capture,
                    require_free_space=strict_capture)
        atomic_write_site(args.file, document, backup=Path(args.file).expanduser().exists())
        node.get_logger().info(
            f'[SITE] captured '
            f'{args.route_point or ("HOME" if args.capture_home else args.target_id)} '
            f'pose=({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f}) '
            f'file={Path(args.file).expanduser()}')
        return 0
    except (RuntimeError, ValueError) as error:
        node.get_logger().error(f'[SITE] capture rejected: {error}')
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
