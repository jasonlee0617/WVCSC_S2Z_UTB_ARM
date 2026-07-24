#!/usr/bin/env python3
"""Real-only five-point route: wide spray while driving, arm spray at stops.

This node intentionally does not use ``/mission/load_manual``.  That API is
shared with simulation and the Nav2 Qt tools; the fixed field demonstration
has a different, fail-closed route contract and therefore owns its own
orchestration boundary.
"""

from __future__ import annotations

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from wvcsc_bringup.field_route import (
    FieldRouteStep,
    load_field_route_document,
    validate_field_route_document,
)
from wvcsc_interfaces.action import ExecuteSpray
from wvcsc_interfaces.msg import MissionStatus
from wvcsc_interfaces.srv import SetRelay
from wvcsc_mission_manager.core import StopDetector, tree_hint_from_arm_base_offset


class FieldRouteManager(Node):
    """Fail-closed sequencer for the physical five-point demonstration."""

    STARTING = 'STARTING'
    NAVIGATING = 'NAVIGATING'
    VERIFYING_INSPECT_STOP = 'VERIFYING_INSPECT_STOP'
    ARM_SPRAYING = 'ARM_SPRAYING'
    VERIFYING_FINISH_STOP = 'VERIFYING_FINISH_STOP'
    COMPLETED = 'COMPLETED'
    FAILED = 'FAILED'

    def __init__(self):
        super().__init__('wvcsc_field_route_manager')
        self._declare_parameters()
        mission_file = str(self.get_parameter('mission_file').value)
        map_file = str(self.get_parameter('map_file').value)
        document = load_field_route_document(mission_file)
        self._steps = validate_field_route_document(document, map_file)
        self._mission_id = str(document['mission']['mission_id'])
        mount = document['arm_base_mount']
        self._arm_base_forward = float(mount['x_m'])
        self._arm_base_left = float(mount['y_m'])

        self._wide_channel = self._positive_channel('wide_relay_channel')
        self._arm_channel = self._positive_channel('arm_relay_channel')
        if self._wide_channel == self._arm_channel:
            raise ValueError('wide_relay_channel and arm_relay_channel must differ')
        self._map_frame = str(self.get_parameter('map_frame').value).strip()
        if not self._map_frame:
            raise ValueError('map_frame must be non-empty')

        self._state = self.STARTING
        self._index = 0
        self._completed_inspects = 0
        self._last_error = ''
        self._ready_logged = False
        self._nav_goal = None
        self._arm_goal = None
        self._nav_active = False
        self._arm_active = False
        self._nav_deadline = 0.0
        self._arm_deadline = 0.0
        self._relay_deadline = 0.0
        self._relay_request_id = 0
        self._initial_pose_received = False
        self._nav_stack_active = False
        self._nav_state_request_active = False
        self._last_nav_state_log = 0.0
        self._stop_detector = StopDetector(
            linear_threshold=float(self.get_parameter('linear_stop_threshold').value),
            angular_threshold=float(self.get_parameter('angular_stop_threshold').value),
            stable_duration=float(self.get_parameter('stop_stable_duration_sec').value),
            stale_timeout=float(self.get_parameter('odom_stale_timeout_sec').value),
            timeout=float(self.get_parameter('stop_verify_timeout_sec').value),
        )

        self._status_pub = self.create_publisher(MissionStatus, '/mission/status', 10)
        self._nav_client = ActionClient(
            self, NavigateToPose,
            str(self.get_parameter('nav_action_name').value))
        self._nav_state_client = self.create_client(
            GetState, '/bt_navigator/get_state')
        self._arm_client = ActionClient(
            self, ExecuteSpray,
            str(self.get_parameter('arm_action_name').value))
        self._relay_client = self.create_client(
            SetRelay, str(self.get_parameter('relay_service_name').value))
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self._on_initial_pose, 10)
        self.create_service(Trigger, '/field_route/cancel', self._on_cancel)
        self._timer = self.create_timer(0.10, self._tick)
        self._publish_status('waiting for Nav2, arm action and relay service')

    def _declare_parameters(self):
        defaults = {
            'mission_file': '',
            'map_file': '',
            'map_frame': 'map',
            'nav_action_name': '/navigate_to_pose',
            'arm_action_name': '/arm/execute_spray',
            'relay_service_name': '/relay/set',
            'wide_relay_channel': 1,
            'arm_relay_channel': 2,
            'auto_start': True,
            'wait_for_initial_pose': False,
            'wait_for_nav_active': False,
            'relay_service_timeout_sec': 2.0,
            'nav_goal_timeout_sec': 120.0,
            'arm_goal_timeout_sec': 180.0,
            'linear_stop_threshold': 0.03,
            'angular_stop_threshold': 0.03,
            'stop_stable_duration_sec': 1.0,
            'odom_stale_timeout_sec': 1.0,
            'stop_verify_timeout_sec': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _positive_channel(self, parameter):
        value = int(self.get_parameter(parameter).value)
        if not 1 <= value <= 255:
            raise ValueError(f'{parameter} must be within 1..255')
        return value

    @property
    def _step(self) -> FieldRouteStep | None:
        return self._steps[self._index] if self._index < len(self._steps) else None

    def _on_odom(self, message):
        self._stop_detector.update(
            time.monotonic(),
            math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y),
            abs(message.twist.twist.angular.z),
        )

    def _on_initial_pose(self, _message):
        self._initial_pose_received = True

    def _poll_nav_state(self, now):
        """Wait until bt_navigator is ACTIVE, not merely discoverable."""
        if self._nav_stack_active:
            return True
        if not self._nav_state_client.service_is_ready():
            if now - self._last_nav_state_log >= 5.0:
                self.get_logger().info(
                    '[FIELD_ROUTE] waiting for active Nav2 lifecycle state')
                self._last_nav_state_log = now
            return False
        if self._nav_state_request_active:
            return False
        self._nav_state_request_active = True
        try:
            future = self._nav_state_client.call_async(GetState.Request())
        except Exception as error:
            self._nav_state_request_active = False
            self.get_logger().warning(
                f'[FIELD_ROUTE] Nav2 lifecycle query failed: {error}')
            return False

        def done(result_future):
            self._nav_state_request_active = False
            try:
                response = result_future.result()
                active = response.current_state.id == 3  # PRIMARY_STATE_ACTIVE
                if active and not self._nav_stack_active:
                    self.get_logger().info(
                        '[FIELD_ROUTE] Nav2 lifecycle is ACTIVE')
                self._nav_stack_active = active
            except Exception as error:
                self.get_logger().warning(
                    f'[FIELD_ROUTE] Nav2 lifecycle response failed: {error}')

        future.add_done_callback(done)
        return False

    def _on_cancel(self, _request, response):
        if self._state in (self.COMPLETED, self.FAILED):
            response.success = False
            response.message = f'route already {self._state.lower()}'
            return response
        self._fail('route canceled through /field_route/cancel')
        response.success = True
        response.message = 'route canceled and both relay channels were commanded off'
        return response

    def _clients_ready(self):
        return (
            self._nav_client.server_is_ready()
            and self._arm_client.server_is_ready()
            and self._relay_client.service_is_ready()
        )

    def _tick(self):
        now = time.monotonic()
        if self._relay_deadline and now >= self._relay_deadline:
            self._relay_deadline = 0.0
            self._relay_request_id += 1
            self._fail('relay service response timed out')
            return
        if (self._state == self.NAVIGATING and self._nav_active
                and now >= self._nav_deadline):
            self._fail('Nav2 goal timed out')
            return
        if (self._state == self.ARM_SPRAYING and self._arm_active
                and now >= self._arm_deadline):
            self._fail('arm spray action timed out')
            return
        if self._state == self.STARTING:
            if not bool(self.get_parameter('auto_start').value):
                return
            if (bool(self.get_parameter('wait_for_initial_pose').value)
                    and not self._initial_pose_received):
                if not self._ready_logged:
                    self.get_logger().info(
                        '[FIELD_ROUTE] waiting for RViz initial pose on /amcl_pose')
                    self._ready_logged = True
                return
            if (bool(self.get_parameter('wait_for_nav_active').value)
                    and not self._poll_nav_state(now)):
                return
            if not self._clients_ready():
                if not self._ready_logged:
                    self.get_logger().info(
                        '[FIELD_ROUTE] waiting for /navigate_to_pose, '
                        '/arm/execute_spray and /relay/set')
                    self._ready_logged = True
                return
            self.get_logger().info(
                f'[FIELD_ROUTE] services ready; auto-starting mission={self._mission_id}')
            self._start_navigation()
            return

        if self._state not in (
                self.VERIFYING_INSPECT_STOP, self.VERIFYING_FINISH_STOP):
            return
        stop_state = self._stop_detector.status(now)
        if stop_state == StopDetector.STABLE:
            self._stop_detector.stop()
            if self._state == self.VERIFYING_INSPECT_STOP:
                self._send_arm_goal()
            else:
                self._complete()
        elif stop_state in (StopDetector.STALE, StopDetector.TIMEOUT):
            self._fail(f'vehicle stop verification {stop_state}')

    def _relay(self, channel, enabled, duration, continuation, context):
        if self._state in (self.COMPLETED, self.FAILED):
            return
        if not self._relay_client.service_is_ready():
            self._fail(f'{context}: /relay/set is unavailable')
            return
        self.get_logger().info(
            f'[FIELD_ROUTE] relay channel={int(channel)} '
            f'enabled={bool(enabled)} duration={float(duration):.1f}s '
            f'context={context}')
        request = SetRelay.Request()
        request.channel = int(channel)
        request.enabled = bool(enabled)
        request.duration = float(duration)
        self._relay_request_id += 1
        request_id = self._relay_request_id
        self._relay_deadline = time.monotonic() + float(
            self.get_parameter('relay_service_timeout_sec').value)
        try:
            future = self._relay_client.call_async(request)
        except Exception as error:
            self._relay_deadline = 0.0
            self._fail(f'{context}: relay request failed: {error}')
            return

        def done(result_future):
            if request_id != self._relay_request_id:
                return
            self._relay_deadline = 0.0
            if self._state in (self.COMPLETED, self.FAILED):
                return
            try:
                response = result_future.result()
            except Exception as error:
                self._fail(f'{context}: relay transport failed: {error}')
                return
            if response is None or not response.success:
                message = '' if response is None else response.message
                self._fail(f'{context}: relay rejected request: {message}')
                return
            continuation()

        future.add_done_callback(done)

    def _command_all_off(self):
        """Best-effort emergency off; it must not depend on another response."""
        for channel in (self._wide_channel, self._arm_channel):
            if not self._relay_client.service_is_ready():
                self.get_logger().error(
                    f'[FIELD_ROUTE] cannot command relay channel {channel} off: service unavailable')
                continue
            request = SetRelay.Request()
            request.channel = channel
            request.enabled = False
            request.duration = 0.0
            try:
                self._relay_client.call_async(request)
            except Exception as error:
                self.get_logger().error(
                    f'[FIELD_ROUTE] relay channel {channel} off request failed: {error}')

    def _start_navigation(self):
        if self._state in (self.COMPLETED, self.FAILED):
            return
        step = self._step
        if step is None:
            self._fail('route ended before finish step')
            return
        # Wide spray state is explicit at every transition.  This avoids
        # relying on the relay's previous latched state after a restart.
        # The wide-spray relay is on while the vehicle is traversing the
        # field.  It is re-enabled after each inspect stop and remains on
        # while driving from point_3 to point_4; point_4 is the stop where it
        # is turned off before the final leg.
        if self._index in (0, 2, 3):
            self._relay(
                self._wide_channel, True, 0.0, self._send_nav_goal,
                f'{step.point_id}: enable wide spray')
        else:
            self._send_nav_goal()

    def _send_nav_goal(self):
        step = self._step
        if step is None or self._state in (self.COMPLETED, self.FAILED):
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = step.navigation_pose['x']
        goal.pose.pose.position.y = step.navigation_pose['y']
        goal.pose.pose.orientation.z = math.sin(step.navigation_pose['yaw'] / 2.0)
        goal.pose.pose.orientation.w = math.cos(step.navigation_pose['yaw'] / 2.0)
        self._state = self.NAVIGATING
        self._nav_active = True
        self._nav_deadline = time.monotonic() + float(
            self.get_parameter('nav_goal_timeout_sec').value)
        self._publish_status(f'navigating {step.point_id} ({step.role})')
        try:
            future = self._nav_client.send_goal_async(goal)
        except Exception as error:
            self._fail(f'{step.point_id}: Nav2 goal send failed: {error}')
            return

        def accepted(done_future):
            try:
                handle = done_future.result()
            except Exception as error:
                self._fail(f'{step.point_id}: Nav2 goal transport failed: {error}')
                return
            if self._state in (self.COMPLETED, self.FAILED):
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()
                return
            if handle is None or not handle.accepted:
                self._fail(f'{step.point_id}: Nav2 rejected goal')
                return
            self._nav_goal = handle
            handle.get_result_async().add_done_callback(self._on_nav_result)

        future.add_done_callback(accepted)

    def _on_nav_result(self, future):
        self._nav_active = False
        self._nav_goal = None
        self._nav_deadline = 0.0
        if self._state in (self.COMPLETED, self.FAILED):
            return
        step = self._step
        try:
            wrapped = future.result()
        except Exception as error:
            self._fail(f'Nav2 result transport failed: {error}')
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(
                f'{step.point_id if step else "unknown"}: Nav2 failed with status={wrapped.status}')
            return
        if step is None:
            self._fail('Nav2 completed with no current route step')
            return
        self.get_logger().info(f'[FIELD_ROUTE] arrived at {step.point_id}')
        if step.role == 'wide_start':
            self._index += 1
            self._start_navigation()
        elif step.role == 'inspect':
            self._relay(
                self._wide_channel, False, 0.0, self._begin_inspect_stop,
                f'{step.point_id}: disable wide spray before arm motion')
        elif step.role == 'wide_stop':
            self._relay(
                self._wide_channel, False, 0.0,
                self._advance_after_wide_stop,
                f'{step.point_id}: disable wide spray before final leg')
        elif step.role == 'finish':
            self._relay(
                self._wide_channel, False, 0.0,
                self._disable_arm_before_finish,
                'finish: ensure wide spray is off')
        else:
            self._fail(f'unsupported route role: {step.role}')

    def _begin_inspect_stop(self):
        if self._state in (self.COMPLETED, self.FAILED):
            return
        self._state = self.VERIFYING_INSPECT_STOP
        self._stop_detector.start(time.monotonic())
        self._publish_status('vehicle arrived; verifying stop before arm spray')

    def _send_arm_goal(self):
        step = self._step
        if step is None or step.role != 'inspect':
            self._fail('inspect stop completed without an inspect route step')
            return
        assert step.tree_id and step.tree_offset_arm_base_m is not None
        assert step.tree_base_z_m is not None and step.arm_spray_duration is not None
        hint_x, hint_y, hint_z = tree_hint_from_arm_base_offset(
            (step.navigation_pose['x'], step.navigation_pose['y'], step.navigation_pose['yaw']),
            step.tree_offset_arm_base_m[0], step.tree_offset_arm_base_m[1],
            step.tree_base_z_m, self._arm_base_forward, self._arm_base_left)
        goal = ExecuteSpray.Goal()
        goal.mission_id = self._mission_id
        goal.tree_id = step.tree_id
        goal.spray_duration = float(step.arm_spray_duration)
        goal.tree_hint = PointStamped()
        goal.tree_hint.header.frame_id = self._map_frame
        goal.tree_hint.header.stamp = self.get_clock().now().to_msg()
        goal.tree_hint.point.x = hint_x
        goal.tree_hint.point.y = hint_y
        goal.tree_hint.point.z = hint_z
        self._state = self.ARM_SPRAYING
        self._arm_active = True
        self._arm_deadline = time.monotonic() + float(
            self.get_parameter('arm_goal_timeout_sec').value)
        self._publish_status(f'arm spraying tree={step.tree_id} duration={goal.spray_duration:.1f}s')
        try:
            future = self._arm_client.send_goal_async(goal)
        except Exception as error:
            self._fail(f'{step.point_id}: arm goal send failed: {error}')
            return

        def accepted(done_future):
            try:
                handle = done_future.result()
            except Exception as error:
                self._fail(f'{step.point_id}: arm goal transport failed: {error}')
                return
            if self._state in (self.COMPLETED, self.FAILED):
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()
                return
            if handle is None or not handle.accepted:
                self._fail(f'{step.point_id}: arm rejected spray goal')
                return
            self._arm_goal = handle
            handle.get_result_async().add_done_callback(self._on_arm_result)

        future.add_done_callback(accepted)

    def _on_arm_result(self, future):
        self._arm_active = False
        self._arm_goal = None
        self._arm_deadline = 0.0
        if self._state in (self.COMPLETED, self.FAILED):
            return
        step = self._step
        try:
            wrapped = future.result()
        except Exception as error:
            self._fail(f'arm result transport failed: {error}')
            return
        result = wrapped.result
        if (wrapped.status != GoalStatus.STATUS_SUCCEEDED or result is None
                or not result.success or result.error_code != ExecuteSpray.Result.OK):
            code = 'none' if result is None else str(result.error_code)
            message = '' if result is None else result.message
            self._fail(
                f'{step.point_id if step else "unknown"}: arm spray failed '
                f'status={wrapped.status} code={code}: {message}')
            return
        self._completed_inspects += 1
        self._index += 1
        self._state = self.NAVIGATING
        self._start_navigation()

    def _disable_arm_before_finish(self):
        self._relay(
            self._arm_channel, False, 0.0, self._begin_finish_stop,
            'finish: ensure arm spray is off')

    def _advance_after_wide_stop(self):
        """Leave point_4 only after the wide-spray relay is confirmed off."""
        if self._state in (self.COMPLETED, self.FAILED):
            return
        self._index += 1
        self._start_navigation()

    def _begin_finish_stop(self):
        if self._state in (self.COMPLETED, self.FAILED):
            return
        self._state = self.VERIFYING_FINISH_STOP
        self._stop_detector.start(time.monotonic())
        self._publish_status('finish reached; verifying vehicle is stationary')

    def _complete(self):
        self._state = self.COMPLETED
        self._command_all_off()
        self._publish_status('five-point route completed; both relay channels commanded off')
        self.get_logger().info('[FIELD_ROUTE][SUCCESS] five-point route completed')

    def _fail(self, reason):
        if self._state in (self.COMPLETED, self.FAILED):
            return
        self._last_error = str(reason)
        self._stop_detector.stop()
        self._relay_request_id += 1
        self._relay_deadline = 0.0
        if self._nav_goal is not None:
            self._nav_goal.cancel_goal_async()
        if self._arm_goal is not None:
            self._arm_goal.cancel_goal_async()
        self._nav_active = False
        self._arm_active = False
        self._state = self.FAILED
        self._command_all_off()
        self._publish_status(self._last_error)
        self.get_logger().error(f'[FIELD_ROUTE][FAILED] {self._last_error}')

    def _publish_status(self, text):
        message = MissionStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._map_frame
        message.mission_id = self._mission_id
        message.state_text = str(text)
        message.current_index = min(self._index, len(self._steps))
        message.total_targets = len(self._steps)
        message.completed_targets = self._completed_inspects
        message.skipped_targets = 0
        message.last_error = self._last_error
        message.nav_goal_active = self._nav_active
        message.arm_goal_active = self._arm_active
        step = self._step
        message.current_tree_id = (
            step.tree_id if self._state == self.ARM_SPRAYING and step and step.tree_id else '')
        state_map = {
            self.STARTING: MissionStatus.WAITING_FOR_TASKS,
            self.NAVIGATING: MissionStatus.NAVIGATING,
            self.VERIFYING_INSPECT_STOP: MissionStatus.VERIFYING_STOP,
            self.ARM_SPRAYING: MissionStatus.ARM_SPRAYING,
            self.VERIFYING_FINISH_STOP: MissionStatus.VERIFYING_STOP,
            self.COMPLETED: MissionStatus.MISSION_COMPLETED,
            self.FAILED: MissionStatus.FAILED,
        }
        message.state = state_map[self._state]
        self._status_pub.publish(message)

    def destroy_node(self):
        if self._state not in (self.COMPLETED, self.FAILED):
            self._command_all_off()
        return super().destroy_node()


def main():
    rclpy.init()
    node = FieldRouteManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warning('[FIELD_ROUTE] interrupted; commanding both relays off')
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
