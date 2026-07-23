#!/usr/bin/env python3
"""Move Alicia-M once to a manually configured calibration start pose."""

import math
import threading
import time

import rclpy
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from wvcsc_arm_task.motion_state import MotionControlState
from wvcsc_arm_task.node_parameters import create_alicia_moveit

from .alicia_sample_geometry import tool_orientation_toward_marker


class InitialCalibrationPose(Node):
    """Run one collision-aware IK precheck, then plan and execute one pose."""

    def __init__(self):
        super().__init__('initial_calibration_pose')
        self._declare_parameters()
        self._group = ReentrantCallbackGroup()
        self._state = MotionControlState()
        self._arm, _ = create_alicia_moveit(self, self._state)
        self._compute_ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self._group)
        self._plan_client = self.create_client(
            GetMotionPlan, '/plan_kinematic_path', callback_group=self._group)
        self._lock = threading.Lock()
        self._joint_positions = None
        self._started = False
        self.create_subscription(
            JointState, str(self.get_parameter('joint_state_topic').value),
            self._on_joint_state, 10, callback_group=self._group)
        self._start_timer = self.create_timer(
            0.2, self._start_once, callback_group=self._group)
        self.get_logger().info(
            '[INITIAL_POSE] ready; waiting for joint states and MoveIt')

    def _declare_parameters(self):
        defaults = {
            'base_frame': 'alicia_base_link',
            'tool_link': 'tool0',
            'group_name': 'arm',
            'tool_position_base_m': [0.0, 0.25, 0.20],
            'marker_position_base_m': [0.0, 0.25, 0.0],
            'joint_state_topic': '/joint_states',
            'position_tolerance_m': 0.003,
            'orientation_tolerance_rad': 0.02,
            'startup_timeout_sec': 20.0,
            'velocity_scaling': 0.10,
            'acceleration_scaling': 0.10,
            'planning_time': 5.0,
            'execution_timeout': 90.0,
            'planning_pipeline_id': 'ompl',
            'planner_id': 'RRTConnectFast',
        }
        for name, default in defaults.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
        self._validate_parameters()

    def _validate_parameters(self):
        for name in ('tool_position_base_m', 'marker_position_base_m'):
            values = tuple(float(value) for value in self.get_parameter(name).value)
            if len(values) != 3 or not all(math.isfinite(value) for value in values):
                raise ValueError(f'{name} must contain three finite values')
        tool = tuple(float(value) for value in self.get_parameter(
            'tool_position_base_m').value)
        marker = tuple(float(value) for value in self.get_parameter(
            'marker_position_base_m').value)
        tool_orientation_toward_marker(tool, marker)
        for name in (
                'position_tolerance_m', 'orientation_tolerance_rad',
                'startup_timeout_sec', 'planning_time', 'execution_timeout'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('velocity_scaling', 'acceleration_scaling'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or not 0.0 < value <= 1.0:
                raise ValueError(f'{name} must be finite and in (0, 1]')

    def _on_joint_state(self, message):
        values = dict(zip(message.name, message.position))
        try:
            positions = tuple(
                float(values[name]) for name in self._arm.JOINT_NAMES)
        except (KeyError, TypeError, ValueError):
            return
        if not all(math.isfinite(value) for value in positions):
            return
        with self._lock:
            self._joint_positions = positions

    def _start_once(self):
        if self._started:
            return
        self._started = True
        self._start_timer.cancel()
        threading.Thread(
            target=self._run_guarded, name='initial-calibration-pose',
            daemon=True).start()

    def _run_guarded(self):
        try:
            self._run()
        except Exception as error:
            self.get_logger().error(f'[INITIAL_POSE][FAILED] {error}')
        finally:
            rclpy.shutdown()

    def _run(self):
        timeout = float(self.get_parameter('startup_timeout_sec').value)
        self._wait_for_moveit(timeout)
        start_joints = self._wait_for_joint_state(timeout)
        tool_position = tuple(float(value) for value in self.get_parameter(
            'tool_position_base_m').value)
        marker_position = tuple(float(value) for value in self.get_parameter(
            'marker_position_base_m').value)
        tool_quaternion = tool_orientation_toward_marker(
            tool_position, marker_position)

        ik_solution = self._arm.compute_ik(
            tool_position, tool_quaternion, start_joints, timeout=timeout)
        if ik_solution is None:
            raise RuntimeError('collision-aware IK rejected the configured tool0 pose')

        trajectory = self._arm.plan_pose(
            tool_position, tool_quaternion,
            frame_id=str(self.get_parameter('base_frame').value),
            tolerance_position=float(self.get_parameter('position_tolerance_m').value),
            tolerance_orientation=float(
                self.get_parameter('orientation_tolerance_rad').value))
        if trajectory is None:
            raise RuntimeError('MoveIt could not plan the configured tool0 pose')
        if not self._arm.execute_trajectory(trajectory):
            raise RuntimeError('MoveIt execution failed')
        self.get_logger().info(
            '[INITIAL_POSE][SUCCESS] '
            f'tool0={tool_position} marker={marker_position} '
            f'quaternion_xyzw={tool_quaternion}')

    def _wait_for_moveit(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            if (self._compute_ik_client.service_is_ready()
                    and self._plan_client.service_is_ready()):
                return
            time.sleep(0.05)
        raise RuntimeError('MoveIt IK or planning service unavailable')

    def _wait_for_joint_state(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            with self._lock:
                if self._joint_positions is not None:
                    return self._joint_positions
            time.sleep(0.05)
        raise RuntimeError('joint state unavailable')


def main(args=None):
    rclpy.init(args=args)
    node = InitialCalibrationPose()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
