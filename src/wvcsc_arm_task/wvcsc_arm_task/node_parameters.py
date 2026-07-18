"""Shared ROS parameter construction for Alicia-M motion nodes."""

from rclpy.callback_groups import ReentrantCallbackGroup

from .alicia_moveit import AliciaMoveIt


def _parameter(node, name, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


def create_alicia_moveit(node, state):
    callback_group = ReentrantCallbackGroup()
    adapter = AliciaMoveIt(
        node=node,
        base_frame=str(_parameter(node, 'base_frame', 'alicia_base_link')),
        group_name=str(_parameter(node, 'group_name', 'arm')),
        tool_link=str(_parameter(node, 'tool_link', 'tool0')),
        velocity_scaling=float(_parameter(node, 'velocity_scaling', 0.1)),
        acceleration_scaling=float(
            _parameter(node, 'acceleration_scaling', 0.1)),
        retime_service_name=str(_parameter(
            node, 'retime_service_name', '/retime_trajectory')),
        retime_timeout=float(_parameter(node, 'retime_timeout', 5.0)),
        execution_timeout=float(_parameter(node, 'execution_timeout', 60.0)),
        planning_time=float(_parameter(node, 'planning_time', 2.0)),
        gripper_action=str(_parameter(
            node, 'gripper_action', '/gripper_controller/gripper_cmd')),
        gripper_open_position=float(_parameter(
            node, 'gripper_open_position', 0.0)),
        gripper_closed_position=float(_parameter(
            node, 'gripper_closed_position', -0.05)),
        gripper_max_effort=float(_parameter(node, 'gripper_max_effort', 5.0)),
        callback_group=callback_group,
        state=state,
    )
    return adapter, callback_group
