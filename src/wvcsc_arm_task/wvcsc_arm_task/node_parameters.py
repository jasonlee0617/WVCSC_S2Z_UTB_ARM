# node_parameters.py
"""
Alicia-M 运动节点的共享 ROS 参数构建工厂。

该模块负责将 ROS2 节点的参数系统与 `AliciaMoveIt` 运动适配器解耦。
它将 `rclpy.node.Node` 的声明参数（declare_parameter）和获取参数（get_parameter）
逻辑集中于此，避免在 `spray_task` 和 `motion_control` 两个主节点中重复写相同的
参数加载样板代码。

使用可重入回调组 (ReentrantCallbackGroup) 实例化适配器，以支持多线程并发访问。
"""

from rclpy.callback_groups import ReentrantCallbackGroup

from .alicia_moveit import AliciaMoveIt


def _parameter(node, name, default):
    """
    统一的参数声明与获取辅助函数（最佳实践）。

    在 ROS2 (rclpy) 中，节点使用参数前必须先进行声明。
    此函数会检查节点是否已声明该参数；如果没有，则使用默认值进行声明。
    无论哪种情况，最后都返回当前的有效参数值。

    这种写法确保了 `create_alicia_moveit` 可以在节点生命周期的任何时刻
    调用，而不会因为参数未声明而抛出异常。

    Args:
        node (rclpy.node.Node): 当前 ROS2 节点实例。
        name (str): 参数名称。
        default: 参数的默认值。

    Returns:
        Any: 参数在 ROS2 参数服务器中的实际值。
    """
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


def create_alicia_moveit(node, state):
    """
    基于节点参数创建并返回配置好的 AliciaMoveIt 适配器实例。

    此函数完成以下任务：
    1. 从 ROS2 参数服务器读取所有与机械臂运动相关的配置项。
    2. 实例化一个 `ReentrantCallbackGroup`（可重入回调组）。
    3. 将读取到的参数和回调组注入到 `AliciaMoveIt` 的构造函数中。
    4. 将项目特定的状态机 (`MotionControlState`) 传递给适配器，实现运动互锁。

    Args:
        node (rclpy.node.Node): 需要操作机械臂的 ROS2 节点实例。
        state (MotionControlState): 线程安全的运动状态锁实例。

    Returns:
        tuple: (AliciaMoveIt, ReentrantCallbackGroup)
            - 第一个元素是配置好的运动适配器。
            - 第二个元素是创建时使用的回调组，便于外部节点在创建订阅/客户端时复用。
    """
    # 实例化可重入回调组 (ReentrantCallbackGroup)
    # 优点：允许不同回调（如 Action 服务器的执行线程、订阅回调）在同一节点内
    #      交错执行，不会因为等待一个执行中的线程而阻塞其他并发的网络请求。
    # 这对于 `spray_task` 这种既要进行长时运动规划，又要监听急停/取消操作
    # 的复杂 Action Server 是必须的配置。
    callback_group = ReentrantCallbackGroup()

    # 逐项声明并读取参数。传递的默认值必须与 `alicia_moveit.py` 中预期的严格一致。
    adapter = AliciaMoveIt(
        node=node,
        base_frame=str(_parameter(node, 'base_frame', 'alicia_base_link')),
        group_name=str(_parameter(node, 'group_name', 'arm')),
        tool_link=str(_parameter(node, 'tool_link', 'tool0')),
        # 速度与加速度缩放 (0~1)
        velocity_scaling=float(_parameter(node, 'velocity_scaling', 0.1)),
        acceleration_scaling=float(
            _parameter(node, 'acceleration_scaling', 0.1)),
        # 轨迹重定时服务配置
        retime_service_name=str(_parameter(
            node, 'retime_service_name', '/retime_trajectory')),
        retime_timeout=float(_parameter(node, 'retime_timeout', 5.0)),
        # 底层运动执行全局超时（如果50秒轨迹还没走完，将被强制终止）
        execution_timeout=float(_parameter(node, 'execution_timeout', 60.0)),
        planning_time=float(_parameter(node, 'planning_time', 2.0)),
        planning_pipeline_id=str(_parameter(
            node, 'planning_pipeline_id', 'ompl')),
        planner_id=str(_parameter(node, 'planner_id', 'RRTConnectFast')),
        # 夹爪配置
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