# action_flow.py
"""
下游 Action 通讯混入模块 (Mixin)。

本模块为长时任务节点提供标准化的 ROS2 Action 客户端调用封装。
任务核心逻辑（如 `wvcsc_spray_task`）通过本模块与视觉伺服 Action 和喷洒 Action
进行交互。封装实现考虑了：
- 服务端就绪等待与健康检查
- 上游任务取消（Cancel Request）的安全传播机制
- 结果超时与异常捕获（Fail-close 策略）
- 下游 Action 超时后的自动取消，防止系统“僵尸”状态的残留。
"""

import time
from action_msgs.msg import GoalStatus
from wvcsc_interfaces.action import AlignTarget, Spray


class DownstreamActionMixin:
    """
    下游 Action 通讯混入类。
    包含通用的 Action 客户端交互逻辑，供喷洒任务基类继承。
    """

    def _align_target(
            self, mission_id, tree_id, target_id, nozzle_aim,
            cancel_requested):
        """
        调用下游的视觉伺服 Action (`AlignTarget`) 进行 IBVS 对准。
        
        负责锁定病果 ID，发送对准目标，并在指定的超时及取消机制下获取执行结果。

        Args:
            mission_id (str): 当前任务的任务 ID。
            tree_id (str): 当前正在处理的病树 ID。
            target_id (str): 视觉伺服节点锁定的病果逻辑目标 ID。
            cancel_requested (callable): 一个返回 bool 的 lambda/函数，用于检查上层任务是否已请求取消。

        Returns:
            tuple: (ok, canceled, error_code, error_message)
                - ok (bool): 如果对准成功并且伺服状态码为成功，则为 True。
                - canceled (bool): 如果任务已被取消，则为 True。
                - error_code (int): `AlignTarget.Result` 中的具体错误代码。
                - error_message (str): 具体的结果或错误信息。
        """
        goal = AlignTarget.Goal()
        goal.mission_id = mission_id
        goal.tree_id = tree_id
        goal.target_id = target_id
        goal.timeout = self._vision_timeout  # 从节点参数获取的视觉伺服超时时间
        desired_u, desired_v, image_width, image_height, working_range = nozzle_aim
        goal.working_range_m = float(working_range)
        goal.desired_u_px = float(desired_u)
        goal.desired_v_px = float(desired_v)
        goal.image_width = int(image_width)
        goal.image_height = int(image_height)

        # 发送 Action 目标，设置的结果超时时间 = 伺服超时 + 下游缓冲余量
        # 这种设计确保即便视觉伺服节点处于高负载，通信层也不会过早判定超时
        wrapped, canceled, error = self._run_downstream_action(
            self._vision_client,
            goal,
            self._vision_timeout + self._downstream_margin,
            cancel_requested,
            'vision alignment'
        )

        if wrapped is None:
            # Action 目标未被接受，或出现网络/超时错误
            code = (AlignTarget.Result.CANCELED if canceled else AlignTarget.Result.TIMEOUT)
            return False, canceled, code, error

        result = wrapped.result
        # 成功的定义：ROS 标准状态是 SUCCEEDED，且自定义 Action 内部逻辑也返回 success
        ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.success
        canceled = (
            wrapped.status == GoalStatus.STATUS_CANCELED or
            result.error_code == AlignTarget.Result.CANCELED
        )
        return (
            ok,
            canceled,
            int(result.error_code),
            result.message or f'vision status={wrapped.status}'
        )

    def _spray_target(self, mission_id, tree_id, duration, cancel_requested):
        """
        调用下游的喷洒 Action (`Spray`) 执行喷洒操作。

        Args:
            mission_id (str): 当前任务 ID。
            tree_id (str): 当前喷洒目标树 ID。
            duration (float): 设定的喷洒持续时间（秒）。
            cancel_requested (callable): 上层取消请求检查函数。

        Returns:
            tuple: (ok, canceled, error_message)
                - ok (bool): 喷洒执行成功。
                - canceled (bool): 任务被用户或上级取消。
                - error_message (str): 具体的结果或错误信息。
        """
        goal = Spray.Goal()
        goal.mission_id = mission_id
        goal.tree_id = tree_id
        goal.duration = duration
        goal.mode = 'continuous'

        # 喷洒任务通常需要在指定的喷洒时长后，再加上一些通讯安全余量
        wrapped, canceled, error = self._run_downstream_action(
            self._spray_client,
            goal,
            duration + self._downstream_margin,
            cancel_requested,
            'spray actuator'
        )

        if wrapped is None:
            return False, canceled, error

        result = wrapped.result
        ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.success
        return ok, False, result.message or f'spray status={wrapped.status}'

    def _run_downstream_action(self, client, goal, result_timeout, cancel_requested, label):
        """
        下游 Action 发送的核心执行与状态机守卫函数。

        **执行逻辑**：
        1. 等待下游 Action 服务器的就绪状态（带超时）。
        2. 发送目标请求 (send_goal_async)。
        3. 等待服务器接受或拒绝目标（带超时）。
        4. 目标被接受后，等待目标执行结果 (get_result_async)。
        5. 支持安全取消：在任意等待循环中，如果 `cancel_requested()` 返回 True，
           则立刻请求下游 Action 取消当前目标。

        Args:
            client (rclpy.action.ActionClient): 目标动作客户端。
            goal: 目标消息实体。
            result_timeout (float): 请求结果返回的最大超时时间。
            cancel_requested (callable): 检查任务取消的标志函数。
            label (str): 当前 Action 的语义标识，用于日志区分。

        Returns:
            tuple: (wrapped, canceled, error_message)
                - wrapped (rclpy.action.client.GoalResponse): 目标的响应包装器。失败时为 None。
                - canceled (bool): 任务是否因为外部取消而被中断。
                - error_message (str): 超时或拒绝的错误信息。
        """
        # 1. 等待 Action 服务器就绪
        deadline = time.monotonic() + self._downstream_server_timeout
        while not client.server_is_ready():
            # 在上层任务被取消的情况下，不再继续等待服务端
            if self._aborted(cancel_requested):
                return None, True, f'{label} canceled'
            if time.monotonic() >= deadline:
                return None, False, f'{label} server is unavailable'
            time.sleep(0.02)

        # 2. 异步发送 Action 目标
        response_future = client.send_goal_async(goal)
        # 等待服务器响应（接受或拒绝），超时时间使用下游连接超时
        response, canceled = self._wait_future(
            response_future, self._downstream_server_timeout, cancel_requested
        )

        if response is None:
            # 如果等待超时或任务取消，且目标异步尚未回包，仍需尝试取消目标（防僵尸）
            if canceled:
                response_future.add_done_callback(self._cancel_late_goal)
            return None, canceled, f'{label} goal response timed out or canceled'

        if not response.accepted:
            # 服务器明确拒绝了目标（通常是因为冲突或状态不可用）
            return None, False, f'{label} goal was rejected'

        # 3. 目标已接受，等待执行结果
        result_future = response.get_result_async()
        wrapped, canceled = self._wait_future(
            result_future, result_timeout, cancel_requested, cancel_handle=response
        )

        if wrapped is None:
            return None, canceled, f'{label} result timed out or canceled'

        return wrapped, False, ''

    def _wait_future(self, future, timeout, cancel_requested, cancel_handle=None):
        """
        等待 `rclpy` 异步 Future 完成，且支持外部取消仲裁和超时。

        这是 ROS2 Action 开发中很核心的防死锁方法。与传统 Python 的 future.result()
        不同，此函数通过轮询 `future.done()` 并在每次轮询时检查 `cancel_requested`
        标志，实现了安全的非阻塞等待。

        Args:
            future (rclpy.task.Future): 待等待的异步任务 Future。
            timeout (float): 最多等待的超时秒数。
            cancel_requested (callable): 外部取消检查函数。
            cancel_handle (rclpy.action.client.ClientGoalHandle, optional): 如果目标存在，
                当检测到取消或超时时，传入此句柄以向上游终止残留目标。

        Returns:
            tuple: (result, canceled)
                - result: future 的结果，如果超时或取消则返回 None。
                - canceled (bool): 任务是否因取消而退出。
        """
        deadline = time.monotonic() + timeout
        while not future.done():
            # 主动检查外部是否请求了紧急停止（如 `motion_control` 锁死、人工急停）
            if self._aborted(cancel_requested) or time.monotonic() >= deadline:
                if cancel_handle is not None:
                    # 启动一个清理任务，尝试取消还在运行的下游 Action
                    self._cancel_downstream_and_wait(cancel_handle, future)
                # 返回 `None` 和取消标志，供上层状态机进行异常处理
                return None, self._aborted(cancel_requested)
            # 极短的 sleep，避免死循环吃满 CPU
            time.sleep(0.02)

        try:
            # Future 完成时，直接提取结果
            return future.result(), False
        except Exception:
            # 捕获 Action 通讯时的底层异常（如 RPC 失败）
            return None, False

    def _cancel_downstream_and_wait(self, goal_handle, result_future):
        """
        强制取消当前正在执行的下游 Action，并等待取消完成。

        用于确保在超时或外部取消时的干净退出。
        在 ROS2 中，终止残留的 Action 极其重要，否则后续任务可能会一直处于 `BUSY` 状态。

        Args:
            goal_handle (rclpy.action.client.ClientGoalHandle): 待取消的目标句柄。
            result_future (rclpy.task.Future): 结果 Future，确保取消后等待其真实完成。

        Returns:
            bool: 取消操作是否成功执行（并完成等待）。
        """
        deadline = time.monotonic() + self._downstream_server_timeout
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception:
            # 捕获底层 API 异常，避免崩坏主线程
            return False

        # 并发等待取消回调（cancel_future）和结果 Future（result_future）
        for future in (cancel_future, result_future):
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
        return result_future.done()

    @staticmethod
    def _cancel_late_goal(future):
        """
        延迟的 Action 目标取消回调（静默取消，不报错）。

        如果在一个已发送的目标还未收到服务端响应时，主任务就被取消了，此函数
        会被调用。它会尝试取消该目标，防止它在后台继续执行而导致状态错乱。
        """
        try:
            handle = future.result()
        except Exception:
            return
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()
