import time
from action_msgs.msg import GoalStatus
from wvcsc_interfaces.action import AlignTarget, Spray

class DownstreamActionMixin:
    def _align_target(self, mission_id, tree_id, target_id, cancel_requested):
        goal = AlignTarget.Goal()
        goal.mission_id = mission_id
        goal.tree_id = tree_id
        goal.target_id = target_id
        goal.timeout = self._vision_timeout
        wrapped, canceled, error = self._run_downstream_action(
            self._vision_client, goal, self._vision_timeout + self._downstream_margin,
            cancel_requested, 'vision alignment')
        if wrapped is None:
            code = (AlignTarget.Result.CANCELED if canceled
                    else AlignTarget.Result.TIMEOUT)
            return False, canceled, code, error
        result = wrapped.result
        ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.success
        canceled = (
            wrapped.status == GoalStatus.STATUS_CANCELED or
            result.error_code == AlignTarget.Result.CANCELED)
        return (
            ok, canceled, int(result.error_code),
            result.message or f'vision status={wrapped.status}')

    def _spray_target(self, mission_id, tree_id, duration, cancel_requested):
        goal = Spray.Goal()
        goal.mission_id = mission_id
        goal.tree_id = tree_id
        goal.duration = duration
        goal.mode = 'continuous'
        wrapped, canceled, error = self._run_downstream_action(
            self._spray_client, goal, duration + self._downstream_margin,
            cancel_requested, 'spray actuator')
        if wrapped is None:
            return False, canceled, error
        result = wrapped.result
        ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.success
        return ok, False, result.message or f'spray status={wrapped.status}'

    def _run_downstream_action(self, client, goal, result_timeout, cancel_requested, label):
        """等待下游 Action 的接受与最终结果，并在上游取消/超时时传播取消。"""
        deadline = time.monotonic() + self._downstream_server_timeout
        while not client.server_is_ready():
            if self._aborted(cancel_requested):
                return None, True, f'{label} canceled'
            if time.monotonic() >= deadline:
                return None, False, f'{label} server is unavailable'
            time.sleep(0.02)
        response_future = client.send_goal_async(goal)
        response, canceled = self._wait_future(
            response_future, self._downstream_server_timeout, cancel_requested)
        if response is None:
            if canceled:
                response_future.add_done_callback(self._cancel_late_goal)
            return None, canceled, f'{label} goal response timed out or canceled'
        if not response.accepted:
            return None, False, f'{label} goal was rejected'
        result_future = response.get_result_async()
        wrapped, canceled = self._wait_future(
            result_future, result_timeout, cancel_requested, cancel_handle=response)
        if wrapped is None:
            return None, canceled, f'{label} result timed out or canceled'
        return wrapped, False, ''

    def _wait_future(self, future, timeout, cancel_requested, cancel_handle=None):
        deadline = time.monotonic() + timeout
        while not future.done():
            if self._aborted(cancel_requested) or time.monotonic() >= deadline:
                if cancel_handle is not None:
                    self._cancel_downstream_and_wait(cancel_handle, future)
                return None, self._aborted(cancel_requested)
            time.sleep(0.02)
        try:
            return future.result(), False
        except Exception:
            return None, False

    def _cancel_downstream_and_wait(self, goal_handle, result_future):
        deadline = time.monotonic() + self._downstream_server_timeout
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception:
            return False
        for future in (cancel_future, result_future):
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
        return result_future.done()

    @staticmethod
    def _cancel_late_goal(future):
        try:
            handle = future.result()
        except Exception:
            return
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()
