# relay_controller.py
# ============================================================================
# 任务层继电器命令与广域喷洒软件确认状态
# ============================================================================

from std_msgs.msg import Bool
from wvcsc_interfaces.srv import SetRelay


class RelayController:
    """Own relay requests and publish only successfully confirmed wide-spray state."""

    def __init__(
            self, node, *, service_name, wide_channel, arm_channel,
            require_service, status_qos, required_failure_callback):
        self._node = node
        self._logger = node.get_logger()
        self._client = node.create_client(SetRelay, str(service_name))
        self._wide_channel = int(wide_channel)
        self._arm_channel = int(arm_channel)
        self._require_service = bool(require_service)
        self._required_failure_callback = required_failure_callback
        self._wide_active_pub = node.create_publisher(
            Bool, '/spray/wide_active', status_qos)
        self._wide_enabled = False
        self._failure_latched = False
        self._publish_wide_active()

    def service_is_ready(self):
        return self._client.service_is_ready()

    def reset_failure_latch(self):
        self._failure_latched = False

    def command(
            self, channel, enabled, duration, continuation, context,
            *, critical=True):
        """Send one non-blocking SetRelay request and then run continuation on success."""
        channel = int(channel)
        enabled = bool(enabled)
        duration = float(duration)
        if not self._client.service_is_ready():
            if self._command_failed(
                    channel, enabled, context, 'service unavailable', critical):
                self._continue(continuation)
            return
        request = SetRelay.Request()
        request.channel = channel
        request.enabled = enabled
        request.duration = duration
        try:
            future = self._client.call_async(request)
        except Exception as error:
            if self._command_failed(
                    channel, enabled, context, str(error), critical):
                self._continue(continuation)
            return

        def done(result_future):
            try:
                response = result_future.result()
                if response is None or not response.success:
                    detail = '' if response is None else response.message
                    proceed = self._command_failed(
                        channel, enabled, context,
                        detail or 'request rejected', critical)
                else:
                    if channel == self._wide_channel:
                        self._wide_enabled = enabled
                        self._publish_wide_active()
                    state = 'ON' if enabled else 'OFF'
                    self._logger.info(
                        f'[RELAY] channel={channel} state={state} '
                        f'duration={duration:.2f}s context={context}')
                    proceed = True
            except Exception as error:
                proceed = self._command_failed(
                    channel, enabled, context, str(error), critical)
            if proceed:
                self._continue(continuation)

        future.add_done_callback(done)

    def command_all_off(self):
        self.command(
            self._wide_channel, False, 0.0, None,
            'mission shutdown: disable wide spray', critical=False)
        self.command(
            self._arm_channel, False, 0.0, None,
            'mission shutdown: disable arm spray', critical=False)

    def _command_failed(self, channel, enabled, context, detail, critical):
        message = (
            f'channel={channel} enabled={enabled} context={context}: {detail}')
        if critical and self._require_service:
            if not self._failure_latched:
                self._failure_latched = True
                self._required_failure_callback(message)
            return False
        self._logger.warning(f'[MISSION][WARN][RELAY] {message}; continuing')
        return True

    def _publish_wide_active(self):
        self._wide_active_pub.publish(Bool(data=self._wide_enabled))

    @staticmethod
    def _continue(continuation):
        if continuation is not None:
            continuation()
