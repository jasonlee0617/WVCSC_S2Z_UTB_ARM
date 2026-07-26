#!/usr/bin/env python3
"""Gazebo-only implementation of the physical two-channel relay contract."""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from wvcsc_interfaces.srv import SetRelay


class SimRelay(Node):
    """Expose ``/relay/set`` for the wide (1) and nozzle (2) simulators.

    The service contract intentionally matches the real relay controller: a
    positive duration is a one-shot pulse and zero keeps a channel on until an
    explicit off request.  Latched state topics let Gazebo GUI and Qt attach
    after the mission manager without losing the current state.
    """

    _CHANNELS = (1, 2)

    def __init__(self):
        super().__init__('wvcsc_sim_relay')
        self.declare_parameter('service_name', '/relay/set')
        self.declare_parameter('wide_channel', 1)
        self.declare_parameter('arm_channel', 2)
        self.declare_parameter('motion_locked_topic', '/motion_control/locked')
        self._states = {channel: False for channel in self._CHANNELS}
        self._deadlines = {channel: None for channel in self._CHANNELS}
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publishers = {
            1: self.create_publisher(Bool, '/relay/sim/channel_1_active', latched),
            2: self.create_publisher(Bool, '/relay/sim/channel_2_active', latched),
        }
        self.create_service(
            SetRelay,
            str(self.get_parameter('service_name').value),
            self._set_relay,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('motion_locked_topic').value),
            self._on_motion_locked,
            latched,
        )
        self._timer = self.create_timer(0.02, self._expire_pulses)
        for channel in self._CHANNELS:
            self._publish(channel)

    def _set_relay(self, request, response):
        channel = int(request.channel)
        duration = float(request.duration)
        if channel not in self._CHANNELS:
            response.success = False
            response.message = 'only simulated relay channels 1 and 2 are available'
            return response
        if not math.isfinite(duration) or duration < 0.0:
            response.success = False
            response.message = 'duration must be finite and non-negative'
            return response
        self._set_channel(
            channel,
            bool(request.enabled),
            None if duration == 0.0 else time.monotonic() + duration,
        )
        response.success = True
        response.message = (
            f'channel {channel} ' + ('enabled' if request.enabled else 'disabled'))
        return response

    def _set_channel(self, channel, enabled, deadline=None):
        self._states[channel] = bool(enabled)
        self._deadlines[channel] = deadline if enabled else None
        self._publish(channel)

    def _publish(self, channel):
        message = Bool()
        message.data = self._states[channel]
        self._publishers[channel].publish(message)

    def _expire_pulses(self):
        now = time.monotonic()
        for channel, deadline in tuple(self._deadlines.items()):
            if deadline is not None and now >= deadline:
                self._set_channel(channel, False)

    def _on_motion_locked(self, message):
        if message.data:
            self.force_off()

    def force_off(self):
        for channel in self._CHANNELS:
            if self._states[channel] or self._deadlines[channel] is not None:
                self._set_channel(channel, False)

    def destroy_node(self):
        self.force_off()
        return super().destroy_node()


def main():
    rclpy.init()
    node = SimRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launch SIGINT can invalidate the ROS context before this handler
        # runs.  Publishing after that point used to turn a normal shutdown
        # into an exit-code-1 failure.
        if rclpy.ok():
            node.force_off()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
