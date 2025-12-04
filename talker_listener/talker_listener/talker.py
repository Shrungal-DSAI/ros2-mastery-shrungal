#!/usr/bin/env python3
"""Talker node that publishes and also serves a get_count service.

- Publishes std_msgs/String to /chatter every second with a prefix + counter.
- Exposes a Trigger service '/get_count' that returns the live "prefix count" string.
- Declares a ROS parameter 'message_prefix' (so ros2 param set works).
"""

from __future__ import annotations
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from std_srvs.srv import Trigger


class TalkerWithService(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # callback group so service and timer can run concurrently in same thread if needed
        self._cb_group = ReentrantCallbackGroup()

        # declare parameter with default
        self.declare_parameter('message_prefix', 'hello SSK')
        self.prefix = self.get_parameter('message_prefix').get_parameter_value().string_value

        # internal counter
        self.count = 0

        # publisher
        self.pub = self.create_publisher(String, 'chatter', 10)

        # timer (1 Hz)
        self.timer = self.create_timer(1.0, self._on_timer, callback_group=self._cb_group)

        # service returning current prefix + count
        self.srv = self.create_service(Trigger, 'get_count', self._on_get_count, callback_group=self._cb_group)

        # watch for parameter changes
        self.add_on_set_parameters_callback(self._on_parameters_changed)

        self.get_logger().info(f'TalkerWithService started with prefix="{self.prefix}"')

    def _on_timer(self):
        msg = String()
        msg.data = f'{self.prefix} {self.count}'
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

    def _on_get_count(self, request, response):
        # synchronous service: return the current prefix + count
        response.success = True
        # return the last published index (count-1) to reflect the most recent message,
        # or count if you prefer the next index. We'll return last published:
        last_index = max(0, self.count - 1)
        response.message = f'{self.prefix} {last_index}'
        self.get_logger().info(f'Handled get_count -> "{response.message}"')
        return response

    def _on_parameters_changed(self, params):
        # rclpy sets params one batch at a time
        for p in params:
            if p.name == 'message_prefix' and p.type_ == Parameter.Type.STRING:
                self.get_logger().info(f'Parameter message_prefix changing: {self.prefix!r} -> {p.value!r}')
                self.prefix = p.value
        # return response allowing change
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True, reason='ok')


def main(args=None):
    rclpy.init(args=args)
    node = TalkerWithService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
