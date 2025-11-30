#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # Declare parameter with default
        self.declare_parameter('message_prefix', 'hello SSK')
        self.msg_prefix = self.get_parameter('message_prefix').get_parameter_value().string_value

        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

        # Register callback for runtime updates
        self.add_on_set_parameters_callback(self.parameter_update_callback)

    def parameter_update_callback(self, params):
        for p in params:
            if p.name == 'message_prefix' and p.type_ == Parameter.Type.STRING:
                self.msg_prefix = p.value
                self.get_logger().info(f'Parameter updated: message_prefix = {self.msg_prefix}')
        return SetParametersResult(successful=True)

    def timer_callback(self):
        msg = String()
        msg.data = f'{self.msg_prefix} {self.i}'
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
