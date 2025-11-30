# integration test using launch_testing
import os
import time
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions


def generate_test_description():
    # Launch the talker node (uses installed entrypoint)
    talker = launch_ros.actions.Node(
        package='talker_listener',
        executable='talker',
        name='minimal_publisher',
        output='screen',
        emulate_tty=True
    )

    return (
        launch.LaunchDescription([
            talker,
            # Tell launch_testing to run the test after launch
            launch_testing.actions.ReadyToTest()
        ]),
        {
            'talker': talker
        }
    )


class SubscriberTestNode(Node):
    def __init__(self):
        super().__init__('integration_test_node')
        self.received = []
        self.sub = self.create_subscription(String, 'chatter', self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f'integration test received: "{msg.data}"')
        self.received.append(msg.data)


@pytest.mark.launch_test
def test_talker_parameter_change(launch_service, proc_info, proc_output, talker):
    # Start rclpy for this test process
    rclpy.init()
    test_node = SubscriberTestNode()

    try:
        # wait for a message from the talker (default prefix)
        timeout = time.time() + 5.0
        while time.time() < timeout and len(test_node.received) == 0:
            rclpy.spin_once(test_node, timeout_sec=0.1)

        assert len(test_node.received) > 0, "No message received from talker within timeout"
        first = test_node.received[-1]
        # default prefix is 'hello SSK'
        assert first.startswith('hello SSK'), f'Unexpected initial prefix: {first}'

        # now set parameter using the ROS2 Parameter API (client within test node)
        param_client = test_node.create_client(
            rclpy.qos.QoSProfile()) if False else None  # placeholder to avoid unused import

        # Use the parameter API on the node itself
        # Note: set_parameters is local on nodes; use service via rclpy API
        from rcl_interfaces.msg import SetParametersResult
        # Use Node.set_parameters to emulate ros2 param set (works for local node only via rclpy)
        # For remote nodes, invoking ROS2 parameter services would be needed; instead we'll call system command as a robust approach:
        import subprocess, shlex
        # set the parameter via the CLI (works in CI)
        cmd = 'ros2 param set /minimal_publisher message_prefix "greetings_from_test"'
        subprocess.run(shlex.split(cmd), check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # wait for next message with new prefix
        timeout = time.time() + 5.0
        before_count = len(test_node.received)
        while time.time() < timeout and len(test_node.received) == before_count:
            rclpy.spin_once(test_node, timeout_sec=0.1)

        assert len(test_node.received) > before_count, "No message received after parameter change"
        new = test_node.received[-1]
        assert new.startswith('greetings_from_test'), f'Parameter change not observed in messages: {new}'

    finally:
        test_node.destroy_node()
        rclpy.shutdown()
