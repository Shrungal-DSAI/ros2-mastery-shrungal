"""
Integration test (process-level) for talker_listener.

This test launches the talker as a subprocess, subscribes to /chatter using rclpy,
changes the talker's runtime parameter via the ros2 CLI, and verifies the message
prefix changes accordingly.
"""

import time
import subprocess
import shutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberTestNode(Node):
    def __init__(self):
        super().__init__('integration_process_test_node')
        self.received = []
        self.sub = self.create_subscription(String, 'chatter', self.cb, 10)
        self.sub  # prevent unused variable warning

    def cb(self, msg):
        self.get_logger().info(f'integration test received: "{msg.data}"')
        self.received.append(msg.data)


def find_ros2_cmd():
    """Return path to the ros2 CLI."""
    cmd = shutil.which('ros2')
    return cmd or '/usr/bin/ros2'


def set_parameter_with_retry(ros2_cmd, param_value, retries=2, delay=0.8):
    """Run ros2 param set and retry a few times if it fails."""
    for attempt in range(retries):
        proc = subprocess.run(
            [ros2_cmd, 'param', 'set', '/minimal_publisher', 'message_prefix', param_value],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        if proc.returncode == 0:
            return True, proc
        time.sleep(delay)
    return False, proc


def wait_for_new_message(node, before_count, timeout=12.0):
    """Wait for node.received to increase beyond before_count within timeout."""
    deadline = time.time() + timeout
    while time.time() < deadline and len(node.received) == before_count:
        rclpy.spin_once(node, timeout_sec=0.1)
    return len(node.received) > before_count


def get_last_message(node):
    return node.received[-1] if node.received else ''


def start_talker_subprocess(ros2_cmd):
    return subprocess.Popen(
        [ros2_cmd, 'run', 'talker_listener', 'talker'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )


def stop_proc(proc):
    try:
        proc.terminate()
        proc.wait(timeout=3)
    except Exception:
        proc.kill()
        proc.wait(timeout=3)


def test_talker_parameter_change_process():
    """Verify talker publishes default prefix and updates after parameter change."""
    ros2 = find_ros2_cmd()

    # Start talker as a subprocess using ros2 run
    p = start_talker_subprocess(ros2)

    # allow node to start
    time.sleep(1.0)

    rclpy.init()
    node = SubscriberTestNode()

    try:
        # wait for first message
        deadline = time.time() + 12.0
        while time.time() < deadline and len(node.received) == 0:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert len(node.received) > 0, "No message received from talker within timeout"
        first = get_last_message(node)
        assert first.startswith('hello SSK'), f'Unexpected initial prefix: {first}'

        # change parameter via ros2 CLI (with a couple retries)
        ok, proc = set_parameter_with_retry(ros2, 'greetings_from_test', retries=3, delay=0.5)
        assert ok, f"ros2 param set failed after retries: {proc.stderr.decode().strip()}"

        # short pause to let the node process the parameter change
        time.sleep(0.8)

        # wait for next message with new prefix (longer timeout)
        before = len(node.received)
        got = wait_for_new_message(node, before_count=before, timeout=12.0)

        if not got:
            # extra safety: try setting parameter once more then wait a bit longer
            set_parameter_with_retry(ros2, 'greetings_from_test', retries=2, delay=0.3)
            got = wait_for_new_message(node, before_count=before, timeout=8.0)

        assert got, "No message received after parameter change"

        new = get_last_message(node)
        assert new.startswith('greetings_from_test'), \
            f'Parameter change not observed in messages: {new}'

    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop_proc(p)
