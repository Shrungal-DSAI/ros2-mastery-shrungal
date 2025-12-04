import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

def test_service_reflects_live_count():
    rclpy.init()
    node = Node('test_client_node')
    # wait for service to appear
    client = node.create_client(Trigger, 'get_count')
    assert client.wait_for_service(timeout_sec=10.0), "get_count service not available"

    # call service the first time
    req = Trigger.Request()
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    res1 = fut.result()
    assert res1 is not None and res1.success

    # wait a couple seconds so talker publishes more messages
    time.sleep(2.5)

    fut = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    res2 = fut.result()
    assert res2 is not None and res2.success

    # messages should show an increased index
    msg1 = res1.message
    msg2 = res2.message
    # parse trailing integer, fallback to simple substring check
    try:
        i1 = int(msg1.split()[-1])
        i2 = int(msg2.split()[-1])
        assert i2 >= i1, f'expected later message index >= earlier: {i1} -> {i2}'
    except Exception:
        assert msg1 != msg2, "expected messages to differ over time"

    node.destroy_node()
    rclpy.shutdown()
