"""Ch 8: vacuum ejector + vacuum switch: a response delay, then a signal that is False if nothing sealed."""
import time, rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from cell_interfaces.srv import SetSuction


class MockGripper(Node):
    """Vacuum ejector + vacuum switch: a response delay, then a signal that is False if nothing sealed."""

    def __init__(self):
        super().__init__('mock_gripper')
        self.declare_parameter('vacuum_delay_s', 0.2)
        cb = ReentrantCallbackGroup()
        self.world = self.create_client(SetBool, 'world/grip', callback_group=cb)
        self.create_service(SetSuction, 'gripper/set_suction', self.handle, callback_group=cb)
        self.vacuum = False
        self.pub = self.create_publisher(Bool, 'gripper/vacuum', 10)
        self.create_timer(0.1, lambda: self.pub.publish(Bool(data=self.vacuum)))

    def handle(self, req, res):
        time.sleep(self.get_parameter('vacuum_delay_s').value)     # ejector response time
        r = self.world.call(SetBool.Request(data=req.on))         # blocking call: needs the multithreaded executor
        self.vacuum = bool(req.on and r.success)
        res.success, res.message = r.success, r.message
        return res


def main():
    rclpy.init(); n = MockGripper()
    ex = MultiThreadedExecutor(); ex.add_node(n); ex.spin()
