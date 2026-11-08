"""Ch 8: vacuum ejector + vacuum switch: a response delay, then a signal that is False if nothing sealed.
Ch 16: fault injection through parameters (fail_every_n, drop_after_s)."""
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
        self.declare_parameter('fail_every_n', 0)          # 0 = never; 4 = every 4th grip finds no vacuum
        self.declare_parameter('drop_after_s', 0.0)        # > 0: the seal fails this long after a grip
        self.grips, self.grip_time = 0, None
        cb = ReentrantCallbackGroup()
        self.world = self.create_client(SetBool, 'world/grip', callback_group=cb)
        self.create_service(SetSuction, 'gripper/set_suction', self.handle, callback_group=cb)
        self.vacuum = False
        self.pub = self.create_publisher(Bool, 'gripper/vacuum', 10)
        self.create_timer(0.1, lambda: self.pub.publish(Bool(data=self.vacuum)))
        self.create_timer(0.1, self.watch_seal, callback_group=cb)

    def handle(self, req, res):
        if req.on:
            self.grips += 1
            n = self.get_parameter('fail_every_n').value
            if n and self.grips % n == 0:
                res.success, res.message = False, 'no vacuum (injected)'; self.vacuum = False; return res
            self.grip_time = time.monotonic()
        time.sleep(self.get_parameter('vacuum_delay_s').value)     # ejector response time
        r = self.world.call(SetBool.Request(data=req.on))         # blocking call: needs the multithreaded executor
        self.vacuum = bool(req.on and r.success)
        res.success, res.message = r.success, r.message
        return res

    def watch_seal(self):
        t = self.get_parameter('drop_after_s').value
        if self.vacuum and t > 0 and self.grip_time and time.monotonic() - self.grip_time > t:
            self.world.call(SetBool.Request(data=False)); self.vacuum = False
            self.get_logger().warn('seal lost (injected)')


def main():
    rclpy.init(); n = MockGripper()
    ex = MultiThreadedExecutor(); ex.add_node(n); ex.spin()
