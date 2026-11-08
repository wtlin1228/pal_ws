"""Ch 14: a belt with a photo-eye at the robot's pick point and two PLC-style commands: index_out and feed_in.
The belt runs along +y of conveyor_frame (aligned with world). world/spawn_box is an upsert, which is how boxes move."""
import time, rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Vector3
from cell_interfaces.msg import Box, BoxArray
from cell_interfaces.srv import SpawnBox
from cell_sim.geometry import trans, pose_from_mat
from cell_sim.spawn_scene import SKUS

EYE_Y, END_Y, START_Y = 0.0, 0.75, -0.70     # along the belt


class MockConveyor(Node):
    """A belt with a photo-eye at the robot's pick point and two PLC-style commands."""

    def __init__(self):
        super().__init__('mock_conveyor')
        self.declare_parameter('speed_mps', 0.2)
        self.declare_parameter('feed_sequence', 'AAAAAA')     # SKUs fed in order, repeating
        self.declare_parameter('jam', False)                  # Ch 14 checkpoint: a stuck belt
        cb = ReentrantCallbackGroup()
        self.boxes, self.running, self.fed = [], False, 0
        self.create_subscription(BoxArray, 'world/boxes', lambda m: setattr(self, 'boxes', m.boxes), 10, callback_group=cb)
        self.spawn = self.create_client(SpawnBox, 'world/spawn_box', callback_group=cb)
        self.pub_eye = self.create_publisher(Bool, 'photo_eye', 10)
        self.pub_sku = self.create_publisher(String, 'conveyor/sku', 10)
        self.create_service(Trigger, 'conveyor/index_out', self.index_out, callback_group=cb)
        self.create_service(Trigger, 'conveyor/feed_in', self.feed_in, callback_group=cb)
        self.create_timer(0.05, self.tick, callback_group=cb)

    def on_belt(self):
        return [b for b in self.boxes if b.location == 'conveyor']

    def at_eye(self):
        return [b for b in self.on_belt() if abs(b.pose.pose.position.y - EYE_Y) < b.size.y / 2]

    def tick(self):
        eye = self.at_eye()
        self.pub_eye.publish(Bool(data=bool(eye)))
        self.pub_sku.publish(String(data=eye[0].sku if eye else ''))
        if not self.running or self.get_parameter('jam').value:
            return
        dy = self.get_parameter('speed_mps').value * 0.05
        for b in self.on_belt():
            if b.pose.pose.position.y >= END_Y:                 # parked at the end: downstream takes over
                continue
            b.pose.pose.position.y += dy
            self.spawn.call_async(SpawnBox.Request(box=b))

    def index_out(self, req, res):
        """Run until the box at the eye has left it, plus half a second."""
        self.running, t0 = True, time.monotonic()
        while self.at_eye() and time.monotonic() - t0 < 10.0:
            time.sleep(0.05)
        time.sleep(0.5); self.running = False
        res.success, res.message = True, 'indexed'
        return res

    def feed_in(self, req, res):
        """Spawn the next box upstream and run until it blocks the eye."""
        seq = self.get_parameter('feed_sequence').value
        sku = seq[self.fed % len(seq)]; self.fed += 1
        b = Box(id=f'in_{self.fed:03d}', sku=sku, size=Vector3(x=SKUS[sku][0], y=SKUS[sku][1], z=SKUS[sku][2]))
        b.pose.header.frame_id = 'conveyor_frame'
        b.pose.pose = pose_from_mat(trans(0.0, START_Y, SKUS[sku][2] / 2))
        self.spawn.call(SpawnBox.Request(box=b))
        self.running, t0 = True, time.monotonic()
        while not self.at_eye() and time.monotonic() - t0 < 15.0:
            time.sleep(0.05)
        self.running = False
        res.success = bool(self.at_eye()); res.message = f'{sku} at eye' if res.success else 'feed timeout'
        return res


def main():
    rclpy.init(); n = MockConveyor()
    ex = MultiThreadedExecutor(); ex.add_node(n); ex.spin()
