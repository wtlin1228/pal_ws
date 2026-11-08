"""Ch 9: turns the world model into a PointCloud2 in camera_optical_frame, with noise, dropout and occlusion."""
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener, TransformException
from cell_interfaces.msg import BoxArray
from cell_sim.geometry import mat_from_pose, mat_from_tf, trans
from cell_sim.world_model import PALLET


class MockCamera(Node):
    def __init__(self):
        super().__init__('mock_camera')
        self.declare_parameter('noise_sigma_m', 0.002)
        self.declare_parameter('dropout', 0.05)
        self.declare_parameter('spacing_m', 0.005)
        self.declare_parameter('continuous_hz', 0.0)        # 0 = only on camera/capture
        self.boxes = []
        self.rng = np.random.default_rng(0)
        self.buf = Buffer(); self.tf_listener = TransformListener(self.buf, self)
        self.create_subscription(BoxArray, 'world/boxes', lambda m: setattr(self, 'boxes', m.boxes), 10)
        self.pub = self.create_publisher(PointCloud2, 'camera/points', qos_profile_sensor_data)
        self.pub_truth = self.create_publisher(BoxArray, 'camera/ground_truth', 10)   # Ch 9 checkpoint: evaluation only
        self.create_service(Trigger, 'camera/capture', self.capture)
        hz = self.get_parameter('continuous_hz').value
        if hz > 0:
            self.create_timer(1.0 / hz, self.publish_cloud)

    def capture(self, req, res):
        res.success = self.publish_cloud(); res.message = 'captured'
        return res

    @staticmethod
    def face_points(T, sx, sy, z, step):
        """Grid of points on a horizontal face of size sx x sy at local height z, transformed by T."""
        u = np.arange(-sx / 2, sx / 2 + 1e-9, step); v = np.arange(-sy / 2, sy / 2 + 1e-9, step)
        U, V = np.meshgrid(u, v)
        P = np.column_stack([U.ravel(), V.ravel(), np.full(U.size, z), np.ones(U.size)])
        return (T @ P.T).T[:, :3]

    def publish_cloud(self) -> bool:
        step = self.get_parameter('spacing_m').value
        boxes = [b for b in self.boxes if b.location == 'pallet']
        deck = trans(PALLET['x'], PALLET['y'], PALLET['z'])
        parts = [self.face_points(deck, PALLET['sx'], PALLET['sy'], 0.0, step)]
        for b in boxes:
            parts.append(self.face_points(mat_from_pose(b.pose.pose), b.size.x, b.size.y, b.size.z / 2, step))
        P = np.vstack(parts)

        # occlusion from straight above: hidden if inside a box's footprint and below that box's top
        visible = np.ones(len(P), dtype=bool)
        Ph = np.column_stack([P, np.ones(len(P))])
        for b in boxes:
            T = mat_from_pose(b.pose.pose); top = T[2, 3] + b.size.z / 2
            local = (np.linalg.inv(T) @ Ph.T).T
            inside = (np.abs(local[:, 0]) < b.size.x / 2 - 1e-6) & (np.abs(local[:, 1]) < b.size.y / 2 - 1e-6)
            visible &= ~(inside & (P[:, 2] < top - 1e-3))
        P = P[visible]

        try:
            tf = self.buf.lookup_transform('camera_optical_frame', 'world', rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f'{e}'); return False
        P = (mat_from_tf(tf.transform) @ np.column_stack([P, np.ones(len(P))]).T).T[:, :3]
        P += self.rng.normal(0.0, self.get_parameter('noise_sigma_m').value, P.shape)
        P = P[self.rng.random(len(P)) > self.get_parameter('dropout').value]

        header = Header(stamp=self.get_clock().now().to_msg(), frame_id='camera_optical_frame')
        self.pub.publish(pc2.create_cloud_xyz32(header, P.astype(np.float32)))
        truth = BoxArray(header=Header(stamp=header.stamp, frame_id='world'), boxes=boxes)
        self.pub_truth.publish(truth)                      # what the cloud was rendered from; never for control
        return True


def main():
    rclpy.init(); rclpy.spin(MockCamera()); rclpy.shutdown()
