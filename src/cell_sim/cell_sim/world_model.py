"""Ch 8: single source of truth for the mocked cell. Mock devices read it; robot software must not."""
import copy, numpy as np, rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import TransformStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from cell_interfaces.msg import Box, BoxArray
from cell_interfaces.srv import SpawnBox, GetBoxes
from cell_sim.geometry import mat_from_pose, mat_from_tf, pose_from_mat, trans

PALLET   = dict(x=0.0,  y=0.75, z=0.144, sx=0.8, sy=0.6)   # deck top center and size, world frame
CONVEYOR = dict(x=-0.9, y=0.0,  z=0.70,  sx=0.5, sy=1.5)
SKU_COLORS = {'A': (0.91, 0.38, 0.15), 'B': (0.39, 0.43, 0.45), 'C': (0.82, 0.08, 0.10)}


class WorldModel(Node):
    """Single source of truth for the mocked cell. Mock devices read it; robot software must not."""

    def __init__(self):
        super().__init__('world_model')
        self.boxes: dict[str, Box] = {}          # id -> Box, poses always in 'world'
        self.held: str | None = None
        self.T_tcp_box = None                     # fixed offset while a box is held
        self.buf = Buffer(); self.tf_listener = TransformListener(self.buf, self)
        self.br = TransformBroadcaster(self)
        cb = ReentrantCallbackGroup()
        self.create_service(SpawnBox, 'world/spawn_box', self.spawn, callback_group=cb)
        self.create_service(GetBoxes, 'world/get_boxes', self.get_boxes, callback_group=cb)
        self.create_service(Trigger, 'world/reset', self.reset, callback_group=cb)
        self.create_service(SetBool, 'world/grip', self.grip, callback_group=cb)
        self.pub_boxes = self.create_publisher(BoxArray, 'world/boxes', 10)
        self.pub_markers = self.create_publisher(MarkerArray, 'world/markers', 10)
        self.create_timer(0.1, self.tick)

    # ---- services -----------------------------------------------------------------
    def spawn(self, req, res):
        box = copy.deepcopy(req.box)
        if box.pose.header.frame_id != 'world':          # store everything in one frame
            try:
                tf = self.buf.lookup_transform('world', box.pose.header.frame_id, rclpy.time.Time(),
                                               timeout=Duration(seconds=2.0))
            except TransformException as e:                 # TF not up yet: answer, never hang the caller
                res.success, res.message = False, f'no transform for {box.pose.header.frame_id}: {e}'; return res
            box.pose.pose = pose_from_mat(mat_from_tf(tf.transform) @ mat_from_pose(box.pose.pose))
            box.pose.header.frame_id = 'world'
        box.location = self.location_of(box)
        self.boxes[box.id] = box
        res.success, res.message = True, f'spawned {box.id} on {box.location}'
        return res

    def get_boxes(self, req, res):
        res.boxes = [b for b in self.boxes.values() if not req.location or b.location == req.location]
        return res

    def reset(self, req, res):
        self.boxes.clear(); self.held = None
        res.success = True; return res

    def grip(self, req, res):
        """on: attach the box whose top face is under the TCP. off: release it and let it settle."""
        try:
            T_world_tcp = mat_from_tf(self.buf.lookup_transform('world', 'tcp', rclpy.time.Time()).transform)
        except TransformException as e:
            res.success, res.message = False, str(e); return res
        if req.data:
            hit = self.box_under(T_world_tcp)
            if hit is None:
                res.success, res.message = False, 'no box under the pad'; return res
            self.held = hit
            self.T_tcp_box = np.linalg.inv(T_world_tcp) @ mat_from_pose(self.boxes[hit].pose.pose)
            self.boxes[hit].location = 'gripper'
            res.success, res.message = True, f'holding {hit}'
        else:
            if self.held:
                box, self.held = self.boxes[self.held], None
                self.settle(box)
            res.success, res.message = True, 'released'
        return res

    # ---- mock physics ---------------------------------------------------------------
    def box_under(self, T_world_tcp, tol=0.02):
        p = np.append(T_world_tcp[:3, 3], 1.0)
        for b in self.boxes.values():
            if b.location == 'gripper':
                continue
            lp = np.linalg.inv(mat_from_pose(b.pose.pose)) @ p     # TCP in the box's own frame
            if abs(lp[0]) <= b.size.x / 2 and abs(lp[1]) <= b.size.y / 2 and abs(lp[2] - b.size.z / 2) < tol:
                return b.id
        return None

    def settle(self, box):
        """Mock gravity: drop straight down onto the highest thing below (surface or box)."""
        T = mat_from_pose(box.pose.pose); x, y = T[0, 3], T[1, 3]
        support = 0.0
        for a in (PALLET, CONVEYOR):
            if abs(x - a['x']) <= a['sx'] / 2 and abs(y - a['y']) <= a['sy'] / 2:
                support = a['z']
        for o in self.boxes.values():                                # axis-aligned footprint overlap
            if o.id == box.id or o.location == 'gripper':
                continue
            To = mat_from_pose(o.pose.pose)
            if abs(To[0, 3] - x) < (o.size.x + box.size.x) / 2 and abs(To[1, 3] - y) < (o.size.y + box.size.y) / 2:
                support = max(support, To[2, 3] + o.size.z / 2)
        T[2, 3] = support + box.size.z / 2
        box.pose.pose = pose_from_mat(T)
        box.location = self.location_of(box)

    def location_of(self, box):
        x, y = box.pose.pose.position.x, box.pose.pose.position.y
        for name, a in (('pallet', PALLET), ('conveyor', CONVEYOR)):
            if abs(x - a['x']) <= a['sx'] / 2 and abs(y - a['y']) <= a['sy'] / 2:
                return name
        return 'unknown'

    # ---- 10 Hz: held box follows the TCP; publish truth for the mocks and RViz -------
    def tick(self):
        if self.held:
            try:
                T_world_tcp = mat_from_tf(self.buf.lookup_transform('world', 'tcp', rclpy.time.Time()).transform)
                self.boxes[self.held].pose.pose = pose_from_mat(T_world_tcp @ self.T_tcp_box)
            except TransformException:
                pass
        now = self.get_clock().now().to_msg()
        arr = BoxArray(); arr.header.stamp, arr.header.frame_id = now, 'world'
        markers = MarkerArray(); markers.markers.append(Marker(action=Marker.DELETEALL))
        for i, b in enumerate(self.boxes.values()):
            b.pose.header.stamp = now
            arr.boxes.append(b)
            m = Marker(type=Marker.CUBE, action=Marker.ADD, id=i, ns='boxes')
            m.header.frame_id, m.header.stamp = 'world', now
            m.pose, m.scale = b.pose.pose, b.size
            m.color.r, m.color.g, m.color.b = SKU_COLORS.get(b.sku, (0.6, 0.6, 0.6)); m.color.a = 0.9
            markers.markers.append(m)
            t = TransformStamped(); t.header.stamp, t.header.frame_id, t.child_frame_id = now, 'world', f'box_{b.id}'
            p = b.pose.pose.position
            t.transform.translation = Vector3(x=p.x, y=p.y, z=p.z); t.transform.rotation = b.pose.pose.orientation
            self.br.sendTransform(t)
        self.pub_boxes.publish(arr); self.pub_markers.publish(markers)


def main():
    rclpy.init(); n = WorldModel()
    ex = MultiThreadedExecutor(); ex.add_node(n); ex.spin()
