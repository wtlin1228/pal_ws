"""Ch 8: a CLI that fills the pallet with a named scene through world/spawn_box."""
import argparse, rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Vector3
from cell_interfaces.msg import Box
from cell_interfaces.srv import SpawnBox
from cell_sim.geometry import trans, pose_from_mat

SKUS = {'A': (0.30, 0.18, 0.15), 'B': (0.20, 0.20, 0.12), 'C': (0.40, 0.30, 0.20)}   # x, y, z [m]


def single_sku(layers):            # 2 x 3 grid of SKU A per layer with 2 cm gaps, column stacked (Ch 12 interlocks)
    return [('A', trans(x, y, 0.15 * L + 0.075))
            for L in range(layers) for x in (-0.16, 0.16) for y in (-0.2, 0.0, 0.2)]


def mixed():                       # three SKUs with gaps; two boxes sit on top of others
    return [('C', trans(-0.2, -0.15, 0.10)), ('B', trans(-0.2, 0.15, 0.06)), ('A', trans(0.2, -0.2, 0.075)),
            ('A', trans(0.2, 0.0, 0.075)), ('B', trans(0.2, 0.0, 0.21)), ('A', trans(-0.2, -0.15, 0.275))]


def call(node, client, req):
    fut = client.call_async(req); rclpy.spin_until_future_complete(node, fut); return fut.result()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scene', choices=['single_sku', 'mixed'], default='single_sku')
    ap.add_argument('--layers', type=int, default=2)
    args = ap.parse_args()
    rclpy.init(); node = Node('spawn_scene')
    reset, spawn = node.create_client(Trigger, 'world/reset'), node.create_client(SpawnBox, 'world/spawn_box')
    reset.wait_for_service(); spawn.wait_for_service()
    call(node, reset, Trigger.Request())
    scene = single_sku(args.layers) if args.scene == 'single_sku' else mixed()
    for n, (sku, T) in enumerate(scene):
        b = Box(id=f'box_{n:03d}', sku=sku, size=Vector3(x=SKUS[sku][0], y=SKUS[sku][1], z=SKUS[sku][2]))
        b.pose.header.frame_id = 'pallet_frame'; b.pose.pose = pose_from_mat(T)   # z = height above the deck
        print(call(node, spawn, SpawnBox.Request(box=b)).message)
    rclpy.shutdown()
