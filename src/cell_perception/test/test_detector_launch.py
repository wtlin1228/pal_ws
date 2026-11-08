# Ch 16: node integration test. Run with `launch_test src/cell_perception/test/test_detector_launch.py`.
import time, unittest, rclpy, launch, launch_testing, launch_testing.actions
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from vision_msgs.msg import Detection3DArray


def generate_test_description():
    rsp = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [FindPackageShare('cell_description'), '/launch/rsp_only.launch.py']))
    nodes = [Node(package='cell_sim', executable='world_model'),
             Node(package='cell_sim', executable='mock_camera', parameters=[{'continuous_hz': 2.0}]),
             Node(package='cell_perception', executable='box_detector')]
    spawn = ExecuteProcess(cmd=['ros2', 'run', 'cell_sim', 'spawn_scene', '--layers', '1'])
    return launch.LaunchDescription([rsp, *nodes, TimerAction(period=3.0, actions=[spawn]),
                                     launch_testing.actions.ReadyToTest()]), {}


class TestDetector(unittest.TestCase):
    def test_six_boxes(self):
        rclpy.init(); node = rclpy.create_node('probe'); got = []
        node.create_subscription(Detection3DArray, 'detections', got.append, 10)
        t0 = time.time()
        while time.time() - t0 < 15 and not any(len(m.detections) == 6 for m in got):
            rclpy.spin_once(node, timeout_sec=0.2)
        rclpy.shutdown()
        self.assertTrue(any(len(m.detections) == 6 for m in got))
