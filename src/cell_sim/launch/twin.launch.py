from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='cell_sim', executable='world_model', output='screen'),
        Node(package='cell_sim', executable='mock_gripper', output='screen'),
        Node(package='cell_sim', executable='mock_camera', output='screen',
             parameters=[{'continuous_hz': 2.0}]),        # continuous while developing; triggered in Ch 13
    ])
