from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(pkg, name, **args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource([FindPackageShare(pkg), '/launch/', name]),
                                    launch_arguments=args.items())


def generate_launch_description():
    return LaunchDescription([
        include('cell_moveit_config', 'demo.launch.py'),          # robot, controllers, move_group, RViz
        include('cell_sim', 'twin.launch.py'),                    # world_model, mock_gripper, mock_camera
        Node(package='cell_perception', executable='box_detector', output='screen'),
        Node(package='cell_motion', executable='pick_place_server', output='screen'),
        Node(package='cell_executive', executable='executive', output='screen'),
    ])
