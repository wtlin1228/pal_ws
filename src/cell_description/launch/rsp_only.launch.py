"""Ch 16: robot_state_publisher only (no GUI, no RViz), for integration tests and headless runs."""
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('cell_description')
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'cell.urdf.xacro'])
    robot_description = ParameterValue(Command([FindExecutable(name='xacro'), ' ', xacro_file]), value_type=str)
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher'),   # fixed default joint state
    ])
