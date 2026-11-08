from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('cell_description')
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'cell_control.urdf.xacro'])
    robot_description = ParameterValue(Command([FindExecutable(name='xacro'), ' ', xacro_file]), value_type=str)
    controllers = PathJoinSubstitution([pkg, 'config', 'ros2_controllers.yaml'])
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        # ros2_control_node reads the URDF from the /robot_description topic in Jazzy
        Node(package='controller_manager', executable='ros2_control_node', parameters=[controllers], output='screen',
             remappings=[('~/robot_description', '/robot_description')]),
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']),
        Node(package='controller_manager', executable='spawner', arguments=['arm_controller']),
        Node(package='rviz2', executable='rviz2', arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'cell.rviz'])]),
    ])
