from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    period = LaunchConfiguration('period_s')
    params = PathJoinSubstitution([FindPackageShare('cell_bringup'), 'config', 'basics.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('period_s', default_value='1.0'),
        Node(package='ros2_basics', executable='photo_eye', output='screen',
             parameters=[params, {'period_s': period}]),   # later entries override earlier ones
        Node(package='ros2_basics', executable='suction', output='screen'),
        Node(package='ros2_basics', executable='countdown', output='screen'),
    ])
