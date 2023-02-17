from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            str(get_package_share_path('teleop_twist_joy')/'launch/teleop-launch.py'),
            launch_arguments={'joy_config': 'n64'}.items()
        ),
        Node(
            name='mecanumbot_micro_ros_agent',
            package='micro_ros_agent',
            executable='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200'],
        ),
        Node(
            name='mecanumbot',
            package='mecanumbot',
            executable='mecanumbot_node',
        ),
    ])