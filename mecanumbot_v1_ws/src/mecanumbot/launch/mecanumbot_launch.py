from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            str(get_package_share_path('teleop_twist_joy')/'launch/teleop-launch.py'),
            launch_arguments={'joy_config': 'ps3'}.items()
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
        IncludeLaunchDescription(
            str(get_package_share_path('slam_toolbox')/'launch/online_async_launch.py'),
            launch_arguments={
                'params_file': os.environ.get('MECANUMBOT_WS_PATH') + '/src/mecanumbot/config/mapper_params_online_async.yaml'
            }.items()
        )
    ])