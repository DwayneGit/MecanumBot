from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            name='rviz',
            package='rviz2',
            executable='rviz2',
        ),
        Node(
            name='mecanumbot_localization_data_pub',
            package='localization_data_pub',
            type='rviz_click_to_2d',
        ),
    ])