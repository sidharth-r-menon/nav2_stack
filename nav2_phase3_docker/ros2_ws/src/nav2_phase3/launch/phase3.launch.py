from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params = os.path.join(get_package_share_directory('nav2_phase3'), 'config', 'phase3.yaml')
    return LaunchDescription([
        Node(package='nav2_phase3', executable='ball_detector', name='ball_detector', output='screen', parameters=[params]),
        Node(package='nav2_phase3', executable='ball_approach_manager', name='ball_approach_manager', output='screen', parameters=[params]),
    ])
