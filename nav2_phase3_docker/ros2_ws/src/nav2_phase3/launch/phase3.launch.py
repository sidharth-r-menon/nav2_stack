from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params = os.path.join(get_package_share_directory('nav2_phase3'), 'config', 'phase3.yaml')
    return LaunchDescription([
        # RGB-D ball detector: publishes /phase3/ball_pose when the red ball is confirmed
        Node(package='nav2_phase3', executable='ball_detector', name='ball_detector',
             output='screen', parameters=[params]),
        # Ball approach manager: stops frontier_exploration_ros2 and sends Nav2 goal
        Node(package='nav2_phase3', executable='ball_approach_manager', name='ball_approach_manager',
             output='screen', parameters=[params]),
        # NOTE: mission_manager has been removed. Frontier exploration is handled by
        # the frontier_exploration_ros2 node launched from start_nav2_phase3.sh
    ])
