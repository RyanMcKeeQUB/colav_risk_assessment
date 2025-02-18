import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.logging import get_logger
import yaml
import os

logger = get_logger("colav_risk_assesment.launch.py")
package_name = 'colav_risk_assessment'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package=package_name,
            executable='unsafe_set_generator_node',
            name='unsafe_set_gen',
            output='screen',
        ),
    ])