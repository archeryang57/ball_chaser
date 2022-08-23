from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    ld = LaunchDescription()

    process_image_node = Node(
        package="ball_chaser",
        executable="process_image_cv"
    )

    ld.add_action(process_image_node)


    return ld