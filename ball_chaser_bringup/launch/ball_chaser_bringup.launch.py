from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    drive_bot_node = Node(
        package="ball_chaser",
        executable="drive_bot"
    )

    process_image_node = Node(
        package="ball_chaser",
        executable="process_image"
    )

    ld.add_action(drive_bot_node)
    ld.add_action(process_image_node)

    return ld