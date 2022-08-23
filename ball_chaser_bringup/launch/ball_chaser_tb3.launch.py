from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        parameters=[{"image_size": [160,90]}],
        remappings=[
            ("/image_raw", "/camera/image")
        ],
        output='screen',
    )

    drive_bot_node = Node(
        package="ball_chaser_cpp",
        executable="drive_bot"
    )

    process_image_node = Node(
        package="ball_chaser_cpp",
        executable="process_image"
    )

    ld.add_action(camera_node)
    ld.add_action(drive_bot_node)
    ld.add_action(process_image_node)

    return ld
