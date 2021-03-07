from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()


    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        parameters=[{"image_size": [320,180]}]
    )

    process_image_node = Node(
        package="ball_chaser",
        executable="process_image_cv"
    )

    ld.add_action(camera_node)
    ld.add_action(process_image_node)

    return ld