from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    ld = LaunchDescription()

    camera_node = Node(
        package="ros_deep_learning",
        executable="video_source",
        parameters=[
            {"resource": "csi://0"},
            {"width": 320},
            {"height": 180},
            {"loop": 0},
            {"flip": "horizontal"},
            {"rtsp_latency": 0}
        ]
    )

    dc_motor_node = Node(
        package="dc_motor",
        executable="dc_motor"
    )

    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')
    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'ydlidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    lidar_node = LifecycleNode(package='ydlidar_ros2_driver',
                                node_executable='ydlidar_ros2_driver_node',
                                node_name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                # node_namespace='/',
                                )
    
    tf2_node = Node(package='tf2_ros',
                    node_executable='static_transform_publisher',
                    node_name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
                    )


    process_image_node = Node(
        package="ball_chaser",
        executable="process_image_cv"
    )

    ld.add_action(camera_node)
    ld.add_action(dc_motor_node)
    ld.add_action(params_declare)
    ld.add_action(lidar_node)
    ld.add_action(tf2_node)
    ld.add_action(process_image_node)

    return ld