#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robot_interfaces.srv import DriveToTarget

class DriveBot(Node):
    def __init__(self):
        super().__init__("drive_bot")
        self.get_logger().info("Hello drive bot")
        
        # create publisher
        self.motor_command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # * 建立 Service ( for service only )
        self.create_service(DriveToTarget, "ball_chaser/command_robot", self.handle_drive_request)

    # * 建立 call back function, 撰寫執行動作
    def handle_drive_request(self, request, response):
        self.get_logger().info("DriveToTargetRequest received - linear_x: " \
            + str(request.linear_x) + ", angular_z: " + str(request.angular_z))

        # setup transer message
        motor_command = Twist()
        motor_command.linear.x = request.linear_x
        motor_command.angular.z = request.angular_z
        # publish message to /cmd_vel topic
        self.motor_command_publisher.publish(motor_command)
        # setup return message
        response.msg_feedback = "OK"
        return response


def main(args=None):
    #  初始化 ROS
    rclpy.init(args=args)
    #  建立 Node
    node = DriveBot()
    #  Spin Node 以持續運行Node工作(執行 callback function)
    rclpy.spin(node)
    #  Shutdown ROS  
    rclpy.shutdown()

if __name__=='__main__':
    main()