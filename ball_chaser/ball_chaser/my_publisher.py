#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_interfaces.srv import DriveToTarget

class MyPublisher(Node):
    def __init__(self):
        super().__init__("my_publisher")
        self.get_logger().info("Hello Publisher")

    # * 建立 Service ( for service only )
        self.create_service(DriveToTarget, "hello_msg", self.callback_func)

    # * 建立 call back function, 撰寫執行動作
    def callback_func(self, request, response):
        request.linear_x = 0.2
        request.angular_z = 0
        self.service_.publish(drv)


def main(args=None):
    #  初始化 ROS
    rclpy.init(args=args)
    #  建立 Node
    node = MyPublisher()

    #  Spin Node 以持續運行Node工作(執行 callback function)
    rclpy.spin(node)
    #  Shutdown ROS  
    rclpy.shutdown()

if __name__=='__main__':
    main()