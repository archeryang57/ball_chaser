#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_interfaces.srv import DriveToTarget

class MySubscription(Node):
    def __init__(self):
        super().__init__("my_subscription")
        self.get_logger().info("Hello Subscription")

    
    # * 建立 Timer/Subscription ( 傳入 callback function )
        self.create_subscription(DriveToTarget, "hello_msg", self.callback_func, 10)

    # * 建立 call back function, 撰寫執行動作
    def callback_func(self, srv):
        self.get_logger().info("Linear x =" + str( srv.linear_x))


def main(args=None):
    #  初始化 ROS
    rclpy.init(args=args)
    #  建立 Node
    node = MySubscription()

    #  Spin Node 以持續運行Node工作(執行 callback function)
    rclpy.spin(node)
    #  Shutdown ROS  
    rclpy.shutdown()

if __name__=='__main__':
    main()