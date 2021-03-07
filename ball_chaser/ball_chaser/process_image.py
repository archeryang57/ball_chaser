#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from robot_interfaces.srv import DriveToTarget
from sensor_msgs.msg import Image

class ProcessImage(Node):
    right_state = 0
    left_state = 0
    
    def __init__(self):
        super().__init__("process_image")
        self.get_logger().info("Hello process_image")

        # * 建立 camera Subscription
        self.create_subscription(Image, "/image_raw", self.callback_func, 10)

        self.publichser_ = self.create_publisher(Image, "ball_image", 10)


    def drive_robot(self, lin_x,  ang_z):
        # TODO: Request a service and pass the velocities to it to drive the robot
        self.get_logger().info(f"Driving the bot forward({lin_x}, turn({ang_z})")

        # Request specified velocity and direction
        req = DriveToTarget.Request()
        req.linear_x = lin_x
        req.angular_z = ang_z

        # Call the command_robot service and pass the specified velocity and direction.
        client_ = self.create_client(DriveToTarget, '/ball_chaser/command_robot')
        # while not client_.wait_for_service(1.0):
        #     self.get_logger().warn("Wait for command_robot service")

        future = client_.call_async(req)
        future.add_done_callback(self.drive_bot_callback)



    def drive_bot_callback(self, future):
        # self.get_logger().info("Client call finished.")
        pass

    # * 建立 call back function, 撰寫執行動作
    def callback_func(self, img: Image):
        self.get_logger().info("Entering the process_image_callback function...")

        height = img.height # image height, that is, number of rows  640
        width = img.width # image width, that is, number of columns  480
        step  = img.step  # Full row length in bytes                1920 = 640 * 8
        # self.get_logger().info(f"height:{height},  width:{width}, step:{step}")
        # The definition of above parameters could be found at 
        # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html

        white_position = []  # a vector to store the positions of white pixels
        positionId = 0 # The position ID of each white pixel we find. 
        # This will be stored in  white_position vector.

        # TODO:
        # (1) Loop through each pixel in the image and check if there's a bright white one
        maxRed = 90  #60
        minGreen = 100 # 110
        maxBlue = 120
        #maxTurnForce = 2.6
        #maxForwardForce = 0.21
        turnForce = 0.5  # max is 2.6
        forwardForce = 0.1  # max is 0.21

        for i in range(0, height*step, 3):
            if img.data[i] < maxBlue  and img.data[i + 1] > minGreen and img.data[i + 2] < maxRed:
                # img.data[i]: Blue; img.data[i + 1]: Green; img.data[i + 2]: Red
                positionId = i % ( width * 3 ) / 3
                # Insert the positionId into white_position vector:
                white_position.append(positionId)
        # (2) Identify if the average of these pixels falls in the left, mid, or right side of the image
        avg = 0
        sum = 0
        size = len(white_position)

        for k in white_position:
            sum += k
        # for k in range(0, size):
        #     sum += white_position[k]

        if size <= 20:
            # Will request a stop when there's no white ball seen by the camera
            self.drive_robot(0.0, 0.0)  # This request a stop
            self.left_state = 0
            self.right_state = 0
        else:
            avg = sum / size

            # Depending on the white ball position, call the drive_bot function and pass velocities to it
            if avg <= width / 3:
                self.drive_robot(0.0, turnForce)  # This request should drive my_robot left
                self.left_state = 1
                self.right_state = 0
            elif avg >= 2 * width / 3:
                self.drive_robot(0.0, -turnForce) # This request drives my_robot right
                self.left_state = 0
                self.right_state = 1 
            else:
                self.drive_robot(forwardForce, 0.0)  # This request drives my_robot robot forward
                self.left_state = 0
                self.right_state = 0
        #self.get_logger().info(f"left_state: {self.left_state}, right_state: {self.right_state}")


def main(args=None):
    #  初始化 ROS
    rclpy.init(args=args)
    #  建立 Node
    node = ProcessImage()

    #  Spin Node 以持續運行Node工作(執行 callback function)
    rclpy.spin(node)
    #  Shutdown ROS  
    rclpy.shutdown()

if __name__=='__main__':
    main()