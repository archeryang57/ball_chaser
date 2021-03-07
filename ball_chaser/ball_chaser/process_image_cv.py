#!/usr/bin/env python3
from collections import deque
from imutils.video import VideoStream
import imutils
import cv2
import cv_bridge
# import time

import rclpy
import numpy as np
from rclpy.node import Node
from robot_interfaces.srv import DriveToTarget
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

class ProcessImageCV(Node):
    right_state = 0
    left_state = 0
    
    def __init__(self):
        super().__init__("process_image")
        self.get_logger().info("Hello process_image")

        # init the OpenCV dependance environment
        self.bridge = cv_bridge.CvBridge()
        # self.greenLower = (29, 86, 6)
        self.greenLower = (31, 120, 20)
        self.greenUpper = (64, 255, 255)
        self.pts = deque(maxlen=32)
        self.counter = 0
        self.direction = 0.0
        self.maxTurnForce = 2.6
        self.maxForwardForce = 0.21


        # * 建立 camera Subscription
        # self.create_subscription(Image, "/image_raw", self.callback_func, 10)
        self.create_subscription(CompressedImage, "/image_raw/compressed", self.callback_func, 10)

        self.publichser_ = self.create_publisher(Image, "ball_image", 10)

        self.motor_command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)


    def drive_robot(self, lin_x,  ang_z):
        # TODO: Request a service and pass the velocities to it to drive the robot
        # self.get_logger().info(f"Driving the bot forward({lin_x}, turn({ang_z})")

        motor_command = Twist()
        motor_command.linear.x = lin_x
        motor_command.angular.z = ang_z
        # publish message to /cmd_vel topic
        self.motor_command_publisher.publish(motor_command)

    # * 建立 call back function, 撰寫執行動作
    def callback_func(self, msgimg: CompressedImage):
        # frame = self.bridge.imgmsg_to_cv2(msgimg, "bgr8")
        np_arr = np.fromstring(msgimg.data.tostring(), np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        frame = cv2.flip(frame, 1)

        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if frame is None:
            return 1

        # height = msgimg.height # image height, that is, number of rows  640
        # width = msgimg.width # image width, that is, number of columns  480
        height, width = frame.shape[:2]
        centerX = width /2
        centerY = height /2

        # resize the frame, blur it, and convert it to the HSV
        # color space
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = (centerX + 0.0 , centerY + 0.0)
        
        (dX, dY) = (0.0, 0.0)
        ((x, y), radius) = ((0.0, 0.0), 0.0) 
        M = 0

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            # only proceed if the radius meets a minimum size
            if radius >= 2:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    center = ( int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                     (0, 255, 255), 2)
                # cv2.circle(frame, center, 5, (0, 0, 255), -1)
                # self.pts.appendleft(center)
                
        # # loop over the set of tracked points
        # for i in np.arange(1, len(self.pts)):
        #     # if either of the tracked points are None, ignore them
        #     if self.pts[i - 1] is None or self.pts[i] is None:
        #         continue

        #     if len(self.pts)>20:
        #         if self.counter >= 10 and i == 1 and self.pts[-10] is not None:
        #             # dX is the data for left or right movement.
        #             # optive is turn right. nagetive is turn left.
        #             dX = self.pts[-10][0] - self.pts[i][0]
        #             dY = self.pts[-10][1] - self.pts[i][1]
        #             (dirX, dirY) = ("", "")

        #             if np.abs(dX) > 10:
        #                 dirX = "East" if np.sign(dX) == 1 else "West"

        #             if np.abs(dY) > 10:
        #                 dirY = "North" if np.sign(dY) == 1 else "South"
        #             # handle when both directions are non-empty
        #             if dirX != "" and dirY != "":
        #                 self.direction = "{}-{}".format(dirY, dirX)
        #             # otherwise, only one direction is non-empty
        #             else:
        #                 self.direction = dirX if dirX != "" else dirY
                    
        #     # otherwise, compute the thickness of the line and draw the connecting lines
        #     thickness = int(np.sqrt(32 / float(i + 1)) * 2.5)
        #     cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)
        # # show the movement deltas and the direction of movement on the frame
        # cv2.putText(frame, self.direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
        #     0.65, (0, 0, 255), 3)
        # cv2.putText(frame, "dx: {}, dy: {}".format(dX, dY),
        #     (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
        #     0.35, (0, 0, 255), 1)

        self.publichser_.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        self.counter += 1     

        # ball found, calculate forces.
        if radius > 0:
            # self.maxForwardForce = 0.21
            # self.maxTurnForce = 2.6
            forwardForce = self.maxForwardForce - ( ( radius * 2 ) / width * self.maxForwardForce )
            forwardForce = 0.0 if forwardForce < 0.0 else forwardForce

            inCenterZone = True if abs(center[0]-centerX) < 20 else False
            if inCenterZone:
                turnForce = 0.0
            else:
                turnForce = ( (center[0] - centerX) / width * 2 ) * (self.maxTurnForce-0.5)
                self.direction = turnForce
            
            self.drive_robot (forwardForce, turnForce)

            self.get_logger().info(f"radius:({round(radius,2)}), Ball Center:({center}), Forward/Turn: {round(forwardForce,2)}/{round(turnForce,2)}")
        else:
            self.drive_robot (0.0, self.direction)
            self.direction = self.direction - 0.01 if self.direction > 0 else self.direction + 0.01
            if abs(self.direction) <= 0.2:
                self.direction = 0.0


        return 0


def main(args=None):
    #  初始化 ROS
    rclpy.init(args=args)
    #  建立 Node
    node = ProcessImageCV()

    #  Spin Node 以持續運行Node工作(執行 callback function)
    rclpy.spin(node)
    #  Shutdown ROS  
    rclpy.shutdown()

if __name__=='__main__':
    main()