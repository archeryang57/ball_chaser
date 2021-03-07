#!/usr/bin/env python3
from collections import deque
from imutils.video import VideoStream
import imutils
import cv2
import cv_bridge
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

class TestBall(Node):
    def __init__(self):
        super().__init__("test_ball")
        self.get_logger().info("Hello Subscription")

        # init the OpenCV dependance environment
        self.bridge = cv_bridge.CvBridge()
        #self.greenLower = (29, 86, 6)
        self.greenLower = (31, 120, 20)
        self.greenUpper = (64, 255, 255)
        self.pts = deque(maxlen=32)
        self.counter = 0
        self.direction = ""

        # * 建立 Timer/Subscription ( 傳入 callback function )
        # self.create_subscription(Image, "image_raw", self.callback_func, 10)
        self.create_subscription(CompressedImage, "/image_raw/compressed", self.callback_func, 10)

        # create publisher to public processed image
        self.publichser_ = self.create_publisher(Image, "ball_image", 10)

    # * 建立 call back function, 撰寫執行動作
    def callback_func(self, msgimg: CompressedImage):
        np_arr = np.fromstring(msgimg.data.tostring(), np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # frame = self.bridge.imgmsg_to_cv2(msgimg, "bgr8")

        frame = cv2.flip(frame, 1)

        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if frame is None:
            return 1

        # height = 320 # msgimg.height # image height, that is, number of rows  640
        # width = 200 # msgimg.width # image width, that is, number of columns  480
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
        center = None
        
        (dX, dY) = (0, 0)
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            self.get_logger().info(f"radius:({radius}), Screen Center:({centerX},{centerY}),  Ball Center:({center})")

            # only proceed if the radius meets a minimum size
            if radius > 2:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                self.pts.appendleft(center)
                
        # loop over the set of tracked points
        for i in np.arange(1, len(self.pts)):
            # if either of the tracked points are None, ignore
            # them
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue

            if len(self.pts)>20:
                if self.counter >= 10 and i == 1 and self.pts[-10] is not None:
                    # compute the difference between the x and y
                    # coordinates and re-initialize the direction
                    # text variables
                    dX = self.pts[-10][0] - self.pts[i][0]
                    dY = self.pts[-10][1] - self.pts[i][1]
                    (dirX, dirY) = ("", "")
                    # ensure there is significant movement in the
                    # x-direction
                    if np.abs(dX) > 20:
                        dirX = "East" if np.sign(dX) == 1 else "West"
                    # ensure there is significant movement in the
                    # y-direction
                    if np.abs(dY) > 20:
                        dirY = "North" if np.sign(dY) == 1 else "South"
                    # handle when both directions are non-empty
                    if dirX != "" and dirY != "":
                        self.direction = "{}-{}".format(dirY, dirX)
                    # otherwise, only one direction is non-empty
                    else:
                        self.direction = dirX if dirX != "" else dirY
                    
            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(32 / float(i + 1)) * 2.5)
            cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)
        # show the movement deltas and the direction of movement on
        # the frame
        cv2.putText(frame, self.direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (0, 0, 255), 3)
        cv2.putText(frame, "dx: {}, dy: {}".format(dX, dY),
            (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
            0.35, (0, 0, 255), 1)
        # show the frame to our screen and increment the frame self.counter
        # cv2.imshow("Frame", frame)

        self.publichser_.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
#        key = cv2.waitKey(1) & 0xFF
        self.counter += 1
        

        return 0


def main(args=None):
    #  初始化 ROS
    rclpy.init(args=args)
    #  建立 Node
    node = TestBall()

    #  Spin Node 以持續運行Node工作(執行 callback function)
    rclpy.spin(node)
    #  Shutdown ROS  
    rclpy.shutdown()

if __name__=='__main__':
    main()