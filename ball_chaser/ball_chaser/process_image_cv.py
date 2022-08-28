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
    
    def __init__(self):
        super().__init__("process_image")
        self.get_logger().info("Hello process_image")

        # 物件變數初始化
        self.bridge = cv_bridge.CvBridge()
        # self.greenLower = (29, 86, 6)
        self.greenLower = (31, 120, 20)
        self.greenUpper = (64, 255, 255)
        self.pts = deque(maxlen=32)
        self.counter = 0
        self.direction = 0.0
        self.maxTurnForce = 3.6     # it is better to get from dc_motor
        self.maxForwardForce = 0.35 # it is better to get from dc_motor
        self.str_direction = ""

        # check if support GPU
        if cv2.cuda.getCudaEnabledDeviceCount() > 0:
            self.isGPU = True
        else:
            self.isGPU = False

        # * 建立 camera Subscription
        # self.create_subscription(CompressedImage, "/image_raw/compressed", self.callback_func, 10)
        self.create_subscription(Image, "/video_source/raw", self.callback_func, 10)

        # 將處理過的 Image publish 以供檢視
        self.publichser_ = self.create_publisher(Image, "ball_image", 10)

        # 因為透過 Service Latancy 很嚴重, 因此直接將動作publish給 TB3, 不透過 Service
        self.motor_command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def drive_robot(self, lin_x,  ang_z):
        motor_command = Twist()
        motor_command.linear.x = float(lin_x)
        motor_command.angular.z = float(ang_z)
        # publish message to /cmd_vel topic
        self.motor_command_publisher.publish(motor_command)
        
#         # drive robot with ACTION
#         req = DriveToTarget.Request()
#         req.linear_x = lin_x
#         req.angular_z = ang_z

#         # Call the command_robot service and pass the specified velocity and direction.
#         client_ = self.create_client(DriveToTarget, '/ball_chaser/command_robot')

    # * 建立 call back function, 撰寫執行動作
    def callback_func(self, msgimg: Image):

        # 若 msg 是 raw image, 直接轉為 CV2 格式
        frame = self.bridge.imgmsg_to_cv2(msgimg, "bgr8")
        
        # 將壓縮影像解壓, 轉為 CV2 格式
        # np_arr = np.fromstring(msgimg.data.tostring(), np.uint8)
        # frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # 取得畫面的高度及寬度        
        # frame.shape example:  ( 1080, 1920, 3 ) ( height, width, color )
        height, width = frame.shape[:2]
        
        # 畫面中心點
        centerX = width /2
        centerY = height /2

        # 畫面做水平鏡射處理
        if self.isGPU:
            cv2.cuda.setDevice(0)
            gpu_frame = cv2.cuda_GpuMat()
            gpu_frame.upload(frame)
            gpu_frame = cv2.cuda.flip(gpu_frame, 1)
            frame = gpu_frame.download()
        else:
            frame = cv2.flip(frame, 1)
        
        # 轉為 HSV 色彩空間
        if  self.isGPU:
            # gpu_frame.upload(frame)
            hsv = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2HSV)
            hsv = hsv.download()
        else:
            hsv = cv2.cvtColor(gpu_frame, cv2.COLOR_BGR2HSV)
            
        # 影像模糊化
        blurred = cv2.GaussianBlur(hsv, (11, 11), 0)

        # 取出綠色部份做為影像遮罩, 以供 findConturs 使用
        if  self.isGPU:
            mask = cv2.cuda.inRange(hsv, self.greenLower, self.greenUpper)
        else:
            mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
        
        # 將影像遮罩做腐蝕及膨脹處理, 以去掉影像毛邊.
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # 取得 mask中的影像輪廓
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # 初始化變數.
        center = (int(centerX), int(centerY))
        ((x, y), radius) = ((0.0, 0.0), 0.0) 

        if len(cnts) > 0:
            # 取得最大的輪廓
            c = max(cnts, key=cv2.contourArea)
            # 取得包含輪廓的最小圓形外框((中心點x,y), 半徑)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            # only proceed if the radius meets a minimum size
            if radius >= 5:
                center = (int(x), int(y))

                # 畫圓形外框
                cv2.circle(frame, center, int(radius), (0, 255, 255), 2)

                # 畫中心點
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                
                # 將物件中心點存入 pts
                self.pts.appendleft(center)
                
        # 處理追蹤點tracked points, 以及畫出軌跡線
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
        #                 self.str_direction = "{}-{}".format(dirY, dirX)
        #             # otherwise, only one direction is non-empty
        #             else:
        #                 self.str_direction = dirX if dirX != "" else dirY
                    
        #     # otherwise, compute the thickness of the line and draw the connecting lines
        #     thickness = int(np.sqrt(32 / float(i + 1)) * 2.5)
        #     cv2.line(frame, self.pts[i-1], self.pts[i], (0, 0, 255), thickness)
    
        # # show the movement deltas and the direction of movement on the frame
        # cv2.putText(frame, self.str_direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
        #     0.65, (0, 0, 255), 3)
        # cv2.putText(frame, "dx: {}, dy: {}".format(dX, dY),
        #     (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
        #     0.35, (0, 0, 255), 1)

        # 將處理過的圖形 publish 供檢視
        self.publichser_.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        self.counter += 1     

        # 若有找到符合物件, 則計算驅動力
        if radius >= 5:
            # self.maxForwardForce = 0.35
            # self.maxTurnForce = 3.6
            # radius has 20 forward steps
            # 計算前進驅動力: 最大前進驅動力 - ((半徑*2) /寬度  * 最大前進驅動力)
            # forwardForce = self.maxForwardForce - ( ( radius * 2 ) / width * self.maxForwardForce )

            # forwardForce calculate method 2
            radius = radius if radius < width /2 else width / 2
            forwardUnit = width / 2 / 20
            forwardForce = self.maxForwardForce - (radius / forwardUnit * 0.01)

            # 若計算值小於 0, 驅動力就設為 0, 不做向後退動作 (有時直徑會超過畫面寬度)
            forwardForce = 0.0 if forwardForce < 0.0 else forwardForce

            # 計算物件是否在中心區域( 物件中心-畫面中心 +- 20 點 )
            inCenterZone = True if abs(center[0]-centerX) < 20 else False
            if inCenterZone:  # 在中心區域不做旋轉
                turnForce = 0.0
            else:
                # 計算旋轉驅動力 ( (物件中心-畫面中心) / 畫面寬度 * 2 ) * 最大旋轉力量
                # (物件在最旁時除以寬度約是 0.5, 乘 2 時會是 1, 物件中心在左右時, 正好會取得左右轉所對應的正負值 )
                turnForce = ( (center[0] - centerX) / width * 2 ) * (self.maxTurnForce-0.5)
                
                # 保存旋轉力數值, 供物件跑出畫面時持續尋找
                self.direction = turnForce
            
            # 送出驅動力數值
            self.drive_robot (forwardForce, turnForce)

            self.get_logger().info(f"radius:({round(radius,2)}), Ball Center:({center}), Forward/Turn: {round(forwardForce,2)}/{round(turnForce,2)}")
        else: # 沒找到物件
            # 依保存的旋轉力值持續旋轉
            self.drive_robot (0.0, self.direction)
            
            # 慢慢減少旋轉力值, 直到 <= 0.2 時停止旋轉
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
