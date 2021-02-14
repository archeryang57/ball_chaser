#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image

class EstimateColor(Node):
    right_state = 0
    left_state = 0
    
    def __init__(self):
        super().__init__("estimate_color")
        self.get_logger().info("Hello estimate_color")

        # * 建立 camera Subscription
        self.create_subscription(Image, "/camera/image", self.estimate_color_callback, 10)

    def gaussian(self, mu: float, sigma2: float, x: float):  # Gaussian Function
        # Use mu, sigma2 (sigma squared), and x to code the 1-dimensional Gaussian
        prob = 1.0 / math.sqrt(2.0 * math.pi * sigma2) * math.exp(-0.5 * pow((x - mu), 2.0) / sigma2)
        return prob

    def gaussianN(self, mu, sigma2, x):  # Normalized Gaussian Function
        #Use mu, sigma2 (sigma squared), and x to code the 1-dimensional Gaussian
        prob = self.gaussian(mu,sigma2,x)/self.gaussian(mu,sigma2,mu)
        return prob

    #double This callback function continuously executes and reads the image data
    def estimate_color_callback( self, img: Image):
        height = img.height; # image height, that is, number of rows
        width = img.width; # image width, that is, number of columns
        step  = img.step;  # Full row length in bytes

        red = [] # a vector to store the value of red component
        green = [];  # a vector to store the value of green component
        blue = [];  # a vector to store the value of green component

        positionId = 0
        

        for i in range(0, height * step, 3):
            positionId = i % ( width * 3 ) / 3
            if (positionId >= width / 3) and (positionId <= 2 * width / 3):
                blue.append(img.data[i])
                green.append(img.data[i+1])
                red.append(img.data[i+2])

        avg_red = 0
        avg_green = 0
        avg_blue = 0
        sum_red = 0
        sum_green = 0
        sum_blue = 0

        ssum_red = 0
        ssum_green = 0
        ssum_blue = 0

        savg_red = 0
        savg_green = 0
        savg_blue = 0

        sig2_red = 0
        sig2_green = 0
        sig2_blue = 0


        size = len(red)

        for k in range(0, size):
            sum_red += red[k]
            sum_green += green[k]
            sum_blue += blue[k]        

            ssum_red += red[k] * red[k]
            ssum_green += green[k] * green[k]
            ssum_blue += blue[k] * blue[k]

        avg_red = sum_red / size
        avg_green = sum_green / size
        avg_blue = sum_blue / size

        savg_red = ssum_red / size
        savg_green = ssum_green / size
        savg_blue = ssum_blue / size

        # sig2_red = savg_red - avg_red * avg_red
        # sig2_green = savg_green - avg_green * avg_green
        # sig2_blue = savg_blue - avg_blue * avg_blue

        self.get_logger().info( f"red= {avg_red:.2f}, green= {avg_green:.2f}, blue= {avg_blue:.2f}"  )


def main(args=None):
    #  初始化 ROS
    rclpy.init(args=args)
    #  建立 Node
    node = EstimateColor()

    #  Spin Node 以持續運行Node工作(執行 callback function)
    rclpy.spin(node)
    #  Shutdown ROS  
    rclpy.shutdown()

if __name__=='__main__':
    main()