#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/drive_to_target.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <iostream>
#include <algorithm>
#include <vector>

#include <math.h>
using namespace std;
using namespace rclcpp;
using namespace robot_interfaces::srv;
using namespace sensor_msgs::msg;

class EstimateColor: public Node {
public:
    EstimateColor(): Node("estimate_color") {
        // ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
        RCLCPP_INFO(this->get_logger(), "Will Subscribe to /image_raw topic for estimate color");

        sub1 = this->create_subscription<Image>("/image_raw", 10,
                    bind(
                        &EstimateColor::estimate_color_callback, 
                        this, 
                        placeholders::_1
                    )
                );

        RCLCPP_INFO(this->get_logger(), "Estimate color finished.");
    }

private:
    Subscription<Image>::SharedPtr sub1;
    double gaussian(double mu, double sigma2, double x)  // Gaussian Function
    {
        //Use mu, sigma2 (sigma squared), and x to code the 1-dimensional Gaussian
        double prob = 1.0 / sqrt(2.0 * M_PI * sigma2) * exp(-0.5 * pow((x - mu), 2.0) / sigma2);
        return prob;
    }

    double gaussianN(double mu, double sigma2, double x)  // Normalized Gaussian Function
    {
        //Use mu, sigma2 (sigma squared), and x to code the 1-dimensional Gaussian
        double prob = gaussian(mu,sigma2,x)/gaussian(mu,sigma2,mu);
        return prob;
    }

    // This callback function continuously executes and reads the image data
    void estimate_color_callback(Image::SharedPtr img)
    {

        int height = img->height; // image height, that is, number of rows
        int width = img->width; // image width, that is, number of columns
        int step  = img->step;  // Full row length in bytes

        vector<int> red;  // a vector to store the value of red component
        vector<int> green;  // a vector to store the value of green component
        vector<int> blue;  // a vector to store the value of green component

        int positionId; 
        

        for (int i = 0; i < height * step; i += 3) {
            positionId = i % ( width * 3 ) / 3;
            if ( (positionId >= width / 3) && (positionId <= 2 * width / 3) ) {
                blue.push_back(img->data[i]);
                green.push_back(img->data[i+1]);
                red.push_back(img->data[i+2]);
            }         
        }

        int avg_red;
        int avg_green;
        int avg_blue;
        int sum_red = 0;
        int sum_green = 0;
        int sum_blue = 0;

        int ssum_red = 0; //squared sum
        int ssum_green = 0;
        int ssum_blue = 0;

        double savg_red = 0; //squared average
        double savg_green = 0;
        double savg_blue = 0;

        double sig2_red = 0;   // variance of red component
        double sig2_green = 0;
        double sig2_blue = 0;


        int size = red.size();

        for(int k=0; k < size; k++){
            sum_red += red[k];  
            sum_green += green[k];
            sum_blue += blue[k];         

            ssum_red += red[k] * red[k];
            ssum_green += green[k] * green[k];
            ssum_blue += blue[k] * blue[k];  
        }
        avg_red = sum_red / size;
        avg_green = sum_green / size;
        avg_blue = sum_blue / size;

        savg_red = ssum_red / size;
        savg_green = ssum_green / size;
        savg_blue = ssum_blue / size;

        sig2_red = savg_red - avg_red * avg_red;
        sig2_green = savg_green - avg_green * avg_green;
        sig2_blue = savg_blue - avg_blue * avg_blue;

        RCLCPP_INFO(this->get_logger(), "red= %d %f, green= %d %f, blue= %d %f", avg_red, sig2_red, avg_green, sig2_green, avg_blue, sig2_blue);
        
    }
};


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    init(argc, argv);
    auto node = std::make_shared<EstimateColor>();
    spin(node);
    shutdown();

    return 0;
}
