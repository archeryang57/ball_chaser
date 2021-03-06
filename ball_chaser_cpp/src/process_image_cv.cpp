#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/drive_to_target.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>

using namespace std;
using namespace rclcpp;
using namespace robot_interfaces::srv;
using namespace sensor_msgs::msg;

class ProcessImageCV : public rclcpp::Node
{
public:
    ProcessImage() : Node("process_image_cv") 
    {
        // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
        RCLCPP_INFO(this->get_logger(), "Will Subscribe to /image_raw topic to read the image data inside the process_image_callback function");

        // ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
        sub1 = this->create_subscription<Image>("/image_raw", 10,
                    bind(
                        &ProcessImage::process_image_callback, 
                        this, 
                        placeholders::_1
                    )
                );

        RCLCPP_INFO(this->get_logger(), "After Subscribe to /image_raw topic to read the image data inside the process_image_callback function");
    }

    void drive_robot(float lin_x, float ang_z)
    {
        RCLCPP_INFO(this->get_logger(), "Driving robot lin_x:"+std::to_string(lin_x)+
                "  ang_z:"+std::to_string(ang_z));
        
        Twist motor_command;
        auto motor_command = std::make_shared<geometry_msgs::msg::Twist>();
        motor_command.linear.x = lin_x;
        motor_command.angular.z = ang_z;
        // publish message to /cmd_vel topic
        motor_command_publisher.publish(motor_command);

    }

// private:
    int right_state = 0;
    int left_state = 0;
    bool isAction = false;

    Subscription<Image>::SharedPtr sub1;
    Publisher<Twist>::SharedPtr motor_command_publisher;
    Client<DriveToTarget>::SharedPtr client;

    // This function calls the command_robot service to drive the robot in the specified direction
    // This callback function continuously executes and reads the image data
    void process_image_callback(Image::SharedPtr img)
    {
        RCLCPP_INFO(this->get_logger(), "Entering the process_image_callback function...");
        // int white_pixel = 200;
        // int black_pixel = 75;
        
        int height = img->height; // image height, that is, number of rows
        int width = img->width; // image width, that is, number of columns
        int step  = img->step;  // Full row length in bytes
        // The definition of above parameters could be found at 
        // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html


        vector<int> white_position;  // a vector to store the positions of white pixels
        int positionId; // The position ID of each white pixel we find. 
        // This will be stored in  white_position vector.

        // TODO:
        // (1) Loop through each pixel in the image and check if there's a bright white one
        int maxRed = 90;
        int minGreen = 100;  // 49;
        int maxBlue = 120;  // 109;
        float turnForce = 0.5;    // max is 2.6
        float forwardForce = 0.1; // max is 0.21

        for (int i = 0; i < height * step; i += 3) {
            positionId = i % ( width * 3 ) / 3;
            if ( img->data[i] < maxRed  && img->data[i + 1] > minGreen && img->data[i + 2] < maxBlue )
            {  // img.data[i]: Blue; img.data[i + 1]: Green; img.data[i + 2]: Red
                // Insert the positionId into white_position vector:
                white_position.push_back(positionId);
            }         
        }
        // (2) Identify if the average of these pixels falls in the left, mid, or right side of the image
        int avg;
        int sum = 0;
        int size = white_position.size();

        for(int k=0; k < size; k++){
            sum += white_position[k];            
        }

        // for test action =============
        // RCLCPP_INFO(this->get_logger(), "set 0.0 0.0");
        // sum = 300;
        // size = 60;
        // =============================

        if (size <= 20){
            // Will request a stop when there's no white ball seen by the camera
            this->drive_robot(0.0, 0.0);  // This request a stop
            // isAction = false;
            left_state = 0;
            right_state = 0;
        }
        else {
            avg = sum / size;

            // Depending on the white ball position, call the drive_bot function and pass velocities to it
            if (avg <= width / 3){
                drive_robot(0.0, turnForce);  // This request should drive my_robot left
                left_state = 1;
                right_state = 0;  
                isAction = true;  
            }
            else if (avg >= 2 * width / 3){
                drive_robot(0.0, -turnForce); // This request drives my_robot right
                left_state = 0;
                right_state = 1; 
                isAction = true;
            }
            else{
                drive_robot(forwardForce, 0.0);  // This request drives my_robot robot forward
                left_state = 0;
                right_state = 0; 
                isAction = true;
            }
        }
    }


};


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    rclcpp::init(argc, argv);
    auto node = make_shared<ProcessImageCV>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}