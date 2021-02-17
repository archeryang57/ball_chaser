#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/srv/drive_to_target.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;
using namespace geometry_msgs::msg;
using namespace robot_interfaces::srv;

class DriveBot : public rclcpp::Node
{
public:
    DriveBot() : Node("drive_bot") 
    {
        RCLCPP_INFO(this->get_logger(), "init Drive Bot");
        service = this->create_service<DriveToTarget>(
                "/ball_chaser/command_robot", 
                std::bind(&DriveBot::handle_drive_request, this, _1, _2));
    }

private:
    rclcpp::Publisher<Twist>::SharedPtr motor_command_publisher;
    rclcpp::Service<DriveToTarget>::SharedPtr service;

    void handle_drive_request(const DriveToTarget::Request::SharedPtr req, 
            DriveToTarget::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "DriveToTargetRequest received - linear_x: %1.2f, angular_z: %1.2f", 
                req->linear_x, req->angular_z);

        Twist motor_command;
        // auto motor_command = std::make_shared<geometry_msgs::msg::Twist>();

        motor_command.linear.x = req->linear_x;
        motor_command.angular.z = req->angular_z;

        motor_command_publisher = this->create_publisher<Twist>("/cmd_vel", 10);
        motor_command_publisher->publish(motor_command);

        res->msg_feedback = "Motor command received -linear_x "+ to_string(motor_command.linear.x) 
                + ", angular_z: "+to_string(motor_command.angular.z);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = make_shared<DriveBot>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

