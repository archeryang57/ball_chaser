#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/srv/drive_to_target.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class DriveBot : public rclcpp::Node
{
public:
    DriveBot() : Node("drive_bot") 
    {
        motor_command_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        service = this->create_service<robot_interfaces::srv::DriveToTarget>(
                "/ball_chaser/command_robot", 
                std::bind(&DriveBot::handle_drive_request, this, _1, _2));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_command_publisher;
    rclcpp::Service<robot_interfaces::srv::DriveToTarget>::SharedPtr service;

    bool handle_drive_request(const robot_interfaces::srv::DriveToTarget::Request::SharedPtr req, 
            robot_interfaces::srv::DriveToTarget::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "DriveToTargetRequest received - linear_x: %1.2f, angular_Z: %1.2f", 
                req->linear_x, req->angular_z);

        geometry_msgs::msg::Twist motor_command;

        motor_command.linear.x = req->linear_x;
        motor_command.angular.z = req->angular_z;

        motor_command_publisher->publish(motor_command);

        res->msg_feedback = "Motor command received -linear_x "+ std::to_string(motor_command.linear.x) 
                + ", angular_z: "+std::to_string(motor_command.angular.z);

        return true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveBot>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

