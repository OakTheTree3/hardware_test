#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
//#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "arduino_comms.hpp"

using namespace std::chrono_literals;

class MotorNode : public rclcpp::Node {
public :
    MotorNode() 
    : Node("motor_node") 
    {
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, 
            std::bind(&MotorNode::execute_command, this, std::placeholders::_1));
        
        if (comms_.connected()) {
            comms_.disconnect();
        }
        comms_.connect("/dev/ttyACM0", 115200, 1000);

        if (comms_.connected()) {
            RCLCPP_INFO(this->get_logger(), "Ready to run!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Fail to start");
        }
    }

private :
    void execute_command(const geometry_msgs::msg::Twist twist_command) {
        comms_.set_motor_values(twist_command.angular.x);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    ArduinoComms comms_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorNode>());
    rclcpp::shutdown();
    return 0;
}