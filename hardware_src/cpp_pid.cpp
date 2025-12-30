#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"

using namespace std::chrono_literals;

float FORCE_ERROR = 2.0;

class PIDPubSubNode : public rclcpp::Node {
public :
    PIDPubSubNode()
    : Node("pid_pubsub_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/RFT_FORCE", 10,
            std::bind(&PIDPubSubNode::read_ft_data, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            500ms, std::bind(&PIDPubSubNode::send_vel_callback, this));
        RCLCPP_INFO(this->get_logger(), "PID NODE STARTED");
    }

    geometry_msgs::msg::Wrench ft_data;

private :
    void send_vel_callback() {
        auto vel_cmd = geometry_msgs::msg::Twist();
        vel_cmd = pid_control(ft_data);
        publisher_->publish(vel_cmd);
    }

    void read_ft_data(const geometry_msgs::msg::WrenchStamped::SharedPtr twist_stamp_msg) {
        ft_data = twist_stamp_msg->wrench;
    }

    geometry_msgs::msg::Twist pid_control(geometry_msgs::msg::Wrench wrench_data) {
        auto output = geometry_msgs::msg::Twist();
        
        if (std::abs(wrench_data.force.z) >= FORCE_ERROR) {
            output.linear.x = 2;
            output.angular.z = 1;
        } else if (std::abs(wrench_data.force.y) >= FORCE_ERROR) {
            output.linear.x = 2;
            output.angular.z = -1;
        } else {
            output.linear.x = 0.0;
            output.angular.z = 0.0;
        }

        return output;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDPubSubNode>());
    rclcpp::shutdown();
    return 0;
}
