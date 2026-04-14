#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"

using namespace std::chrono_literals;

class PIDPubSubNode : public rclcpp::Node {
public :
    PIDPubSubNode()
    : Node("pid_pubsub_node")
    {
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        error_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/error", 10);
        subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/RFT_FORCE", 10,
            std::bind(&PIDPubSubNode::read_ft_data, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            100ms, std::bind(&PIDPubSubNode::send_vel_callback, this));
        RCLCPP_INFO(this->get_logger(), "PID NODE STARTED");
    }

    geometry_msgs::msg::Wrench ft_data;

private :
    void send_vel_callback() {
        auto vel_cmd = geometry_msgs::msg::Twist();
        auto wrench_error = geometry_msgs::msg::Wrench();

        vel_cmd = pid_control(ft_data);
        wrench_error.force.x = prev_error;

        twist_publisher_->publish(vel_cmd);
        error_publisher_->publish(wrench_error);
    }

    void read_ft_data(const geometry_msgs::msg::WrenchStamped::SharedPtr twist_stamp_msg) {
        ft_data = twist_stamp_msg->wrench;
    }

    geometry_msgs::msg::Twist pid_control(geometry_msgs::msg::Wrench wrench_data) {
        auto output = geometry_msgs::msg::Twist();

        float curr_error  = wrench_data.force.x - FORCE_ERROR;
        float delta_error = curr_error - prev_error;
        sum_error += curr_error * 0.1;
        sum_error = std::max(std::min(sum_error, (float)250.0), (float)-250);
        
        output.angular.x = K_P * curr_error + K_D * (delta_error / 0.1) + K_I * sum_error;
        prev_error = curr_error;

        return output;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr error_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber_;

    float prev_error = 0.0;
    float sum_error = 0.0;

    float FORCE_ERROR = 0.0;
    float K_P = -30.0;
    float K_D = 0.0;
    float K_I = 2.0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDPubSubNode>());
    rclcpp::shutdown();
    return 0;
}
