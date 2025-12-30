/*
    RFT Force Torque sensor. - serial communication version

    Simple stand-alone ROS node that takes data from RFT sensor
    and Publishes it to a ROS topic

    website: www.robotous.com
    e-mail: support@robotous.com

    
      - Ubuntu 20.04 LTS
      - ROS2 Foxy
*/

/*
    ver. 0.0.0, 2017.11.29 (Robotous)
    Ver. 0.0.1, 2017.12.18 (Robotous)
    ver. 1.0.1, 2024.06.17 (Yash)
    ver. 1.0.2, 2025.09.03 (Robotous)
*/
#define ROS_RFT_SERIAL_SW_VER "VER 1.0.1(Read Only)"

#include <chrono>
#include <cstdio>
#include <stdlib.h>
#include <unistd.h>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include "std_msgs/msg/bool.hpp"
#include "RFT_UART_SAMPLE.h"

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "std_srvs/srv/set_bool.hpp"
#include "example_interfaces/srv/add_two_ints.hpp" // AddTwoInts 

using namespace std::chrono_literals;

/*
    Definitions
*/
#define RFT_SERVICE_OK            (0)
#define RFT_SERVICE_RQST_TIMEOUT  (0xF0)

/*
    Global Variables
*/
CRT_RFT_UART RFT_SENSOR;

/*
    Node Class
*/
class RFTSensorSerial : public rclcpp::Node {
public:
    RFTSensorSerial() : Node("rft_sensor_serial") {

        read_fqs_ = { {200,0},{20,1},{40,2},{100,3},
                      {200,4},{333,4},{350,6},{500,7},{1000,8} };
        filter_fqs_ = { {500,1},{300,2},{200,3},{150,4},
                        {100,5},{100,6},{50,6},{40,7},{30,8},{20,9},{10,10},{5,11},
                        {3,12},{2,13},{1,14} };
        baud_params_ = { {115200,B115200},{921600,B921600},{460800,B460800},
                         {230400,B230400},{57600,B57600} };

        RCLCPP_INFO(this->get_logger(), "Initializing ROBOTOUS sensor");


        this->declare_parameter<std::string>("DEV_NAME", "/dev/ttyUSB0");
        this->declare_parameter<int>("PORT", 0);
        this->declare_parameter<int>("BAUD", 115200);
        this->declare_parameter<float>("FORCE_DIVIDER", 50.0f);
        this->declare_parameter<float>("TORQUE_DIVIDER", 2000.0f);
        this->declare_parameter<int>("FREQUENCY", 200);
        this->declare_parameter<bool>("SET_CUTOFF_FILTER", false);
        this->declare_parameter<int>("FILTER_FREQUENCY", 500);
 


        this->get_parameter("DEV_NAME", dev_name_);
        this->get_parameter("PORT", port_);
        this->get_parameter("BAUD", baud_rate_);
        this->get_parameter("FORCE_DIVIDER", force_divider_);
        this->get_parameter("TORQUE_DIVIDER", torque_divider_);
        this->get_parameter("FREQUENCY", read_freq_);
        this->get_parameter("SET_CUTOFF_FILTER", cutoff_filter_);
        this->get_parameter("FILTER_FREQUENCY", filter_freq_);
 

        RCLCPP_INFO(this->get_logger(), "RFT Serial device: %s%d", dev_name_.c_str(), port_);
        RCLCPP_INFO(this->get_logger(), "Baud-rate: %d", baud_rate_);
        RCLCPP_INFO(this->get_logger(), "Force Divider: %f", force_divider_);
        RCLCPP_INFO(this->get_logger(), "Torque Divider: %f", torque_divider_);
        RCLCPP_INFO(this->get_logger(), "Read/Publish Frequency %d", read_freq_);
        RCLCPP_INFO(this->get_logger(), "Cutoff filter: %s", cutoff_filter_ ? "True" : "False");
        RCLCPP_INFO(this->get_logger(), "Filter frequency: %d", filter_freq_);

        bias_status_ = false;

        check_params();

        bool success = init_sensor();
        if (success) {
            RCLCPP_INFO(this->get_logger(), "RFT Force/Torque Sensor ready!");
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "RFT Sensor startup error.");
            rclcpp::shutdown();
        }

        // publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("RFT_FORCE", 10);

        // service
        bias_service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_bias",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                    std::unique_lock<std::mutex> lock(com_port_mutex_);
                    if (request->data) {
                        bias_status_ = RFT_SENSOR.set_FT_Bias(1);
                    }
                    else {
                        bias_status_ = RFT_SENSOR.set_FT_Bias(0);
                    }
                    response->success = bias_status_;
                    response->message = bias_status_ ? "Bias ON" : "Bias OFF";
                    RCLCPP_INFO(this->get_logger(), "Bias service called: %s", response->message.c_str());
            });

        ft_control_service_ = this->create_service<std_srvs::srv::SetBool>(
            "ft_continuous",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                    std::unique_lock<std::mutex> lock(com_port_mutex_);
                    bool success = false;
                    if (request->data) {
                        success = RFT_SENSOR.rqst_FT_Continuous();
                        response->message = success ? "FT Output ON" : "Failed to start FT output";
                    }
                    else {
                        success = RFT_SENSOR.rqst_FT_Stop();
                        response->message = success ? "FT Output OFF" : "Failed to stop FT output";
                    }
                    response->success = success;
                    RCLCPP_INFO(this->get_logger(), "FT control service called: %s", response->message.c_str());
            });

        // filter service (AddTwoInts use)
        lpf_service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "set_filter",
            [this](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
                    std::unique_lock<std::mutex> lock(com_port_mutex_);
                    int sub_type = request->a;  // AddTwoInts�� a -> sub_type

                    if (sub_type < 0 || sub_type > 14) {
                        response->sum = -1;  // error
                        RCLCPP_ERROR(this->get_logger(), "LPF service called with invalid sub_type: %d", sub_type);
                    }
                    else {
                        if (RFT_SENSOR.set_FT_Filter_Type(1, sub_type)) {
                            response->sum = sub_type;
                            RCLCPP_INFO(this->get_logger(), "LPF set to sub_type[%d]", sub_type);
                        }
                        else {
                            response->sum = -1;
                            RCLCPP_ERROR(this->get_logger(), "Failed to set LPF to sub_type[%d]", sub_type);
                        }
                    }
            });

        // timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&RFTSensorSerial::timer_callback, this));
    }

private:

    // publish
    void timer_callback() {
        std::unique_lock<std::mutex> lock(com_port_mutex_);
        bool isSensorOk = RFT_SENSOR.readWorker();
        lock.unlock();

        if ((RFT_SENSOR.m_nCurrMode == CMD_FT_CONT) && isSensorOk) {
            auto wrench_msg = geometry_msgs::msg::WrenchStamped();
            wrench_msg.header.stamp = this->now();
            wrench_msg.wrench.force.x = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[0];
            wrench_msg.wrench.force.y = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[1];
            wrench_msg.wrench.force.z = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[2];
            wrench_msg.wrench.torque.x = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[3];
            wrench_msg.wrench.torque.y = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[4];
            wrench_msg.wrench.torque.z = RFT_SENSOR.m_RFT_IF_PACKET.m_rcvdForce[5];
            publisher_->publish(wrench_msg);
        }
    }


    bool init_sensor() {
        DWORD baud = (DWORD)baud_reg_;
        DWORD byte_size = CS8;
        BYTE port = (BYTE)port_;

        if (!RFT_SENSOR.openPort((char*)dev_name_.c_str(), port, baud, byte_size)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open interface: %s port %d", dev_name_.c_str(), port_);
            return false;
        }

        RFT_SENSOR.m_RFT_IF_PACKET.setDivider(force_divider_, torque_divider_);
        if (!RFT_SENSOR.rqst_FT_Continuous()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start continuous output.");
        }
        return true;
    }

    // check parameter
    void check_params() {
        if (baud_params_.find(baud_rate_) != baud_params_.end()) {
            baud_reg_ = baud_params_.at(baud_rate_);
        }
        else {
            baud_rate_ = B115200;
            RCLCPP_INFO(this->get_logger(), "Invalid baud rate, reset to %d", baud_rate_);
        }
    }


    std::string dev_name_;
    int port_;
    int baud_rate_;
    int baud_reg_;
    float force_divider_;
    float torque_divider_;
    int read_freq_;
    int filter_freq_;
    bool cutoff_filter_;
    bool bias_status_;
    float bias_Fx, bias_Fy, bias_Fz;
    float bias_Tx, bias_Ty, bias_Tz;

    std::map<int, int> read_fqs_;
    std::map<int, int> filter_fqs_;
    std::map<int, int> baud_params_;

    std::mutex com_port_mutex_;

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr bias_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr ft_control_service_;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr lpf_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RFTSensorSerial>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
