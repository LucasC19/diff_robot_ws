#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "motor_control/i2c_device.hpp"
#include "motor_control/motor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/string.hpp"

class DiffRobotControlNode : public rclcpp::Node {
    public:
    DiffRobotControlNode() : Node("diff_control_node") {
        device_ptr_ = std::make_shared<I2CDevice>();
        motor_1_ = Motor(device_ptr_, std::make_tuple(8, 9, 10), "1");
        motor_2_ = Motor(device_ptr_, std::make_tuple(13, 11, 12), "2");
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_out", 10, std::bind(&DiffRobotControlNode::cmd_vel_callback, this, std::placeholders::_1));
    }

    private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    std::shared_ptr<I2CDevice> device_ptr_;
    Motor motor_1_;
    Motor motor_2_;
}; 