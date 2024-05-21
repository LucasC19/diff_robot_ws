#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "motor_control/i2c_device.hpp"
#include "motor_control/motor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/string.hpp"

class DiffRobotControlNode : public rclcpp::Node {
    public:
        DiffRobotControlNode() : Node("diff_control_node") {
            device_ptr_ = std::make_shared<I2CDevice>();
            motor_1_ = Motor(device_ptr_, std::make_tuple(2, 3, 4), "2");
            motor_2_ = Motor(device_ptr_, std::make_tuple(8, 10, 9), "1");
            WHEEL_BASE = 0.1;
            WHEEL_RADIUS = 0.03;
            MAX_WHEEL_VELOCITY_RPM = 200.0;
            MAX_WHEEL_VELOCITY_LINEAR = (MAX_WHEEL_VELOCITY_RPM / 60) * 2 * 3.14 * WHEEL_RADIUS;
            subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_out", 10, std::bind(&DiffRobotControlNode::cmd_vel_callback, this, std::placeholders::_1));
        }

    private:
        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        std::vector<float> cmd_vel_to_motor(geometry_msgs::msg::Twist::SharedPtr velocity);
        void set_speed_limit(std::vector<float> &speed);
        float normalize_velocity(float const &velocity, float const &max_velocity);

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
        std::shared_ptr<I2CDevice> device_ptr_;
        Motor motor_1_;
        Motor motor_2_;
        float WHEEL_BASE {};
        float WHEEL_RADIUS {};
        float MAX_WHEEL_VELOCITY_RPM {};
        float MAX_WHEEL_VELOCITY_LINEAR {};
}; 