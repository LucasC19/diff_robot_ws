#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "motor_control/i2c_device.hpp"
#include "motor_control/motor.hpp"
#include "motor_control/motor_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/string.hpp"

void DiffRobotControlNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    float demand_velocity = msg->linear.x;
    float demand_rotation = msg->angular.z;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffRobotControlNode>());
  rclcpp::shutdown();
  return 0;
}
