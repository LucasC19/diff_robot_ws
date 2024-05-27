#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "motor_control/i2c_device.hpp"
#include "motor_control/motor.hpp"
#include "motor_control/motor_control.hpp"
#include "motor_control/odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/string.hpp"

void DiffRobotControlNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::vector<float> motor_velocity = cmd_vel_to_motor(msg);
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), std::to_string(motor_velocity[0]));
    set_speed_limit(motor_velocity);
    calculate_velocities(motor_velocity, this->WHEEL_BASE);
    motor_1_.trySetVelocity(motor_velocity[0]);
    motor_2_.trySetVelocity(motor_velocity[1]);

    std::cout << "motor 1 velocity: "<< motor_velocity[0] << "    motor 2 velocity: "<< motor_velocity[1] << std::endl;
    std::cout << "velocity: "<< msg->linear.x << std::endl;
    std::cout << "max velocity: "<< this->MAX_WHEEL_VELOCITY_LINEAR << std::endl;
    std::cout << std::endl;
}

std::vector<float> DiffRobotControlNode::cmd_vel_to_motor(geometry_msgs::msg::Twist::SharedPtr velocity) {
  
  std::vector<float> motor_velocity;

  float right_wheel_velocity = (velocity->linear.x + velocity->angular.z * this->WHEEL_BASE / 2.0);  
  float left_wheel_velocity = (velocity->linear.x - velocity->angular.z * this->WHEEL_BASE / 2.0);

  motor_velocity.push_back(normalize_velocity(right_wheel_velocity, this->MAX_WHEEL_VELOCITY_LINEAR));
  motor_velocity.push_back(normalize_velocity(left_wheel_velocity, this->MAX_WHEEL_VELOCITY_LINEAR));

  return motor_velocity;
} 

float DiffRobotControlNode::normalize_velocity(float const &velocity, float const &max_velocity) {
  return velocity/max_velocity;
}

void DiffRobotControlNode::set_speed_limit(std::vector<float> &speed) {
  float scale = std::max(std::abs(speed[0]), std::abs(speed[1]));
  if (scale > 1) {
    speed[0] /= scale;
    speed[1] /= scale;
  }
}

void DiffRobotControlNode::calculate_velocities(std::vector<float> const &motor_velocities, float const &WHEEL_BASE) {
  geometry_msgs::msg::Twist velocities;
  velocities.linear.x = (motor_velocities[0] + motor_velocities[1]) / 2;
  velocities.angular.z = (motor_velocities[0] - motor_velocities[1]) / WHEEL_BASE;
  this->velocities_publisher->publish(velocities);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffRobotControlNode>());
  rclcpp::shutdown();
  return 0;
}
