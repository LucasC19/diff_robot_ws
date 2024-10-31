#ifndef DYNAMIXEL_VELOCITY_INTERFACE_NODE_HPP_
#define DYNAMIXEL_VELOCITY_INTERFACE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_velocity.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_velocity.hpp"
#include <geometry_msgs/msg/twist.hpp>


class DynamixelVelocityInterface : public rclcpp::Node
{
public:
  using SetVelocity = dynamixel_sdk_custom_interfaces::msg::SetVelocity;
  using GetVelocity = dynamixel_sdk_custom_interfaces::srv::GetVelocity;

  DynamixelVelocityInterface();
  virtual ~DynamixelVelocityInterface();

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocities_subscriber;
  //rclcpp::Service<GetVelocity>::SharedPtr get_position_server_;

  void velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  int current_velocity;
};

#endif  // DYNAMIXEL_VELOCITY_INTERFACE_NODE_HPP_
