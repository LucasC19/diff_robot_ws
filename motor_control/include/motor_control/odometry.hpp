#ifndef ODOMETRY__NODE_HPP_
#define ODOMETRY__NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "motor_control/i2c_device.hpp"
#include "motor_control/motor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

class OdometryNode : public rclcpp::Node {
    public:
        OdometryNode() : Node("odometry_node") {
            odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
            velocities_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
                "robot_velocities", 10, std::bind(&OdometryNode::velocities_callback, this, std::placeholders::_1));
            timer_ = this->create_wall_timer(
                20ms, std::bind(&OdometryNode::timer_callback, this));
        }
        void calculate_odometry(std::vector<float> const &motor_velocities, float const &WHEEL_BASE);

    private:
        void timer_callback();
        void velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocities_subscriber;
        float linear_velocity {};
        float angular_velocity {};
}; 
#endif //ODOMETRY__NODE_HPP_