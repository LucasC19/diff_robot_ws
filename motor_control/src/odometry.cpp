#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "motor_control/odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "nav_msgs/msg/odometry.hpp"

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

using namespace std::chrono_literals;

void OdometryNode::timer_callback() {

    tf2::Quaternion tf_quaternion;
    geometry_msgs::msg::Quaternion quaternion;
    tf_quaternion.setRPY(0.0, 0.0, 0.0);
    quaternion = tf2::toMsg(tf_quaternion);

    nav_msgs::msg::Odometry odom; 
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.header.stamp = this->now();
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.twist.twist.linear.x = this->linear_velocity;
    odom.twist.twist.angular.z = this->angular_velocity;
    odom.pose.pose.orientation = quaternion;

    odom_publisher->publish(odom);

}

void OdometryNode::velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
  this->linear_velocity = msg->linear.x;
  this->angular_velocity = msg->angular.z;
}


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryNode>());
  rclcpp::shutdown();
  return 0;
}
