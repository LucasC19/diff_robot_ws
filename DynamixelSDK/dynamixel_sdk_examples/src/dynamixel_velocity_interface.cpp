#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_velocity.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_velocity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include <geometry_msgs/msg/twist.hpp>
#include "dynamixel_velocity_interface.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/u2d2"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_velocity = 0;
int dxl_comm_result = COMM_TX_FAIL;


DynamixelVelocityInterface::DynamixelVelocityInterface()
: Node("dynamixel_velocity_interface")
{
    RCLCPP_INFO(this->get_logger(), "Run Dynamixel Velocity Interface node");
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    const auto QOS_RKL10V =
        rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    velocities_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", QOS_RKL10V, std::bind(&DynamixelVelocityInterface::velocities_callback, this, std::placeholders::_1));
}

void DynamixelVelocityInterface::velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received linear velocity: %f, angular velocity: %f", msg->linear.x, msg->angular.z);
    // Velocity Value of X series is 4 byte data.
    // For AX & MX(1.0) use 2 byte data(uint16_t) for the Velocity Value.
    uint32_t goal_velocity = (unsigned int)msg->linear.x;  // Convert int32 -> uint32

    // Write Goal Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(
        portHandler,
        1,
        ADDR_GOAL_VELOCITY,
        goal_velocity,
        &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("dynamixel_velocity_interface"), "Failed to write Goal Velocity.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("dynamixel_velocity_interface"), "Succeeded to write Goal Velocity.");
    }
}

DynamixelVelocityInterface::~DynamixelVelocityInterface()
{
}

void setupDynamixel(uint8_t dxl_id)
{
  // Use Velocity Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("dynamixel_velocity_interface"), "Failed to set Velocity Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("dynamixel_velocity_interface"), "Succeeded to set Velocity Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("dynamixel_velocity_interface"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("dynamixel_velocity_interface"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("dynamixel_velocity_interface"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("dynamixel_velocity_interface"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("dynamixel_velocity_interface"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("dynamixel_velocity_interface"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto dynamixel_velocity_interface = std::make_shared<DynamixelVelocityInterface>();
  rclcpp::spin(dynamixel_velocity_interface);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}