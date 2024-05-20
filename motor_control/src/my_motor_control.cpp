#include "motor_control/i2c_device.hpp"
#include "motor_control/motor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Diff_Robot_Control : public rclcpp::Node {
    public:
        DiffRobotControlNode() : Node("diff_control_node"), spinning_(false) {
            
        }
} 