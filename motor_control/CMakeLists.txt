cmake_minimum_required(VERSION 3.8)
project(motor_control)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2  REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(pcl_conversions REQUIRED)

add_executable(motor_control
  src/motor_control.cpp
  src/i2c_device.cpp
  src/motor.cpp
)
target_include_directories(motor_control PUBLIC include)
ament_target_dependencies(motor_control rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(odometry
  src/odometry.cpp
)
target_include_directories(odometry PUBLIC include)
ament_target_dependencies(odometry rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
  motor_control
  odometry
  DESTINATION lib/${PROJECT_NAME})

ament_package()
