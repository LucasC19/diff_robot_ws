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

add_executable(motor_control
  src/motor_control.cpp
  src/i2c_device.cpp
  src/motor.cpp
)
target_include_directories(motor_control PUBLIC include)
ament_target_dependencies(motor_control rclcpp std_msgs)

install(TARGETS
  motor_control
  DESTINATION lib/${PROJECT_NAME})

ament_package()
