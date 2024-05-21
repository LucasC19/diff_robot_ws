#pragma once

#ifndef DIFF_ROBOT_CONTROL__MOTOR_HPP_
#define DIFF_ROBOT_CONTROL__MOTOR_HPP_

#include <memory>
#include <tuple>

#include "motor_control/i2c_device.hpp"

using u8 = uint8_t;
using I2CDevicePtr = std::shared_ptr<I2CDevice>;
using MotorPins = std::tuple<u8, u8, u8>;

class Motor {
 public:
  Motor() = default;
  Motor(I2CDevicePtr i2c, MotorPins pins, std::string name);
  ~Motor();
  bool trySetVelocity(double velocity);

 private:
  I2CDevicePtr i2c_;
  MotorPins pins_;
  std::string name_;
};

#endif  // JETBOT_CONTROL__MOTOR_HPP_
