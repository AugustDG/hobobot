#pragma once

#include <Arduino.h>

#include "DFRobot_BMX160.h"

struct imu_config_t {
};

class imu_t {
private:
  DFRobot_BMX160 bmx160;

  sBmx160SensorData_t magn, gyro, accel;

  std::array<float, 3> rotation;
  std::array<float, 3> linear_vel;

public:
  imu_t() = default;
  imu_t(const imu_config_t &config);

  void update(float dt);

  const std::array<float, 3> get_rotation() const { return {rotation[0], rotation[1], rotation[2]}; }
  const std::array<float, 3> get_rotation_rad() const {
    return rotation;
  }

  const std::array<float, 3> get_angular_vel() const { return {gyro.x, gyro.y, gyro.z}; }
  const std::array<float, 3> get_angular_vel_rad() const {
    return {gyro.x, gyro.y, gyro.z};
  }

  const std::array<float, 3> get_linear_vel() const { return linear_vel; }
  const std::array<float, 3> get_accel() const { return {accel.x, accel.y, accel.z}; }
};