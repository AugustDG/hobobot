#pragma once

#include <Arduino.h>

#include "MadgwickAHRS.h"
#include "vendor/BMX160/DFRobot_BMX160.h"

struct imu_config_t {
  float linear_vel_alpha = 0.9f; // low-pass filter for linear velocity
  float calibration_time = 7.5f; // time to calibrate in seconds

  float g_x = 0.f;
  float g_y = 0.f;
  float g_z = DEFAULT_GRAVITY_STANDARD;
};

class imu_t {
private:
  float linear_vel_alpha;
  float calibration_time;

  float g_x, g_y, g_z;
  float est_g_x, est_g_y, est_g_z;

  Madgwick filter;
  DFRobot_BMX160 bmx160;

  sBmx160SensorData_t cal_gyro, cal_accel;
  sBmx160SensorData_t magn, gyro, accel;

  std::array<float, 3> rotation;
  std::array<float, 3> linear_vel;

public:
  imu_t() = default;
  imu_t(const imu_config_t &config);

  void calibrate();
  void update(float dt, bool update_rotation = false);

  const std::array<float, 3> get_rotation() const { return {rotation[0], rotation[1], rotation[2]}; }
  const std::array<float, 3> get_rotation_rad() const { return rotation; }

  const std::array<float, 3> get_angular_vel() const { return {gyro.x, gyro.y, gyro.z}; }
  const std::array<float, 3> get_angular_vel_rad() const {
    return {gyro.x * float(DEG_TO_RAD), gyro.y * float(DEG_TO_RAD), gyro.z * float(DEG_TO_RAD)};
  }

  const std::array<float, 3> get_linear_vel() const { return linear_vel; }
  const std::array<float, 3> get_accel() const { return {accel.x, accel.y, accel.z}; }

private:
  void compute_rotation(float dt);
  void compute_linear_vel(float dt);
};