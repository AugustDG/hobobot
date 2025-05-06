#pragma once

#include <Arduino.h>

struct imu_config_t {
  float gyro_scale = 0.0f;  // Gyroscope scale in degrees per second
  float accel_scale = 0.0f; // Accelerometer scale in g (gravity)
  float mag_scale = 0.0f;   // Magnetometer scale in microteslas (uT)
};

class imu_t {
public:
  float gyro_scale = 0.0f;  // Gyroscope scale in degrees per second
  float accel_scale = 0.0f; // Accelerometer scale in g (gravity)
  float mag_scale = 0.0f;   // Magnetometer scale in microteslas (uT)

public:
  imu_t() = default;
  imu_t(const imu_config_t &config);

  const std::array<float, 3> get_linear_vel();
  const std::array<float, 3> get_angular_vel();
};