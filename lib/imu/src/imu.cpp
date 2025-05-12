#include "imu.h"

#include "filters.h"

imu_t::imu_t(const imu_config_t &config) {
  linear_vel_alpha = config.linear_vel_alpha;
  calibration_time = config.calibration_time;
  est_g_x = g_x = config.g_x;
  est_g_y = g_y = config.g_y;
  est_g_z = g_z = config.g_z;

  linear_vel.fill(0.f);
  rotation.fill(0.f);

  if (!bmx160.begin()) {
    Serial.println(F("BMX160 initialization failed!"));
    return;
  }

  bmx160.setGyroRange(eGyroRange_2000DPS);
  bmx160.setAccelRange(eAccelRange_16G);
}

void imu_t::calibrate() {
  bmx160.getAllData(nullptr, nullptr, nullptr); // clear any data from the registers
  delay(100);                                   // wait for the sensor to refresh

  bmx160.getAllData(nullptr, &cal_gyro, &cal_accel);
  cal_accel.x = cal_accel.x - g_x;
  cal_accel.y = cal_accel.y - g_y;
  cal_accel.z = cal_accel.z - g_z;

  for (int i = 0; i < calibration_time * 200; i++) {
    update(5, true); // update the sensor data
    delay(5);
  }

  linear_vel.fill(0.f);
  rotation.fill(0.f);
}

void imu_t::update(float dt, bool update_rotation) {
  bmx160.getAllData(&magn, &gyro, &accel);

  static iir_moving_average_config_t filter_x_config = {
      .alpha = 0.05f,
      .initial_value = accel.x - cal_accel.x,
  };
  static iir_moving_average_config_t filter_y_config = {
      .alpha = 0.05f,
      .initial_value = accel.y - cal_accel.y,
  };
  static iir_moving_average_config_t filter_z_config = {
      .alpha = 0.05f,
      .initial_value = accel.z - cal_accel.z,
  };
  static iir_moving_average_t accel_x_filter(filter_x_config);
  static iir_moving_average_t accel_y_filter(filter_y_config);
  static iir_moving_average_t accel_z_filter(filter_z_config);

  // remove initial calibration offsets
  gyro.x -= cal_gyro.x;
  gyro.y -= cal_gyro.y;
  gyro.z -= cal_gyro.z;

  // remove initial calibration offsets & filter
  accel_x_filter.update(accel.z - cal_accel.z);
  accel_y_filter.update(accel.y - cal_accel.y);
  accel_z_filter.update(accel.x - cal_accel.x);
  accel.z = accel_x_filter.avg_value;
  accel.y = accel_y_filter.avg_value;
  accel.x = accel_z_filter.avg_value;

  if (update_rotation) {
    compute_rotation(dt);
  }

  compute_linear_vel(dt);
}

void imu_t::compute_rotation(float dt) {
  filter.begin(1.f / dt);
  filter.update(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, magn.x, magn.y, magn.z);

  rotation[0] = filter.getRoll();
  rotation[1] = filter.getPitch();
  rotation[2] = filter.getYaw();
}

void imu_t::compute_linear_vel(float dt) {
  // 1. low-pass to estimate gravity
  est_g_x = linear_vel_alpha * est_g_x + (1 - linear_vel_alpha) * accel.x;
  est_g_y = linear_vel_alpha * est_g_y + (1 - linear_vel_alpha) * accel.y;
  est_g_z = linear_vel_alpha * est_g_z + (1 - linear_vel_alpha) * accel.z;

  // 2. high-pass = raw - gravity
  float lin_ax = accel.x - est_g_x;
  float lin_ay = accel.y - est_g_y;
  float lin_az = accel.z - est_g_z;

  // 3. integrate
  linear_vel[0] += lin_ax * dt;
  linear_vel[1] += lin_ay * dt;
  linear_vel[2] += lin_az * dt;

  linear_vel[0] *= 0.999f;
  linear_vel[1] *= 0.999f;
  linear_vel[2] *= 0.999f;
}
