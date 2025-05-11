#include "imu.h"

imu_t::imu_t(const imu_config_t &config) {
  if (!bmx160.begin()) {
    Serial.println(F("BMX160 initialization failed!"));
    return;
  }

  bmx160.setGyroRange(eGyroRange_2000DPS);
  bmx160.setAccelRange(eAccelRange_16G);
}

void imu_t::update(float dt) {
  bmx160.getAllData(&magn, &gyro, &accel);

  accel.x = accel.x / 9.8065f; // Convert to g
  accel.y = accel.y / 9.8065f; // Convert to g
  accel.z = accel.z / 9.8065f; // Convert to g

  linear_vel[0] = accel.x * dt;
  linear_vel[1] = accel.y * dt;
  linear_vel[2] = accel.z * dt;

  rotation[0] += gyro.x * dt;
  rotation[1] += gyro.y * dt;
  rotation[2] += gyro.z * dt;
}
