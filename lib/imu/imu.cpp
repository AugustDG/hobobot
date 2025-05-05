#include "imu.h"

imu_t::imu_t(const imu_config_t &config)
{
    gyro_scale = config.gyro_scale;
    accel_scale = config.accel_scale;
    mag_scale = config.mag_scale;
}