#include "imu.h"

imu_t::imu_t(const imu_config_t &config)
{
    gyro_scale = config.gyro_scale;
    accel_scale = config.accel_scale;
    mag_scale = config.mag_scale;
}

const std::array<float, 3> imu_t::get_linear_vel()
{
    return std::array<float, 3>();
}

const std::array<float, 3> imu_t::get_angular_vel()
{
    return std::array<float, 3>();
}
