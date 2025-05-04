#include "ddrive.h"

void ddrive_init(const ddrive_config_t &config, ddrive_t &drive)
{
    drive.track_width = config.track_width;
    drive.wheel_radius = config.wheel_radius;

    drive.target_linear_velocity = 0.f;
    drive.target_angular_velocity = 0.f;

    drive.linear_wheel_velocities.fill(0.f);
    drive.angular_wheel_velocities.fill(0.f);
}

void ddrive_set_target_velocity(ddrive_t &drive, float linear_velocity, float angular_velocity)
{
    drive.target_linear_velocity = linear_velocity;
    drive.target_angular_velocity = angular_velocity;

    // calculate wheel velocities
    drive.linear_wheel_velocities[0] = linear_velocity - (angular_velocity * drive.track_width / 2.f) * drive.gear_ratio;
    drive.linear_wheel_velocities[1] = linear_velocity + (angular_velocity * drive.track_width / 2.f) * drive.gear_ratio;

    // calculate angular velocities
    drive.angular_wheel_velocities[0] = (linear_velocity / drive.wheel_radius) * drive.gear_ratio;
    drive.angular_wheel_velocities[1] = (angular_velocity / drive.wheel_radius) * drive.gear_ratio;
}

float get_left_wheel_angular_velocity(const ddrive_t &drive)
{
    return drive.angular_wheel_velocities[0];
}

float get_right_wheel_angular_velocity(const ddrive_t &drive)
{
    return drive.angular_wheel_velocities[1];
}

float get_left_wheel_linear_velocity(const ddrive_t &drive)
{
    return drive.linear_wheel_velocities[0];
}

float get_right_wheel_linear_velocity(const ddrive_t &drive)
{
    return drive.linear_wheel_velocities[1];
}
