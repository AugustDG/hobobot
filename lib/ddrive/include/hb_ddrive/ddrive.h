#pragma once

#include <Arduino.h>

struct ddrive_config_t
{
    float track_width;      // in meters
    float wheel_radius;     // in meters
    float gear_ratio = 1.f; // reduction ratio of motor speed to wheel speed
};

class ddrive_t
{
public:
    float track_width;  // in meters
    float wheel_radius; // in meters
    float gear_ratio;   // reduction ratio of motor speed to wheel speed

    float target_linear_velocity;  // in meters per second
    float target_angular_velocity; // in meters per second

private:
    std::array<float, 2> linear_wheel_velocities;  // [left, right]; in meters per second
    std::array<float, 2> angular_wheel_velocities; // [left, right]; in radians per second

public:
    ddrive_t() = default;
    ddrive_t(const ddrive_config_t &config);

    // Setters
    void update(float linear_vel, float angular_vel);

    // Getters
    float get_left_angular_vel() const;
    float get_right_angular_vel() const;

    float get_left_linear_vel() const;
    float get_right_linear_vel() const;
};