#pragma once

#include <Arduino.h>

struct ddrive_config_t
{
    float track_width;      // in meters
    float wheel_radius;     // in meters
    float gear_ratio = 1.f; // reduction ratio of motor speed to wheel speed
};

struct ddrive_t
{
    float track_width;  // in meters
    float wheel_radius; // in meters
    float gear_ratio;   // reduction ratio of motor speed to wheel speed

    float target_linear_velocity;  // in meters per second
    float target_angular_velocity; // in meters per second

    std::array<float, 2> linear_wheel_velocities;  // [left, right]; in meters per second
    std::array<float, 2> angular_wheel_velocities; // [left, right]; in radians per second
};

void ddrive_init(const ddrive_config_t &config, ddrive_t &drive);

// setters

void ddrive_set_target_velocity(ddrive_t &drive, float linear_velocity, float angular_velocity);

// getters

float get_left_wheel_angular_velocity(const ddrive_t &drive);
float get_right_wheel_angular_velocity(const ddrive_t &drive);

float get_left_wheel_linear_velocity(const ddrive_t &drive);
float get_right_wheel_linear_velocity(const ddrive_t &drive);