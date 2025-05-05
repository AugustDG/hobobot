#pragma once

#include "hb_ddrive/ddrive.h"
#include "imu.h"

struct motion_controller_config_t
{
    float max_linear_vel;  // m/s
    float max_angular_vel; // rad/s

    float pose_xy_threshold = 0.05f;    // m; how close to the goal x and y we need to be to consider it reached
    float pose_theta_threshold = 0.05f; // rad; how close to the goal theta we need to be to consider it reached
    float pose_alpha = 0.5f;            // how much to trust the algorithm vs the sensors (0.f = all sensors, 1.f = all algorithm)

    float default_linear_gain = 1.f;  // how aggressive should the robot be when moving forward
    float default_angular_gain = 2.f; // how aggressive should the robot be when turning
    float default_theta_gain = -0.5f; // how smooth is the path to the final orientation

    bool goal_theta_only = false; // if true, the mc will only move to the goal theta, not the goal x and y

    ddrive_t *ddrive = nullptr;
    imu_t *imu = nullptr;
};

class motion_controller_t
{
private:
    float max_linear_vel;  // m/s
    float max_angular_vel; // rad/s

    float pose_xy_threshold;    // m; how close to the goal x and y we need to be to consider it reached
    float pose_theta_threshold; // rad; how close to the goal theta we need to be to consider it reached
    float pose_alpha;           // how much to trust the algorithm vs the sensors (0.f = all sensors, 1.f = all algorithm)

    float default_linear_gain;  // how aggressive should the robot be when moving forward
    float default_angular_gain; // how aggressive should the robot be when turning
    float default_theta_gain;   // how smooth is the path to the final orientation

    bool goal_theta_only; // if true, the mc will only move to the goal theta, not the goal x and y

    ddrive_t *ddrive = nullptr;
    imu_t *imu = nullptr;

    std::array<float, 3> current_pose = {0.f, 0.f, 0.f}; // [x, y, theta]; in meters and radians
    std::array<float, 3> goal_pose = {0.f, 0.f, 0.f};    // [x, y, theta]; in meters and radians
    std::array<float, 2> current_vels = {0.f, 0.f};      // [linear, angular]; in rad/s
    std::array<float, 2> target_vels = {0.f, 0.f};       // [linear, angular]; in rad/s

    std::array<float, 2> current_wheel_vels = {0.f, 0.f}; // [left, right]; in rad/s
    std::array<float, 2> target_wheel_vels = {0.f, 0.f};  // [left, right]; in rad/s

public:
    motion_controller_t() = default;
    motion_controller_t(const motion_controller_config_t &config);

    void update(float dt, const std::array<float, 2> &wheel_vels);
    void update_with_linear_gain(float dt, const std::array<float, 2> &wheel_vels, float linear_gain_override);
    void update_with_angular_gain(float dt, const std::array<float, 2> &wheel_vels, float angular_gain_override);
    void update_with_theta_gain(float dt, const std::array<float, 2> &wheel_vels, float theta_gain_override);
    void update(float dt, const std::array<float, 2> &wheel_vels, float linear_gain_override, float angular_gain_override, float theta_gain_override);

    // Setters

    void set_initial_pose(float x, float y, float theta); // sets the initial pose of the robot, in meters and radians
    void set_goal_pose(float x, float y, float theta);    // sets where the robot should be at, in meters and radians

    // Getters

    bool reached_goal() const;                                   // checks if the robot is at the goal pose
    const std::array<float, 3> &get_current_pose() const;      // retrieves the current pose of the robot
    const std::array<float, 2> &get_target_wheel_vels() const; // retrieves the target wheel velocities of the robot [left, right], in rad/s

private:
    void update_current_vels();         // updates the current velocities of the robot
    void update_current_pose(float dt); // updates the current pose of the robot

    void compute_target_vels(float linear_gain, float angular_gain, float theta_gain); // updates the target velocities of the robot
    void compute_target_wheel_vels();                                                  // updates the target wheel velocities of the robot
};