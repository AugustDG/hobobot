#include "hb_ddrive/motion_controller.h"

motion_controller_t::motion_controller_t(const motion_controller_config_t &config) {
  max_linear_vel = config.max_linear_vel;   // m/s
  max_angular_vel = config.max_angular_vel; // rad/s

  pose_xy_threshold = config.pose_xy_threshold;
  pose_theta_threshold = config.pose_theta_threshold;
  pose_alpha = config.pose_alpha;

  default_linear_gain = config.default_linear_gain;
  default_angular_gain = config.default_angular_gain;

  goal_theta_only = config.goal_theta_only;

  ddrive = config.ddrive;
  imu = config.imu;
}

void motion_controller_t::update(float dt, const std::array<float, 2> &wheel_vels) {
  update(dt, wheel_vels, default_linear_gain, default_angular_gain, default_theta_gain);
}

void motion_controller_t::update_with_linear_gain(
    float dt, const std::array<float, 2> &wheel_vels, float override_linear_gain) {
  update(dt, wheel_vels, override_linear_gain, default_angular_gain, default_theta_gain);
}

void motion_controller_t::update_with_angular_gain(
    float dt, const std::array<float, 2> &wheel_vels, float override_angular_gain) {
  update(dt, wheel_vels, default_linear_gain, override_angular_gain, default_theta_gain);
}

void motion_controller_t::update_with_theta_gain(
    float dt, const std::array<float, 2> &wheel_vels, float override_theta_gain) {
  update(dt, wheel_vels, default_linear_gain, default_angular_gain, override_theta_gain);
}

void motion_controller_t::update(
    float dt, const std::array<float, 2> &wheel_vels, float override_linear_gain, float override_angular_gain,
    float override_theta_gain)

{
  // store the current state of the wheels
  current_wheel_vels[0] = wheel_vels[0];
  current_wheel_vels[1] = wheel_vels[1];

  // update estimated velocities and pose, using the IMU and the differential drive values
  update_current_vels();
  update_current_pose(dt);

  // compute what needs to be done to reach the goal
  compute_target_vels(override_linear_gain, override_angular_gain, override_theta_gain);
  compute_target_wheel_vels();
}

void motion_controller_t::set_initial_pose(float x, float y, float theta) {
  current_pose[0] = x;
  current_pose[1] = y;
  current_pose[2] = theta;
}

void motion_controller_t::set_goal_pose(float x, float y, float theta) {
  goal_pose[0] = x;
  goal_pose[1] = y;
  goal_pose[2] = theta;
}

bool motion_controller_t::reached_goal() const {
  // check if the robot is at the goal pose
  float dx = goal_pose[0] - current_pose[0];
  float dy = goal_pose[1] - current_pose[1];
  float dtheta = goal_pose[2] - current_pose[2];

  // wrap‐to‐[-PI,PI]
  auto wrap = [](float a) {
    while (a > M_PI)
      a -= 2 * M_PI;
    while (a < -M_PI)
      a += 2 * M_PI;
    return a;
  };

  dtheta = wrap(dtheta);

  bool reached_xy = goal_theta_only || (fabs(dx) < pose_xy_threshold && fabs(dy) < pose_xy_threshold);
  bool reached_theta = fabs(dtheta) < pose_theta_threshold;

  return reached_xy && reached_theta;
}

bool motion_controller_t::pushed_detected() const {
  float ddrive_linear_vel = ddrive->get_target_linear_vel(); // m/s
  float imu_linear_vel = imu->get_linear_vel()[0]; // x is forward, and in this class we consider forward as linear vel

  float delta = fabs(ddrive_linear_vel - imu_linear_vel);
  return delta > pushed_threshold;
}

const std::array<float, 3> &motion_controller_t::get_current_pose() const { return current_pose; }

const std::array<float, 2> &motion_controller_t::get_target_wheel_vels() const { return target_wheel_vels; }

void motion_controller_t::update_current_vels() {
  float ddrive_linear_vel = (current_wheel_vels[1] + current_wheel_vels[0]) / 2.f;
  float ddrive_angular_vel = (current_wheel_vels[1] - current_wheel_vels[0]) / ddrive->track_width;

  float imu_linear_vel = imu->get_linear_vel()[0];   // x is forward
  float imu_angular_vel = imu->get_angular_vel()[2]; // z is yaw

  current_vels[0] = pose_alpha * ddrive_linear_vel + (1.f - pose_alpha) * imu_linear_vel;
  current_vels[1] = pose_alpha * ddrive_angular_vel + (1.f - pose_alpha) * imu_angular_vel;
}

void motion_controller_t::update_current_pose(float dt) {
  float displacement = current_vels[0] * dt;
  float delta_theta = current_vels[1] * dt;

  current_pose[0] += displacement * cos(current_pose[2] + delta_theta / 2.f);
  current_pose[1] += displacement * sin(current_pose[2] + delta_theta / 2.f);
  current_pose[2] += delta_theta;
}

void motion_controller_t::compute_target_vels(float linear_gain, float angular_gain, float theta_gain) {
  // current_pose = [x, y, theta]
  // goal_pose    = [x_g, y_g, theta_g]
  float dx = goal_pose[0] - current_pose[0];
  float dy = goal_pose[1] - current_pose[1];
  float rho = sqrt(dx * dx + dy * dy);

  // absolute bearing from robot to goal
  float lambda = atan2(dy, dx);

  // wrap‐to‐[-PI,PI]
  auto wrap = [](float a) {
    while (a > M_PI)
      a -= 2 * M_PI;
    while (a < -M_PI)
      a += 2 * M_PI;
    return a;
  };

  float alpha = wrap(lambda - current_pose[2]);
  float beta = wrap(goal_pose[2] - lambda);

  // un‐normalized commands
  float v_cmd = linear_gain * rho;
  float omega_cmd = angular_gain * alpha + theta_gain * beta;

  // optionally normalize by max velocities
  target_vels[0] = goal_theta_only ? 0.f : constrain(v_cmd, -max_linear_vel, max_linear_vel);
  target_vels[1] = constrain(omega_cmd, -max_angular_vel, max_angular_vel);
}

void motion_controller_t::compute_target_wheel_vels() {
  ddrive->update(target_vels[0], target_vels[1]);

  target_wheel_vels[0] = ddrive->get_left_angular_vel();
  target_wheel_vels[1] = ddrive->get_right_angular_vel();
}
