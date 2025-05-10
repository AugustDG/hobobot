#include "hb_ddrive/ddrive.h"

ddrive_t::ddrive_t(const ddrive_config_t &config) {
  track_width = config.track_width;
  wheel_radius = config.wheel_radius;
  gear_ratio = config.gear_ratio;

  target_linear_velocity = 0.f;
  target_angular_velocity = 0.f;

  linear_wheel_velocities.fill(0.f);
  angular_wheel_velocities.fill(0.f);
}

void ddrive_t::update(float linear_velocity, float angular_velocity) {
  target_linear_velocity = linear_velocity;
  target_angular_velocity = angular_velocity;

  // calculate wheel velocities
  linear_wheel_velocities[0] = linear_velocity - (angular_velocity * track_width / 2.f) / gear_ratio;
  linear_wheel_velocities[1] = linear_velocity + (angular_velocity * track_width / 2.f) / gear_ratio;

  // calculate angular velocities
  angular_wheel_velocities[0] = (linear_velocity / wheel_radius) / gear_ratio;
  angular_wheel_velocities[1] = (linear_velocity / wheel_radius) / gear_ratio;
}
