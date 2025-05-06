#include "ir_sensor.h"
#include "utils.hpp"

ir_sensor_t::ir_sensor_t(const ir_sensor_config_t &config) {
  pin = config.pin;
  angle_from_center = config.angle_from_center;
  distance_from_center = config.distance_from_center;
  interrupt = config.callback != nullptr;
  filtered = config.debounce_config != nullptr;

  pinMode(pin, INPUT);

  if (interrupt)
    attachInterrupt(pin, config.callback, CHANGE);

  if (filtered)
    debounce_filter = debounce_t(*config.debounce_config);
}

bool ir_sensor_t::read() {
  bool value = digitalRead(pin) == HIGH;
  if (filtered) {
    debounce_filter.update(value);
    return debounce_filter.stable_value;
  }

  return value;
}

bool detect_object(const std::vector<ir_sensor_t *> &sensors, float &found_angle) {
  uint32_t count = 0;

  for (auto &&sensor : sensors) {
    if (sensor->read()) {
      found_angle += sensor->angle_from_center;
      count++;
    }
  }

  if (count > 0)
    found_angle /= count;
  else
    found_angle = 0.f;

  return count > 0;
}
float triangulate_distance(const std::vector<ir_sensor_t> &sensors) {}
