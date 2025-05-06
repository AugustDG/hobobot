#pragma once

#include <Arduino.h>

#include "filters.h"

typedef void (*ir_sensor_callback)();

struct ir_sensor_config_t {
  u_int16_t pin;
  float angle_from_center;
  float distance_from_center;

  ir_sensor_callback callback;
  debounce_config_t *debounce_config = nullptr;
};

class ir_sensor_t {
public:
  u_int16_t pin;
  float angle_from_center;
  float distance_from_center;

  bool interrupt;

  bool filtered;
  debounce_t debounce_filter;

  ir_sensor_t() = default;
  ir_sensor_t(const ir_sensor_config_t &config);

  bool read();
};

bool detect_object(const std::vector<ir_sensor_t *> &sensors, float &found_angle);

float triangulate_distance(const std::vector<ir_sensor_t *> &sensors);