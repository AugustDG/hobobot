#pragma once

#include <Arduino.h>

#include "filters.h"

typedef void (*ir_sensor_callback)();

struct ir_sensor_config_t {
  u_int16_t pin;

  ir_sensor_callback callback;
  debounce_config_t *debounce_config = nullptr;
};

class ir_sensor_t {
public:
  u_int16_t pin;
  bool interrupt;

  bool filtered;
  debounce_t debounce_filter;

  ir_sensor_t() = default;
  ir_sensor_t(const ir_sensor_config_t &config);

  bool read();
};