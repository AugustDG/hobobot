#pragma once

#include <Arduino.h>

#include "filters.h"

typedef void (*line_sensor_callback)();

struct line_sensor_config_t {
  u_int16_t pin;
  int threshold = 2048;

  line_sensor_callback callback;
  iir_moving_average_config_t *moving_average_config = nullptr;
};

class line_sensor_t {
public:
  u_int16_t pin;
  int threshold = 2048;
  bool interrupt;

  bool filtered;
  iir_moving_average_t moving_average_filter;

  line_sensor_t() = default;
  line_sensor_t(const line_sensor_config_t &config);

  int read();
  bool read_thresholded();
};