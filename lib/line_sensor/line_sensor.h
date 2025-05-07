#pragma once

#include <Arduino.h>

#include "filters.h"

typedef void (*line_sensor_callback)();

enum border_detection_t : uint8_t {
  NO_BORDER = 0,
  FRONT_BORDER = 1,
  FRONT_LEFT_BORDER = 2,
  LEFT_BORDER = 3,
  BACK_LEFT_BORDER = 4,
  BACK_BORDER = 5,
  BACK_RIGHT_BORDER = 6,
  RIGHT_BORDER = 7,
  FRONT_RIGHT_BORDER = 8,
};

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

border_detection_t detect_border_rect(
    const std::array<line_sensor_t *, 4>
        &sensors); // returns which side of the bot the border should be (based on the assumption that it's a rect bot);
                   // sensors should be in the order: front_left, back_left, back_right, front_right