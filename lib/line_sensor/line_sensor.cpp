#include "line_sensor.h"
#include "utils.hpp"

line_sensor_t::line_sensor_t(const line_sensor_config_t &config) {
  pin = config.pin;
  interrupt = config.callback != nullptr;
  filtered = config.moving_average_config != nullptr;

  pinMode(pin, INPUT);

  if (interrupt)
    attachInterrupt(pin, config.callback, CHANGE);

  if (filtered)
    moving_average_filter = iir_moving_average_t(*config.moving_average_config);
}

int line_sensor_t::read() {
  int value = analogRead(pin);
  if (filtered) {
    moving_average_filter.update(value);
    return moving_average_filter.avg_value;
  }

  return value;
}

bool line_sensor_t::read_thresholded() { return read() > threshold; }

border_detection_t detect_border_rect(const std::array<line_sensor_t *, 4> &sensors) { return border_detection_t(); }
