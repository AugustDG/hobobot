#pragma once

#include <Arduino.h>

/* IIR Moving Average */

struct iir_moving_average_config_t {
  float alpha = 0.1f;
  float initial_value;
};

class iir_moving_average_t {
public:
  float alpha;
  float avg_value;

  iir_moving_average_t() = default;
  iir_moving_average_t(const iir_moving_average_config_t &config)
      : alpha(config.alpha), avg_value(config.initial_value) {}

  void update(float new_value);
};

/* Debounce */

struct debounce_config_t {
  uint32_t debounce_time = 50;
  bool initial_value = false;
};

class debounce_t {
public:
  bool last_value;
  bool stable_value;

  uint32_t debounce_time;

private:
  uint32_t last_time;

public:
  debounce_t() = default;
  debounce_t(const debounce_config_t &config)
      : last_value(config.initial_value), stable_value(config.initial_value), debounce_time(config.debounce_time),
        last_time(millis()) {}

  void update(bool new_value);
};