#include "ir_sensor.h"
#include "utils.hpp"

ir_sensor_t::ir_sensor_t(const ir_sensor_config_t &config) {
  pin = config.pin;
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
