#pragma once

#include <Arduino.h>

typedef void (*micro_start_callback)();

struct micro_start_config {
  u_int16_t pin;
  micro_start_callback callback;
};

class micro_start_t {
public:
  u_int16_t pin;
  bool interrupt;

  micro_start_t() = default;
  micro_start_t(const micro_start_config &config);

  bool read();
};