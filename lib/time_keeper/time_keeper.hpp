#pragma once

#include <Arduino.h>

class time_keeper_t {
private:
  uint32_t last_time = 0;    // in milliseconds
  uint32_t current_time = 0; // in milliseconds
  uint32_t elapsed_time = 0; // in milliseconds

public:
  time_keeper_t() = default;

  void start() { last_time = micros(); }

  void update() {
    current_time = micros();
    elapsed_time = current_time - last_time;
    last_time = current_time;
  }

  uint32_t get_dt() const { return elapsed_time; }
  uint32_t get_dt_ms() const { return elapsed_time / 1000u; }
  float get_dt_s() const { return elapsed_time / 1000000.f; }

  uint32_t get_last_time() const { return last_time; }
};