#pragma once

#include <Arduino.h>

class time_keeper_t {
private:
  uint32_t last_time = 0;    // in milliseconds
  uint32_t current_time = 0; // in milliseconds
  uint32_t elapsed_time = 0; // in milliseconds

public:
  time_keeper_t() = default;

  void start() { last_time = millis(); }

  void update() {
    current_time = millis();
    elapsed_time = current_time - last_time;
    last_time = current_time;
  }

  uint32_t get_dt() const { return elapsed_time; }

  uint32_t get_last_time() const { return last_time; }
};