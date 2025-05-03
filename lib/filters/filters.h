#pragma once

#include <Arduino.h>

/* IIR Moving Average */

struct iir_moving_average_config_t
{
    float alpha = 0.1f;
    float initial_value;
};

struct iir_moving_average_t
{
    float alpha;
    float avg_value;
};

void iir_moving_average_init(const iir_moving_average_config_t &config, iir_moving_average_t &filter);
void iir_moving_average_update(iir_moving_average_t &filter, float new_value);

/* Debounce */

struct debounce_config_t
{
    uint32_t debounce_time = 50;
    bool initial_value = false;
};

struct debounce_t
{
    uint32_t last_time;
    bool last_value;
    bool stable_value;

    uint32_t debounce_time;
};

void debounce_init(const debounce_config_t &config, debounce_t &filter);
void debounce_update(debounce_t &filter, bool new_value);