#pragma once

#include <Arduino.h>

#include "filters.h"

typedef void (*line_sensor_callback)();

struct line_sensor_t
{
    u_int16_t pin;
    int threshold = 2048;
    bool interrupt;

    bool filtered;
    iir_moving_average_t moving_average_filter;
};

struct line_sensor_config_t
{
    u_int16_t pin;
    int threshold = 2048;

    line_sensor_callback callback;
    iir_moving_average_config_t *moving_average_config = nullptr;
};

void line_sensor_init(const line_sensor_config_t &config, line_sensor_t &sensor);

int line_sensor_read(line_sensor_t &sensor);
bool line_sensor_read_thresholded(line_sensor_t &sensor);