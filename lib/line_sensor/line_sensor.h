#pragma once

#include <Arduino.h>

typedef void (*line_sensor_callback)();

struct line_sensor
{
    u_int16_t pin;
    bool digital;
    bool interrupt;
};

struct line_sensor_config
{
    u_int16_t pin;
    line_sensor_callback callback;
};

line_sensor *line_sensor_create(const line_sensor_config *config);

int line_sensor_read(line_sensor *sensor);