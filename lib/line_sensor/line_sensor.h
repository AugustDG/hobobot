#pragma once

#include <Arduino.h>

typedef void (*line_sensor_callback)();

struct line_sensor_t
{
    u_int16_t pin;
    bool digital;
    bool interrupt;
};

struct line_sensor_config_t
{
    u_int16_t pin;
    line_sensor_callback callback;
};

line_sensor_t *line_sensor_create(const line_sensor_config_t *config);

int line_sensor_read(const line_sensor_t *sensor);