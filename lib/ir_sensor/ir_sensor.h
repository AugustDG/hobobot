#pragma once

#include <Arduino.h>

#include "filters.h"

typedef void (*ir_sensor_callback)();

struct ir_sensor_t
{
    u_int16_t pin;
    bool interrupt;

    bool filtered;
    debounce_t debounce_filter;
};

struct ir_sensor_config_t
{
    u_int16_t pin;

    ir_sensor_callback callback;
    debounce_config_t *debounce_config = nullptr;
};

void ir_sensor_init(const ir_sensor_config_t &config, ir_sensor_t &sensor);

bool ir_sensor_read(ir_sensor_t &sensor);