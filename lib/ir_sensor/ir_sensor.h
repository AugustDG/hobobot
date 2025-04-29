#pragma once

#include <Arduino.h>

typedef void (*ir_sensor_callback)();

struct ir_sensor_t
{
    u_int16_t pin;
    bool digital;
    bool interrupt;
};

struct ir_sensor_config_t
{
    u_int16_t pin;
    bool digital;
    ir_sensor_callback callback;
};

ir_sensor_t *ir_sensor_create(const ir_sensor_config_t *config);

int ir_sensor_read(ir_sensor_t *sensor);