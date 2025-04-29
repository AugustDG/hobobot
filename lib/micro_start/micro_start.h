#pragma once

#include <Arduino.h>

typedef void (*micro_start_callback)();

struct micro_start_t
{
    u_int16_t pin;
    bool interrupt;
};

struct micro_start_config
{
    u_int16_t pin;
    micro_start_callback callback;
};

micro_start_t *micro_start_create(const micro_start_config *config);

bool micro_start_read(const micro_start_t *sensor);
