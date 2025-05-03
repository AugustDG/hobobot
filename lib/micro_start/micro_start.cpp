#include "micro_start.h"
#include "utils.hpp"

void micro_start_init(const micro_start_config &config, micro_start_t &sensor)
{
    sensor.pin = config.pin;
    sensor.interrupt = config.callback != nullptr;

    pinMode(sensor.pin, INPUT_PULLDOWN);

    if (sensor.interrupt)
        attachInterrupt(sensor.pin, config.callback, CHANGE);
}

bool micro_start_read(const micro_start_t &sensor)
{
    return (digitalRead(sensor.pin) == HIGH);
}
