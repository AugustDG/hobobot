#include "ir_sensor.h"
#include "utils.hpp"

void ir_sensor_init(const ir_sensor_config_t &config, ir_sensor_t &sensor)
{
    sensor.pin = config.pin;
    sensor.interrupt = config.callback != nullptr;
    sensor.filtered = config.debounce_config != nullptr;

    pinMode(sensor.pin, INPUT);

    if (sensor.interrupt)
        attachInterrupt(sensor.pin, config.callback, CHANGE);

    if (sensor.filtered)
        debounce_init(*config.debounce_config, sensor.debounce_filter);
}

bool ir_sensor_read(ir_sensor_t &sensor)
{
    bool value = digitalRead(sensor.pin) == HIGH;
    if (sensor.filtered)
    {
        debounce_update(sensor.debounce_filter, value);
        return sensor.debounce_filter.stable_value;
    }

    return value;
}