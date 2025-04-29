#include "line_sensor.h"
#include "utils.hpp"

line_sensor_t *line_sensor_create(const line_sensor_config_t *config)
{
    line_sensor_t *sensor = new line_sensor_t;
    CREATION_CHECK(sensor);

    sensor->pin = config->pin;
    sensor->interrupt = config->callback != nullptr;

    pinMode(sensor->pin, INPUT);
    if (sensor->interrupt)
        attachInterrupt(sensor->pin, config->callback, CHANGE);

    return sensor;
}

int line_sensor_read(const line_sensor_t *sensor)
{
    return analogRead(sensor->pin);
}