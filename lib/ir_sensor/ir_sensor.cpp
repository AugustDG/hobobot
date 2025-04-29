#include "ir_sensor.h"
#include "utils.hpp"

ir_sensor_t *ir_sensor_create(const ir_sensor_config_t *config)
{
    ir_sensor_t *sensor = new ir_sensor_t;
    CREATION_CHECK(sensor);

    sensor->pin = config->pin;
    sensor->digital = config->digital;
    sensor->interrupt = config->callback != nullptr;

    pinMode(sensor->pin, INPUT);
    if (sensor->interrupt)
        attachInterrupt(sensor->pin, config->callback, CHANGE);

    return sensor;
}

int ir_sensor_read(ir_sensor_t *sensor)
{
    if (sensor->digital)
        return digitalRead(sensor->pin);

    return analogRead(sensor->pin);
}