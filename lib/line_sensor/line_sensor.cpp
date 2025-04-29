#include "line_sensor.h"

line_sensor_t *line_sensor_create(const line_sensor_config_t *config)
{
    line_sensor_t *sensor = new line_sensor_t;
    if (!sensor)
    {
        printf("Failed to allocate memory for IR sensor\n");
        return nullptr;
    }

    sensor->pin = config->pin;
    sensor->interrupt = config->callback != nullptr;

    pinMode(sensor->pin, INPUT);
    if (sensor->interrupt)
        attachInterrupt(sensor->pin, config->callback, CHANGE);

    return sensor;
}

int line_sensor_read(line_sensor_t *sensor)
{
    return analogRead(sensor->pin);
}