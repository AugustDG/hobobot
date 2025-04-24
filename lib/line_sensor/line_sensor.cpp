#include "line_sensor.h"

line_sensor *line_sensor_create(const line_sensor_config *config)
{
    line_sensor *sensor = new line_sensor;
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

int line_sensor_read(line_sensor *sensor)
{
    return analogRead(sensor->pin);
}