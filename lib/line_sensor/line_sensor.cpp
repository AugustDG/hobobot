#include "line_sensor.h"
#include "utils.hpp"

void line_sensor_init(const line_sensor_config_t &config, line_sensor_t &sensor)

{
    sensor.pin = config.pin;
    sensor.interrupt = config.callback != nullptr;
    sensor.filtered = config.moving_average_config != nullptr;

    pinMode(sensor.pin, INPUT);

    if (sensor.interrupt)
        attachInterrupt(sensor.pin, config.callback, CHANGE);

    if (sensor.filtered)
        iir_moving_average_init(*config.moving_average_config, sensor.moving_average_filter);
}

int line_sensor_read(line_sensor_t &sensor)
{
    int value = analogRead(sensor.pin);
    if (sensor.filtered)
    {
        iir_moving_average_update(sensor.moving_average_filter, value);
        return sensor.moving_average_filter.avg_value;
    }

    return value;
}

bool line_sensor_read_thresholded(line_sensor_t &sensor)
{
    return line_sensor_read(sensor) > sensor.threshold;
}
