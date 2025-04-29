#include "micro_start.h"
#include "utils.hpp"

micro_start_t *micro_start_create(const micro_start_config *config)
{
    micro_start_t *sensor = new micro_start_t;
    CREATION_CHECK(sensor);

    sensor->pin = config->pin;
    sensor->interrupt = config->callback != nullptr;

    pinMode(sensor->pin, INPUT_PULLDOWN);
    if (sensor->interrupt)
        attachInterrupt(sensor->pin, config->callback, CHANGE);

    return sensor;
}

bool micro_start_read(const micro_start_t *sensor)
{
    return (digitalRead(sensor->pin) == HIGH);
}
