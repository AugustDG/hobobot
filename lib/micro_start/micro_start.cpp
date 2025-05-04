#include "micro_start.h"
#include "utils.hpp"

micro_start_t::micro_start_t(const micro_start_config &config)
{
    pin = config.pin;
    interrupt = config.callback != nullptr;

    pinMode(pin, INPUT_PULLDOWN);

    if (interrupt)
        attachInterrupt(pin, config.callback, CHANGE);
}

bool micro_start_t::read()
{
    return digitalRead(pin) == HIGH;
}
