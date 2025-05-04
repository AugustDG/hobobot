#include "filters.h"

void iir_moving_average_t::update(float new_value)
{
    avg_value += alpha * (new_value - avg_value);
}

void debounce_t::update(bool new_value)
{
    if (new_value != last_value)
    {
        last_time = millis();
        last_value = new_value;
    }

    // we only update the stable value after we know it's stable
    if (millis() - last_time < debounce_time)
        return;

    stable_value = new_value;
}
