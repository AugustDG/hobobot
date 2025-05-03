#include "filters.h"

void iir_moving_average_init(const iir_moving_average_config_t &config, iir_moving_average_t &filter)
{
    filter.avg_value = config.initial_value;
    filter.alpha = config.alpha;
}

void iir_moving_average_update(iir_moving_average_t &filter, float new_value)
{
    filter.avg_value += filter.alpha * (new_value - filter.avg_value);
}

void debounce_init(const debounce_config_t &config, debounce_t &filter)
{
    filter.last_time = millis();
    filter.last_value = config.initial_value;
    filter.debounce_time = config.debounce_time;
    filter.last_value = config.initial_value;
}

void debounce_update(debounce_t &filter, bool new_value)
{
    if (new_value != filter.last_value)
    {
        filter.last_time = millis();
        filter.last_value = new_value;
    }

    // we only update the stable value after we know it's stable
    if (millis() - filter.last_time < filter.debounce_time)
        return;

    filter.stable_value = new_value;
}
