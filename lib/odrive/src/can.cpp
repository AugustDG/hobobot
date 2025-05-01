#include "can.h"
#include "ESP32SJA1000.h"
#include "utils.hpp"

can_one_t *can_one_create(const can_config_t *config)
{
    can_one_t *can_one = new can_one_t;
    CREATION_CHECK(can_one);

    can_one->interface = CAN;

    can_one->interface.setPins(config->rx_pin, config->tx_pin);
    if (!can_one->interface.begin(config->baudrate))
    {
        Serial.printf("Failed to initialize CAN interface\n");
        delete can_one;
        return nullptr;
    }
    can_one->interface.onReceive(config->callback);

    return can_one;
}