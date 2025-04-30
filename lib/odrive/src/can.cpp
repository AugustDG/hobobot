#include "can.h"
#include <utils.hpp>

can_one_t *can_one_create(const can_config_t *config)
{
    can_one_t *can_one = new can_one_t;
    CREATION_CHECK(can_one);

    can_one->interface = new FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>;
    CREATION_CHECK(can_one->interface);

    can_one->interface->begin();
    can_one->interface->setBaudRate(config->baudrate);

    return can_one;
}

void refresh_can_events(can_one_t *can_one)
{
    can_one->interface->events();
}

bool read_can_message(can_one_t *can_one, CAN_message_t *msg)
{
    return can_one->interface->read(*msg) > 0;
}
