#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>

typedef void (*can_callback)(const CAN_message_t &msg);

struct can_config_t
{
    uint64_t baudrate = 250000;
    can_callback callback = nullptr;
};

struct can_one_t
{
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> *interface;
};

can_one_t *can_one_create(const can_config_t *config);

void refresh_can_events(can_one_t *can_one);