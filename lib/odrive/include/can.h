#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>

typedef void (*can_callback)(const CAN_message_t &msg);

struct can_config_t
{
    uint64_t baudrate = 250000;
    can_callback callback = nullptr;
};

static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> interface;

void can_setup(const can_config_t *config);

void refresh_can_events();
bool read_can_message(CAN_message_t &msg);