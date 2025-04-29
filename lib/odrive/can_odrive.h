#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>

struct odrive_can_t
{
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> *can;
};