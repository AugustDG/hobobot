#pragma once

#include <Arduino.h>

enum sumo_state_t : uint32_t
{
    WAITING_FOR_START = 0,
    SEARCHING_FOR_OPPONENT = 1,
    MOVING_TO_OPPONENT = 2,
    AVOIDING_OPPONENT = 3,
    AVOIDING_BORDER = 4,
    STOPPED = 5,
};