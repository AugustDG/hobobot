#pragma once

#include <Arduino.h>
#include <ESP32SJA1000.h>

typedef void (*can_callback)(int packet_size);

struct can_config_t
{
    uint64_t baudrate = 250000;
    uint16_t rx_pin = 21;
    uint16_t tx_pin = 22;
    can_callback callback = nullptr;
};

struct can_one_t
{
    ESP32SJA1000Class &interface = CAN;
};

can_one_t *can_one_create(const can_config_t *config);