#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>

typedef void (*can_callback)(const CAN_message_t &msg);

struct can_config_t {
  uint64_t baudrate = 250000;
  uint8_t rx_mb_count = 56;
  can_callback callback = nullptr;
};

static FlexCAN_T4<CAN1, RX_SIZE_1024, TX_SIZE_32> interface;

void can_setup(const can_config_t *config);

void refresh_can_events();
bool read_can_message(CAN_message_t &msg);
void print_can_message(const CAN_message_t &msg);