#include "can.h"
#include <utils.hpp>

void can_setup(const can_config_t *config)
{
    Serial.printf("Baudrate: %d\n", config->baudrate);

    interface.begin();
    interface.setBaudRate(config->baudrate);
    interface.setMaxMB(64);

    for (int i = 0; i < config->rx_mb_count; i++)
        interface.setMB(FLEXCAN_MAILBOX(i), RX);

    for (int i = config->rx_mb_count; i < 64; i++)
        interface.setMB(FLEXCAN_MAILBOX(i), TX);

    interface.enableMBInterrupts();        // enable mailbox interrupts
    interface.onReceive(config->callback); // set the callback
    interface.mailboxStatus();             // print mailbox status
}

void refresh_can_events()
{
    interface.events();
}

bool read_can_message(CAN_message_t &msg)
{
    return interface.read(msg);
}

void print_can_message(const CAN_message_t &msg)
{
    Serial.printf("CAN message: ID: %d, len: %d, data: ", msg.id, msg.len);
    for (int i = 0; i < msg.len; i++)
    {
        Serial.printf("%02X ", msg.buf[i]);
    }
    Serial.println();
}
