#include "can.h"
#include <utils.hpp>

void can_setup(const can_config_t *config)
{
    Serial.printf("Baudrate: %d\n", config->baudrate);
    Serial.printf("RX size: %d\n", RX_SIZE_256);
    Serial.printf("TX size: %d\n", TX_SIZE_16);

    interface.begin();
    // interface.setBaudRate(config->baudrate);
    // interface.setMaxMB(16);                // set the maximum number of mailboxes
    // interface.enableFIFO();                // enable FIFO mode
    // interface.enableFIFOInterrupt();       // enable FIFO interrupt
    // interface.onReceive(config->callback); // set the callback
    // interface.mailboxStatus();             // print mailbox status
}

void refresh_can_events()
{
    interface.events();
}

bool read_can_message(CAN_message_t &msg)
{
    return interface.read(msg);
}
