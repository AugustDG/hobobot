#include <Arduino.h>

#include "ir_sensor.h"
#include "line_sensor.h"
#include "micro_start.h"
#include "utils.hpp"

/* CONSTANTS */

const uint16_t IR_PINS[] = {0, 1, 2, 3, 4, 5, 6};
const uint16_t LINE_PINS[] = {14, 15, 16, 17, 20, 21};
const uint16_t MICRO_START_PIN = 6;

/* GLOBAL VARIABLES */

ir_sensor_t *ir_sensors[ARRAY_SIZE(IR_PINS)];
line_sensor_t *line_sensors[ARRAY_SIZE(LINE_PINS)];
micro_start_t *micro_start = nullptr;

/* VOLATILES */

volatile bool can_run = false;

/* FUNCTION DECLARATIONS */

void micro_start_cb();

/* FUNCTION DEFINITIONS */

void setup()
{
    // initialize IR sensors
    for (size_t i = 0; i < ARRAY_SIZE(ir_sensors); i++)
    {
        const ir_sensor_config_t config = {IR_PINS[i], true, nullptr};
        ir_sensors[i] = ir_sensor_create(&config);
        CREATION_CHECK(ir_sensors[i]);
    }

    // initialize line sensors
    for (size_t i = 0; i < ARRAY_SIZE(line_sensors); i++)
    {
        const line_sensor_config_t config = {LINE_PINS[i], nullptr};
        line_sensors[i] = line_sensor_create(&config);
        CREATION_CHECK(line_sensors[i]);
    }

    micro_start_config micro_start_config = {MICRO_START_PIN, micro_start_cb};
    micro_start = micro_start_create(&micro_start_config);
    CREATION_CHECK(micro_start);
}

void loop()
{
    constexpr size_t NUM_IR_SENSORS = ARRAY_SIZE(ir_sensors);
    constexpr size_t NUM_LINE_SENSORS = ARRAY_SIZE(line_sensors);

    int ir_values[NUM_IR_SENSORS];
    int line_values[NUM_LINE_SENSORS];

    // read IR sensors
    for (size_t i = 0; i < NUM_IR_SENSORS; i++)
        ir_values[i] = ir_sensor_read(ir_sensors[i]);

    // read line sensors
    for (size_t i = 0; i < NUM_LINE_SENSORS; i++)
        line_values[i] = line_sensor_read(line_sensors[i]);

    Serial.printf("Sensor Values: ");
    for (size_t i = 0; i < NUM_IR_SENSORS; i++)
        Serial.printf("%d ", ir_values[i]);

    Serial.printf("| ");
    for (size_t i = 0; i < NUM_LINE_SENSORS; i++)
        Serial.printf("%d ", line_values[i]);

    Serial.printf("\n");

    delay(10);
}

void micro_start_cb()
{
    can_run = micro_start_read(micro_start);

    Serial.printf("Micro start sensor switched to: %d\n", can_run);
}