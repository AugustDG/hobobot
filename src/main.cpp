#include <Arduino.h>

#include "ir_sensor.h"
#include "line_sensor.h"
#include "micro_start.h"
#include "odrive.h"
#include "utils.hpp"

/* CONSTANTS */

const uint16_t IR_PINS[] = {0, 1, 2, 3, 4, 5, 6};
const uint16_t LINE_PINS[] = {14, 15, 16, 17, 20, 21};
const uint16_t MICRO_START_PIN = 6;

/* GLOBAL VARIABLES */

ir_sensor_t *ir_sensors[ARRAY_SIZE(IR_PINS)];
line_sensor_t *line_sensors[ARRAY_SIZE(LINE_PINS)];
micro_start_t *micro_start = nullptr;

odrive_t *o_left = nullptr;
odrive_t *o_right = nullptr;

/* VOLATILES */

volatile bool can_run = false;

/* FUNCTION DECLARATIONS */

void odrive_test();
void sensors_test();
void micro_start_cb();

/* FUNCTION DEFINITIONS */

void setup()
{
    Serial.begin(115200);
    delay(200);

    Serial.print("Starting up...\n");

    Serial.printf("Creating %d IR sensors\n", ARRAY_SIZE(ir_sensors));
    for (size_t i = 0; i < ARRAY_SIZE(ir_sensors); i++)
    {
        const ir_sensor_config_t config = {
            .pin = IR_PINS[i],
            .digital = true,
            .callback = nullptr,
        };

        ir_sensors[i] = ir_sensor_create(&config);
    }

    Serial.printf("Creating %d line sensors\n", ARRAY_SIZE(line_sensors));
    for (size_t i = 0; i < ARRAY_SIZE(line_sensors); i++)
    {
        const line_sensor_config_t config = {
            .pin = LINE_PINS[i],
            .callback = nullptr,
        };

        line_sensors[i] = line_sensor_create(&config);
    }

    Serial.printf("Creating micro start sensor\n");
    micro_start_config micro_start_config = {
        .pin = MICRO_START_PIN,
        .callback = micro_start_cb,
    };
    micro_start = micro_start_create(&micro_start_config);

    Serial.printf("Creating ODrives\n");

    can_config_t can_config = {
        .baudrate = 500000,
    };
    can_setup(&can_config);

    odrive_config_t o_left_config = {
        .node_id = ODRV0_NODE_ID,
    };
    odrive_config_t o_right_config = {
        .node_id = ODRV1_NODE_ID,
    };

    o_left = odrive_create(&o_left_config);
    o_right = odrive_create(&o_right_config);

    Serial.printf("Left ODrive ID: %d, waiting for hearbeat\n", o_left_config.node_id);
    while (!o_left->updated_heartbeat)
    {
        odrive_can_process_message();
        delay(10);
    }
    Serial.printf("Left ODrive heartbeat received!\n");

    Serial.printf("Right ODrive ID: %d, waiting for hearbeat\n", o_right_config.node_id);
    while (!o_right->updated_heartbeat)
    {
        odrive_can_process_message();
        delay(10);
    }
    Serial.printf("Right ODrive heartbeat received!\n");

    Serial.print("Enabling closed loop control...\n");
    set_closed_loop_control(o_left);
    set_closed_loop_control(o_right);

    Serial.print("Setting ODrives to closed loop control...\n");
    set_closed_loop_control(o_left);
    set_closed_loop_control(o_right);

    Serial.print("Starting up complete!\n");
}

void loop()
{
    odrive_can_process_message();

    odrive_test();
    // sensors_test();
}

void odrive_test()
{
    float ol_p = get_position(o_left);
    float ol_v = get_velocity(o_left);
    Serial.printf("Left ODrive Position: %f, Velocity: %f\n", ol_p, ol_v);

    float or_p = get_position(o_right);
    float or_v = get_velocity(o_right);
    Serial.printf("Right ODrive Position: %f, Velocity: %f\n", or_p, or_v);

    set_velocity(o_left, 10.f, 0.f);
    set_velocity(o_right, 0.f, 0.f);
    Serial.printf("Setting new position for both ODrives\n");
}

void sensors_test()
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
}

void micro_start_cb()
{
    can_run = micro_start_read(micro_start);

    Serial.printf("Micro start sensor switched to: %d\n", can_run);
}