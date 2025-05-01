#include <Arduino.h>

#include "ir_sensor.h"
#include "line_sensor.h"
#include "micro_start.h"
#include "odrive.h"
#include "utils.hpp"
#include <avr/sleep.h>
#include <constants.h>

/* CONSTANTS */

const odrive_controller_mode_change_t controller_mode_change = NO_MODE_CHANGE;

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

volatile bool match_started = true;
volatile bool odrive_errored = false;

/* FUNCTION DECLARATIONS */

void micro_start_cb();
void odrive_error_cb(odrive_t *odrive);

void odrive_test();
void sensors_test();

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

    Serial.print("Creating ODrives\n");

    can_config_t can_config = {
        .baudrate = 500000,
        .callback = odrive_can_cb,
    };
    can_setup(&can_config);

    odrive_config_t o_left_config = {
        .node_id = ODRV0_NODE_ID,
        .error_callback = odrive_error_cb,
    };
    odrive_config_t o_right_config = {
        .node_id = ODRV1_NODE_ID,
        .error_callback = odrive_error_cb,
    };

    o_left = odrive_create(&o_left_config);
    o_right = odrive_create(&o_right_config);

    do
        Serial.print("Waiting for ODrive heartbeats...\n");
    while (!wait_for_heartbeat_all(3000));

    set_idle(o_left);
    set_idle(o_right);

    switch (controller_mode_change)
    {
    case POSITION_CONTROL:
        Serial.print("Setting ODrive to position control...\n");
        set_position_control(o_left);
        set_position_control(o_right);
        break;
    case VELOCITY_CONTROL:
        Serial.print("Setting ODrive to velocity control...\n");
        set_velocity_control(o_left);
        set_velocity_control(o_right);
        break;
    case TORQUE_CONTROL:
        Serial.print("Setting ODrive to torque control...\n");
        set_torque_control(o_left);
        set_torque_control(o_right);
        break;
    case NO_MODE_CHANGE:
    default:
        Serial.print("No controller mode change requested...\n");
        break;
    }

    if (controller_mode_change != NO_MODE_CHANGE)
    {
        // we wait again, because the ODrive will reboot after setting the control mode
        do
            Serial.print("Waiting for ODrive heartbeats...\n");
        while (!wait_for_heartbeat_all(3000));
    }

    Serial.print("Enabling closed loop control...\n");
    set_closed_loop_control(o_left);
    // set_closed_loop_control(o_right);

    Serial.print("Starting up complete!\n");
}

void loop()
{
    // something went wrong or we're not supposed to run yet
    if (!match_started || odrive_errored)
        return;

    // necessary to process any CAN messages and interrupts
    odrive_can_refresh_events();

    odrive_test();
    // sensors_test();
}

/* CALLBACKS */

void micro_start_cb()
{
    match_started = micro_start_read(micro_start);

    Serial.printf("Micro start sensor switched to: %d\n", match_started);
}

void odrive_error_cb(odrive_t *odrive)
{
    if (odrive->latest_error.Active_Errors & ODriveError::ODRIVE_ERROR_INITIALIZING)
        return; // ignore this error

    odrive_errored = true;

    Serial.printf("ODrive(%d) error: %d, disarm reason: %d\n", odrive->node_id, odrive->latest_error.Active_Errors, odrive->latest_error.Disarm_Reason);

    stop_all();
}

/* TESTS */

void odrive_test()
{
    float ol_p = get_position(o_left);
    float ol_v = get_velocity(o_left);

    float or_p = get_position(o_right);
    float or_v = get_velocity(o_right);

    // Serial.printf("ODrive(%d) position: %f, velocity: %f\n", o_right->node_id, or_p, or_v);

    set_velocity(o_left, -or_p, 0.f);
    // set_velocity(o_right, sin((double)millis()) * MINIMUM_NON_SHAKE_SPEED, 0.f);
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