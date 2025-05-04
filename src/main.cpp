#include <Arduino.h>
#include <avr/sleep.h>

#include "ir_sensor.h"
#include "line_sensor.h"
#include "micro_start.h"
#include "ddrive.h"
#include "odrive.h"
#include "state_machine.h"
#include "utils.hpp"
#include "types.h"
#include "constants.h"

/* FUNCTION DECLARATIONS */

void micro_start_cb();
void odrive_error_cb(const odrive_t *odrive);

void ddrive_test();
void odrive_test();
void sensors_test();

void waiting_for_start();
void searching_for_opponent();
void moving_to_opponent();
void avoiding_opponent();
void avoiding_border();
void stopped();

/* CONSTANTS */

constexpr odrive_controller_mode_change_t controller_mode_change = NO_MODE_CHANGE;

constexpr uint16_t IR_PINS[] = {0, 1, 2, 3, 4, 5, 6};
constexpr uint16_t LINE_PINS[] = {14, 15, 16, 17, 20, 21};
constexpr uint16_t MICRO_START_PIN = 6;

constexpr sumo_state_t INITIAL_STATE = WAITING_FOR_START;
const std::vector<uint32_t> STATES = {
    WAITING_FOR_START,
    SEARCHING_FOR_OPPONENT,
    MOVING_TO_OPPONENT,
    // AVOIDING_OPPONENT,
    AVOIDING_BORDER,
    STOPPED,
};

const std::vector<transition_t> TRANSITIONS = {
    {ALL_STATES_NO_SELF, STOPPED},
    {WAITING_FOR_START, SEARCHING_FOR_OPPONENT},
    {SEARCHING_FOR_OPPONENT, MOVING_TO_OPPONENT},
    {SEARCHING_FOR_OPPONENT, AVOIDING_BORDER},
    {MOVING_TO_OPPONENT, SEARCHING_FOR_OPPONENT},
    {MOVING_TO_OPPONENT, AVOIDING_BORDER},
    {AVOIDING_BORDER, SEARCHING_FOR_OPPONENT},
    {STOPPED, WAITING_FOR_START},
};

/* GLOBAL VARIABLES */

ir_sensor_t ir_sensors[ARRAY_SIZE(IR_PINS)];
line_sensor_t line_sensors[ARRAY_SIZE(LINE_PINS)];
micro_start_t micro_start;

odrive_t o_left;
odrive_t o_right;
ddrive_t ddrive;

state_machine_t state_machine;

/* VOLATILES */

volatile bool match_started = true;
volatile bool odrive_errored = false;

/* FUNCTION DEFINITIONS */

void setup()
{
    Serial.begin(115200);
    delay(200);

    Serial.print("Starting up...\n");

    debounce_config_t debounce_config = {
        .debounce_time = 50,
        .initial_value = false,
    };

    Serial.printf("Creating %d IR sensors\n", ARRAY_SIZE(ir_sensors));
    for (size_t i = 0; i < ARRAY_SIZE(ir_sensors); i++)
    {
        const ir_sensor_config_t config = {
            .pin = IR_PINS[i],
            .callback = nullptr,
            .debounce_config = &debounce_config,
        };

        ir_sensor_init(config, ir_sensors[i]);
    }

    iir_moving_average_config_t moving_average_config = {
        .alpha = 0.1f,
        .initial_value = 2048.f,
    };

    Serial.printf("Creating %d line sensors\n", ARRAY_SIZE(line_sensors));
    for (size_t i = 0; i < ARRAY_SIZE(line_sensors); i++)
    {
        const line_sensor_config_t config = {
            .pin = LINE_PINS[i],
            .callback = nullptr,
            .moving_average_config = &moving_average_config,
        };

        line_sensor_init(config, line_sensors[i]);
    }

    Serial.printf("Creating micro start sensor\n");
    micro_start_config micro_start_config = {
        .pin = MICRO_START_PIN,
        .callback = micro_start_cb,
    };
    micro_start_init(micro_start_config, micro_start);

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

    odrive_init(o_left_config, o_left);
    odrive_init(o_right_config, o_right);

    do
        Serial.print("Waiting for ODrive heartbeats...\n");
    while (!wait_for_heartbeats_all(3000));

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
        while (!wait_for_heartbeats_all(3000));
    }

    Serial.print("Enabling ODrive closed loop control...\n");
    set_closed_loop_control(o_left);
    set_closed_loop_control(o_right);

    Serial.print("Creating Differential Drive...\n");
    ddrive_config_t ddrive_config = {
        .track_width = TRACK_WIDTH,
        .wheel_radius = WHEEL_RADIUS,
        .gear_ratio = GEAR_RATIO,
    };
    ddrive_init(ddrive_config, ddrive);

    Serial.print("Creating state machine...\n");
    state_machine_config_t state_machine_config = {
        .initial_state = INITIAL_STATE,
        .states = STATES,
        .transitions = TRANSITIONS,
        .state_actions = {
            {WAITING_FOR_START, waiting_for_start},
            {SEARCHING_FOR_OPPONENT, searching_for_opponent},
            {MOVING_TO_OPPONENT, moving_to_opponent},
            {AVOIDING_BORDER, avoiding_border},
            {STOPPED, stopped},
        },
        .should_log = true,
    };

    if (!verify_state_machine_config(state_machine_config))
    {
        Serial.print("State machine config is invalid!\n");
        return;
    }
    state_machine_init(state_machine_config, state_machine);

    Serial.print("Start up complete!\n");
}

void loop()
{
    // necessary to process any CAN messages and interrupts
    odrive_can_refresh_events();

    // state_machine_loop(state_machine);

    ddrive_test();
}

/* STATE FUNCTIONS */

void waiting_for_start()
{
    Serial.print("Match started!\n");
    set_state(state_machine, SEARCHING_FOR_OPPONENT);
}

void searching_for_opponent()
{
    Serial.print("Searching for opponent...\n");
    set_state(state_machine, MOVING_TO_OPPONENT);
}

void moving_to_opponent()
{
    Serial.print("Moving to opponent...\n");
    set_state(state_machine, AVOIDING_BORDER);
}

void avoiding_opponent()
{
    Serial.print("Avoiding opponent...\n");
    set_state(state_machine, AVOIDING_BORDER);
}

void avoiding_border()
{
    Serial.print("Avoiding border...\n");
    set_state(state_machine, STOPPED);
}

void stopped()
{
    Serial.print("Stopped...\n");
    set_state(state_machine, WAITING_FOR_START);
}

/* CALLBACKS */

void micro_start_cb()
{
    match_started = micro_start_read(micro_start);

    Serial.printf("Micro start sensor switched to: %d\n", match_started);
}

void odrive_error_cb(const odrive_t *odrive)
{
    if (odrive->latest_error.Active_Errors & ODriveError::ODRIVE_ERROR_INITIALIZING)
        return; // ignore this error

    odrive_errored = true;

    Serial.printf("ODrive(%d) error: %d, disarm reason: %d\n", odrive->node_id, odrive->latest_error.Active_Errors, odrive->latest_error.Disarm_Reason);

    stop_all();
}

/* TESTS */

void ddrive_test()
{
    // generate a circle for the target angular velocity and keep linear velocity constant
    float linear_velocity = 0.1f;                                       // m/s
    float angular_velocity = 0.25f * sin(2.f * PI * millis() / 1000.f); // rad/s

    ddrive_set_target_velocity(ddrive, linear_velocity, angular_velocity);

    float left_wheel_velocity = get_left_wheel_angular_velocity(ddrive);
    float right_wheel_velocity = get_right_wheel_angular_velocity(ddrive);

    set_velocity(o_left, left_wheel_velocity);
    set_velocity(o_right, right_wheel_velocity);
}

void odrive_test()
{
    float ol_p = get_position(o_left);
    float ol_v = get_velocity(o_left);

    float or_p = get_position(o_right);
    float or_v = get_velocity(o_right);

    // Serial.printf("ODrive(%d) position: %f, velocity: %f\n", o_right->node_id, or_p, or_v);

    float speed = 50.f * sin(2.f * PI * millis() / 1000.f);

    set_velocity(o_left, speed, 0.f);
    set_velocity(o_right, -speed, 0.f);
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