#include <Arduino.h>
#include <avr/sleep.h>

#include "ir_sensor.h"
#include "line_sensor.h"
#include "micro_start.h"
#include "hb_ddrive/ddrive.h"
#include "hb_odrive/odrive.h"
#include "hb_ddrive/motion_controller.h"
#include "state_machine.h"
#include "time_keeper.hpp"
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

void set_wheel_vels_from_ddrive_targets(float linear_vel, float angular_vel);
void set_wheel_vels_from_mc_targets();

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
motion_controller_t motion_controller;

state_machine_t state_machine;
time_keeper_t time_keeper;

/* VOLATILES */

/* FUNCTION DEFINITIONS */

void setup()
{
    Serial.begin(115200);
    delay(200);

    Serial.print(F("Starting up...\n"));

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

        ir_sensors[i] = ir_sensor_t(config);
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

        line_sensors[i] = line_sensor_t(config);
    }

    Serial.printf(F("Creating micro start sensor\n"));
    micro_start_config micro_start_config = {
        .pin = MICRO_START_PIN,
        .callback = micro_start_cb,
    };
    micro_start = micro_start_t(micro_start_config);

    Serial.print(F("Creating ODrives\n"));

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

    o_left = odrive_t(o_left_config);
    o_right = odrive_t(o_right_config);

    do
        Serial.print(F("Waiting for ODrive heartbeats...\n"));
    while (!wait_for_heartbeat_all(3000));

    o_left.set_idle();
    o_right.set_idle();

    switch (controller_mode_change)
    {
    case POSITION_CONTROL:
        Serial.print(F("Setting ODrive to position control...\n"));
        o_left.set_position_control();
        o_right.set_position_control();
        break;
    case VELOCITY_CONTROL:
        Serial.print(F("Setting ODrive to velocity control...\n"));
        o_left.set_velocity_control();
        o_right.set_velocity_control();
        break;
    case TORQUE_CONTROL:
        Serial.print(F("Setting ODrive to torque control...\n"));
        o_left.set_torque_control();
        o_right.set_torque_control();
        break;
    case NO_MODE_CHANGE:
    default:
        Serial.print(F("No controller mode change requested...\n"));
        break;
    }

    if (controller_mode_change != NO_MODE_CHANGE)
    {
        // we wait again, because the ODrive will reboot after setting the control mode
        do
            Serial.print(F("Waiting for ODrive heartbeats...\n"));
        while (!wait_for_heartbeat_all(3000));
    }

    Serial.print(F("Enabling ODrive closed loop control...\n"));
    o_left.set_closed_loop_control();
    o_right.set_closed_loop_control();

    Serial.print(F("Creating Differential Drive...\n"));
    ddrive_config_t ddrive_config = {
        .track_width = TRACK_WIDTH,
        .wheel_radius = WHEEL_RADIUS,
        .gear_ratio = GEAR_RATIO,
    };
    ddrive = ddrive_t(ddrive_config);

    Serial.print(F("Creating motion controller...\n"));
    motion_controller_config_t motion_controller_config = {
        .max_linear_vel = MAX_LINEAR_VEL,
        .max_angular_vel = MAX_ANGULAR_VEL,
        .pose_alpha = POSE_ALPHA,
        .default_linear_gain = DEFAULT_MC_LINEAR_GAIN,
        .default_angular_gain = DEFAULT_MC_ANGULAR_GAIN,
        .default_theta_gain = DEFAULT_MC_THETA_GAIN,
        .ddrive = &ddrive,
        .imu = nullptr,
    };
    motion_controller = motion_controller_t(motion_controller_config);
    motion_controller.set_initial_pose(0.f, 0.f, 0.f);
    motion_controller.set_goal(0.f, 0.f, 0.f);

    Serial.print(F("Creating state machine...\n"));
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

    if (!state_machine_config.verify())
    {
        Serial.print(F("State machine config is invalid!\n"));
        return;
    }
    state_machine = state_machine_t(state_machine_config);

    Serial.print(F("Creating time keeper...\n"));
    time_keeper = time_keeper_t();

    Serial.print(F("Start up complete!\n"));
}

void loop()
{
    // necessary to process any CAN messages and interrupts
    odrive_can_refresh_events();

    state_machine.loop(); // state machine processing
    time_keeper.update();

    set_wheel_vels_from_mc_targets();
}

/* STATE FUNCTIONS */

void waiting_for_start()
{
    Serial.print("Match started!\n");
    state_machine.set_state(SEARCHING_FOR_OPPONENT);
}

void searching_for_opponent()
{
    Serial.print("Searching for opponent...\n");
    state_machine.set_state(MOVING_TO_OPPONENT);
}

void moving_to_opponent()
{
    Serial.print("Moving to opponent...\n");
    state_machine.set_state(AVOIDING_BORDER);
}

void avoiding_opponent()
{
    Serial.print("Avoiding opponent...\n");
    state_machine.set_state(AVOIDING_BORDER);
}

void avoiding_border()
{
    Serial.print("Avoiding border...\n");
    state_machine.set_state(STOPPED);
}

void stopped()
{
    Serial.print("Stopped...\n");
    state_machine.set_state(WAITING_FOR_START);
}

/* CALLBACKS */

void micro_start_cb()
{
    bool match_started = micro_start.read();
    state_machine.set_state(match_started ? SEARCHING_FOR_OPPONENT : STOPPED);

    Serial.printf("Micro start sensor switched to: %d\n", match_started);
}

void odrive_error_cb(const odrive_t *odrive)
{
    if (odrive->latest_error.Active_Errors & ODriveError::ODRIVE_ERROR_INITIALIZING)
        return; // ignore this error

    Serial.printf("ODrive(%d) error: %d, disarm reason: %d\n", odrive->node_id, odrive->latest_error.Active_Errors, odrive->latest_error.Disarm_Reason);

    stop_all();
}

/* TESTS */

void ddrive_test()
{
    // generate a circle for the target angular velocity and keep linear velocity constant
    float linear_velocity = 0.1f;                                       // m/s
    float angular_velocity = 0.25f * sin(2.f * PI * millis() / 1000.f); // rad/s

    set_wheel_vels_from_ddrive_targets(linear_velocity, angular_velocity);
}

void odrive_test()
{
    float ol_p = o_left.get_position();
    float ol_v = o_left.get_velocity();

    float ol_t = o_left.get_torque();
    float ol_i = o_left.get_current();

    float ol_bc = o_left.get_bus_current();
    float ol_bv = o_left.get_bus_voltage();

    Serial.printf("ODrive(%d) torque: %f, current: %f\n", o_left.node_id, ol_t, ol_i);
    Serial.printf("ODrive(%d) position: %f, velocity: %f\n", o_left.node_id, ol_p, ol_v);
    Serial.printf("ODrive(%d) bus current: %f, bus voltage: %f\n", o_left.node_id, ol_bc, ol_bv);

    float speed = 50.f * sin(2.f * PI * millis() / 1000.f);

    o_left.set_velocity(speed, 0.f);
    o_right.set_velocity(-speed, 0.f);
}

void sensors_test()
{
    constexpr size_t NUM_IR_SENSORS = ARRAY_SIZE(ir_sensors);
    constexpr size_t NUM_LINE_SENSORS = ARRAY_SIZE(line_sensors);

    int ir_values[NUM_IR_SENSORS];
    int line_values[NUM_LINE_SENSORS];

    // read IR sensors
    for (size_t i = 0; i < NUM_IR_SENSORS; i++)
        ir_values[i] = ir_sensors[i].read();

    // read line sensors
    for (size_t i = 0; i < NUM_LINE_SENSORS; i++)
        line_values[i] = line_sensors[i].read();

    Serial.printf("Sensor Values: ");
    for (size_t i = 0; i < NUM_IR_SENSORS; i++)
        Serial.printf("%d ", ir_values[i]);

    Serial.printf("| ");
    for (size_t i = 0; i < NUM_LINE_SENSORS; i++)
        Serial.printf("%d ", line_values[i]);

    Serial.printf("\n");
}

/* UTILITIES */

void set_wheel_vels_from_ddrive_targets(float linear_vel, float angular_vel)
{
    ddrive.update(linear_vel, angular_vel);

    // get the left and right wheel velocities from the differential drive
    float left_wheel_velocity = ddrive.get_left_angular_vel() / TWO_PI;
    float right_wheel_velocity = ddrive.get_right_angular_vel() / TWO_PI;

    // set the ODrive velocities based on the differential drive velocities
    o_left.set_velocity(left_wheel_velocity);
    o_right.set_velocity(right_wheel_velocity);
}

void set_wheel_vels_from_mc_targets()
{
    // get the left and right wheel velocities from the motion controller
    float left_wheel_velocity = motion_controller.get_target_wheel_vels()[0] / TWO_PI;
    float right_wheel_velocity = motion_controller.get_target_wheel_vels()[1] / TWO_PI;

    // set the ODrive velocities based on the motion controller velocities (which uses the differential drive)
    o_left.set_velocity(left_wheel_velocity);
    o_right.set_velocity(right_wheel_velocity);
}