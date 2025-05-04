#pragma once

#include <Arduino.h>
#include <ODriveCAN.h>
#include "can.h"

#define STATE_MODE_SET_DELAY 10 // ms
#define CLEAR_ERRORS_DELAY 1    // ms
#define HEARTBEAT_WAIT_TIME 10  // ms

struct odrive_t;

typedef void (*odrive_error_callback_t)(const odrive_t *odrive);

enum odrive_controller_mode_change_t : uint8_t
{
    NO_MODE_CHANGE = 0,
    POSITION_CONTROL = 1,
    VELOCITY_CONTROL = 2,
    TORQUE_CONTROL = 3,
};

enum odrive_node_id_t : uint32_t
{
    ODRV0_NODE_ID = 0,
    ODRV1_NODE_ID = 1,
};

struct odrive_t
{
    ODriveCAN *odrive_can;
    odrive_node_id_t node_id;

    ODriveControlMode control_mode = ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL;

    Heartbeat_msg_t latest_heartbeat;
    bool updated_heartbeat = false;

    Get_Encoder_Estimates_msg_t latest_feedback;
    bool updated_feedback = false;

    Get_Torques_msg_t latest_torques;
    bool updated_torques = false;

    Get_Iq_msg_t latest_currents;
    bool updated_currents = false;

    Get_Bus_Voltage_Current_msg_t latest_bus_voltage_current;
    bool updated_bus_voltage_current = false;

    Get_Error_msg_t latest_error;
    bool updated_error = false;
    odrive_error_callback_t error_callback = nullptr;
};

struct odrive_config_t
{
    uint32_t node_id;
    odrive_error_callback_t error_callback = nullptr;
};

static std::vector<odrive_t *> odrives = std::vector<odrive_t *>();

void odrive_can_cb(const CAN_message_t &msg);
void on_heartbeat(Heartbeat_msg_t &msg, void *odrive);
void on_feedback(Get_Encoder_Estimates_msg_t &msg, void *odrive);
void on_torques(Get_Torques_msg_t &msg, void *odrive);
void on_currents(Get_Iq_msg_t &msg, void *odrive);
void on_bus_vi(Get_Bus_Voltage_Current_msg_t &msg, void *odrive);
void on_error(Get_Error_msg_t &msg, void *odrive);

void odrive_init(const odrive_config_t &config, odrive_t &odrive);
bool wait_for_heartbeats_all(uint32_t timeout = 1000);
bool wait_for_heartbeats(const std::vector<odrive_t *> &odrive, uint32_t timeout = 1000);
void odrive_can_refresh_events();
bool odrive_can_process_message();

void save_configuration(const odrive_t &odrive);
void reboot(const odrive_t &odrive);

// setters

void set_idle(odrive_t &odrive);
void set_closed_loop_control(odrive_t &odrive);

void set_position_control(odrive_t &odrive);
void set_velocity_control(odrive_t &odrive);
void set_torque_control(odrive_t &odrive);

void set_state(odrive_t &odrive, ODriveAxisState requested_state);
void set_controller_mode(odrive_t &odrive, ODriveControlMode control_mode, ODriveInputMode input_mode);

void stop(odrive_t &odrive);
void stop_all();

void set_position(odrive_t &odrive, float position, float ff_velocity = 0.f, float ff_torque = 0.f);
void set_velocity(odrive_t &odrive, float velocity, float ff_torque = 0.f);
void set_torque(odrive_t &odrive, float torque);

void set_position_all(const std::vector<float> &positions, const std::vector<float> &ff_velocities = {}, const std::vector<float> &ff_torques = {});
void set_velocity_all(const std::vector<float> &velocities, const std::vector<float> &ff_torques = {});
void set_torque_control_all(const std::vector<float> &torques);

// getters

float get_position(const odrive_t &odrive);
float get_velocity(const odrive_t &odrive);
float get_torque(const odrive_t &odrive);
float get_current(const odrive_t &odrive);
float get_bus_voltage(const odrive_t &odrive);
float get_bus_current(const odrive_t &odrive);