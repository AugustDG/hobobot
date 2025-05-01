#pragma once

#include <Arduino.h>
#include <ODriveCAN.h>
#include <vector>
#include "can.h"

enum odrive_node_id_t
{
    ODRV0_NODE_ID = 0x00,
    ODRV1_NODE_ID = 0x01,
};

struct odrive_t
{
    can_one_t *can_one;
    ODriveCAN *odrive_can;

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
};

struct odrive_config_t
{
    can_config_t *can_config;
    uint32_t node_id;
};

static std::vector<odrive_t *> odrives = std::vector<odrive_t *>();

void odrive_cb(int packet_size);
void on_heartbeat(Heartbeat_msg_t &msg, void *odrive);
void on_feedback(Get_Encoder_Estimates_msg_t &msg, void *odrive);
void on_torques(Get_Torques_msg_t &msg, void *odrive);
void on_currents(Get_Iq_msg_t &msg, void *odrive);
void on_bus_vi(Get_Bus_Voltage_Current_msg_t &msg, void *odrive);
void on_error(Get_Error_msg_t &msg, void *odrive);

odrive_t *odrive_create(const odrive_config_t *config);
void odrive_can_refresh_events();
bool odrive_can_process_message();

// setters

void set_state(odrive_t *odrive, ODriveAxisState requested_state);
void set_controller_mode(odrive_t *odrive, ODriveControlMode control_mode, ODriveInputMode input_mode);

void set_idle(odrive_t *odrive);
void set_closed_loop_control(odrive_t *odrive);

void set_position_control(odrive_t *odrive);
void set_velocity_control(odrive_t *odrive);
void set_torque_control(odrive_t *odrive);

void set_position(odrive_t *odrive, float position, float ff_velocity = 0.f, float ff_torque = 0.f);
void set_velocity(odrive_t *odrive, float velocity, float ff_torque = 0.f);

// getters

float get_position(const odrive_t *odrive);
float get_velocity(const odrive_t *odrive);
float get_torque(const odrive_t *odrive);
float get_current(const odrive_t *odrive);
float get_bus_voltage(const odrive_t *odrive);
float get_bus_current(const odrive_t *odrive);