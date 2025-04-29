#include "odrive.h"

#include <ODriveFlexCAN.hpp>
#include "utils.hpp"

void odrive_cb(const CAN_message_t &msg)
{
    for (auto &&odrive : odrives)
        onReceive(msg, *odrive->odrive_can);
}

void on_heartbeat(Heartbeat_msg_t &msg, void *odrive)
{
    odrive_t *odrive_ptr = static_cast<odrive_t *>(odrive);
    odrive_ptr->latest_heartbeat = msg;
    odrive_ptr->updated_heartbeat = true;
}

void on_feedback(Get_Encoder_Estimates_msg_t &msg, void *odrive)
{
    odrive_t *odrive_ptr = static_cast<odrive_t *>(odrive);
    odrive_ptr->latest_feedback = msg;
    odrive_ptr->updated_feedback = true;
}

void on_torques(Get_Torques_msg_t &msg, void *odrive)
{
    odrive_t *odrive_ptr = static_cast<odrive_t *>(odrive);
    odrive_ptr->latest_torques = msg;
    odrive_ptr->updated_torques = true;
}

void on_currents(Get_Iq_msg_t &msg, void *odrive)
{
    odrive_t *odrive_ptr = static_cast<odrive_t *>(odrive);
    odrive_ptr->latest_currents = msg;
    odrive_ptr->updated_currents = true;
}

void on_bus_vi(Get_Bus_Voltage_Current_msg_t &msg, void *odrive)
{
    odrive_t *odrive_ptr = static_cast<odrive_t *>(odrive);
    odrive_ptr->latest_bus_voltage_current = msg;
    odrive_ptr->updated_bus_voltage_current = true;
}

void on_error(Get_Error_msg_t &msg, void *odrive)
{
    odrive_t *odrive_ptr = static_cast<odrive_t *>(odrive);
    odrive_ptr->latest_error = msg;
    odrive_ptr->updated_error = true;
}

odrive_t *odrive_create(const odrive_config_t *config)
{
    odrive_t *odrive = new odrive_t;
    CREATION_CHECK(odrive);

    // update CAN callback to the ODrive callback
    can_config_t can_config = *config->can_config;
    can_config.callback = odrive_cb;

    odrive->can_one = can_one_create(&can_config);
    CREATION_CHECK(odrive->can_one);

    ODriveCanIntfWrapper can_intf = wrap_can_intf(*(odrive->can_one->interface));
    odrive->odrive_can = new ODriveCAN(can_intf, config->node_id);
    CREATION_CHECK(odrive->odrive_can);

    // set the callbacks
    odrive->odrive_can->onFeedback(on_feedback, odrive);
    odrive->odrive_can->onStatus(on_heartbeat, odrive);
    odrive->odrive_can->onBusVI(on_bus_vi, odrive);
    odrive->odrive_can->onTorques(on_torques, odrive);
    odrive->odrive_can->onCurrents(on_currents, odrive);
    odrive->odrive_can->onError(on_error, odrive);

    // store a copy of the pointer to the ODrive
    odrives.push_back(odrive);

    return odrive;
}

void set_state(odrive_t *odrive, ODriveAxisState requested_state)
{
    ODriveCAN *odrive_can = odrive->odrive_can;
    odrive->updated_heartbeat = false; // because we need to wait for a new heartbeat, the current one is old

    while (odrive->latest_heartbeat.Axis_State != requested_state)
    {
        odrive_can->clearErrors();
        delay(1);
        odrive_can->setState(requested_state);

        for (int i = 0; i < 15; i++)
        {
            delay(10);
            pumpEvents(*odrive->can_one->interface);
        }
    }
}

void set_controller_mode(odrive_t *odrive, ODriveControlMode control_mode, ODriveInputMode input_mode)
{
    ODriveCAN *odrive_can = odrive->odrive_can;
    odrive->updated_heartbeat = false; // because we need to wait for a new heartbeat, the current one is old

    while (odrive->latest_heartbeat.Procedure_Result != ODriveProcedureResult::PROCEDURE_RESULT_SUCCESS)
    {
        odrive_can->clearErrors();
        delay(1);
        odrive_can->setControllerMode(control_mode, input_mode);

        // TODO: unsure if this is necessary for controller mode setting
        for (int i = 0; i < 15; i++)
        {
            delay(10);
            pumpEvents(*odrive->can_one->interface);
        }
    }
}

void set_idle(odrive_t *odrive)
{
    set_state(odrive, ODriveAxisState::AXIS_STATE_IDLE);
}

void set_closed_loop_control(odrive_t *odrive)
{
    set_state(odrive, ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
}

void set_position_control(odrive_t *odrive)
{
    set_controller_mode(odrive, ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
}

void set_velocity_control(odrive_t *odrive)
{
    set_controller_mode(odrive, ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
}

void set_torque_control(odrive_t *odrive)
{
    set_controller_mode(odrive, ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
}

void set_position(odrive_t *odrive, float position, float ff_velocity, float ff_torque)
{
    ODriveCAN *odrive_can = odrive->odrive_can;
    odrive->updated_feedback = false; // since we're moving the motor, whatever we have now is old

    odrive_can->setPosition(position, ff_velocity, ff_torque);
}

void set_velocity(odrive_t *odrive, float velocity, float ff_torque)
{
    ODriveCAN *odrive_can = odrive->odrive_can;
    odrive->updated_feedback = false; // since we're moving the motor, whatever we have now is old

    odrive_can->setVelocity(velocity, ff_torque);
}

float get_position(const odrive_t *odrive)
{
    return odrive->latest_feedback.Pos_Estimate;
}

float get_velocity(const odrive_t *odrive)
{
    return odrive->latest_feedback.Vel_Estimate;
}

float get_torque(const odrive_t *odrive)
{
    return odrive->latest_torques.Torque_Estimate;
}

float get_current(const odrive_t *odrive)
{
    return odrive->latest_currents.Iq_Measured;
}

float get_bus_voltage(const odrive_t *odrive)
{
    return odrive->latest_bus_voltage_current.Bus_Voltage;
}

float get_bus_current(const odrive_t *odrive)
{
    return odrive->latest_bus_voltage_current.Bus_Current;
}
