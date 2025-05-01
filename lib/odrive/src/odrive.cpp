#include "odrive.h"

#include <ODriveFlexCAN.hpp>
#include "utils.hpp"

void odrive_can_cb(const CAN_message_t &msg)
{
    if (msg.id == odrives[0]->node_id)
        print_can_message(msg); // print the message if it is from the ODrive

    for (auto odrive : odrives)
        onReceive(msg, *odrive->odrive_can);
}

void on_heartbeat(Heartbeat_msg_t &msg, void *odrive)
{
    odrive_t *odrive_ptr = static_cast<odrive_t *>(odrive);
    odrive_ptr->latest_heartbeat = msg;
    odrive_ptr->updated_heartbeat = true;

    Serial.printf("ODrive(%u) heartbeat: Axis State: %d, Procedure Result: %d\n", odrive_ptr->node_id, msg.Axis_State, msg.Procedure_Result);
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

    if (odrive_ptr->error_callback != nullptr && msg.Disarm_Reason & ~ODriveError::ODRIVE_ERROR_NONE)
        odrive_ptr->error_callback(odrive_ptr);
}

odrive_t *odrive_create(const odrive_config_t *config)
{
    odrive_t *odrive = new odrive_t;
    CREATION_CHECK(odrive);
    odrive->node_id = static_cast<odrive_node_id_t>(config->node_id);
    odrive->error_callback = config->error_callback;

    odrive->odrive_can = new ODriveCAN(wrap_can_intf(interface), config->node_id);
    CREATION_CHECK(odrive->odrive_can);

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

bool wait_for_heartbeat_all(uint32_t timeout)
{
    return wait_for_heartbeat(odrives, timeout);
}

bool wait_for_heartbeat(const std::vector<odrive_t *> &odrives_to_check, uint32_t timeout)
{
    elapsedMillis elapsed_time;
    auto odrives_left = odrives_to_check;

    while (elapsed_time < timeout && !odrives_left.empty())
    {
        odrive_can_refresh_events();
        delay(HEARTBEAT_WAIT_TIME);

        for (size_t i = 0; i < odrives_left.size(); i++)
        {
            if (odrives_left[i]->updated_heartbeat)
                odrives_left.erase(odrives_left.begin() + i);
        }
    }

    return odrives_left.empty();
}

void odrive_can_refresh_events()
{
    refresh_can_events();
}

bool odrive_can_process_message()
{
    odrive_can_refresh_events();

    CAN_message_t msg;
    bool has_msg = read_can_message(msg); // process the message

    if (has_msg)
        odrive_can_cb(msg); // call the ODrive callback

    return has_msg;
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

void set_state(odrive_t *odrive, ODriveAxisState requested_state)
{
    ODriveCAN *odrive_can = odrive->odrive_can;
    odrive->updated_heartbeat = false; // because we need to wait for a new heartbeat, the current one is old

    while (odrive->latest_heartbeat.Axis_State != requested_state)
    {
        odrive_can->clearErrors();
        delay(CLEAR_ERRORS_DELAY);
        odrive_can->setState(requested_state);

        for (int i = 0; i < 15; i++)
        {
            delay(STATE_MODE_SET_DELAY);
            odrive_can_refresh_events();
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
        delay(CLEAR_ERRORS_DELAY);
        odrive_can->setControllerMode(control_mode, input_mode);

        for (int i = 0; i < 15; i++)
        {
            delay(STATE_MODE_SET_DELAY);
            odrive_can_refresh_events();
        }
    }

    odrive->control_mode = control_mode;
}

void e_stop(odrive_t *odrive)
{
    if (odrive->control_mode == ODriveControlMode::CONTROL_MODE_POSITION_CONTROL)
        set_position(odrive, odrive->latest_feedback.Pos_Estimate, 0.f, 0.f);
    else if (odrive->control_mode == ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL)
        set_velocity(odrive, 0.f, 0.f);
    else if (odrive->control_mode == ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL)
        set_torque(odrive, 0.f);

    set_idle(odrive);
}

void e_stop_all()
{
    for (size_t i = 0; i < odrives.size(); i++)
        e_stop(odrives[i]);
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

void set_torque(odrive_t *odrive, float torque)
{
    ODriveCAN *odrive_can = odrive->odrive_can;
    odrive->updated_feedback = false; // since we're moving the motor, whatever we have now is old

    odrive_can->setTorque(torque);
}

void set_position_all(const std::vector<float> &positions, const std::vector<float> &ff_velocities, const std::vector<float> &ff_torques)
{
    for (size_t i = 0; i < odrives.size(); i++)
        set_position(odrives[i], positions[i], ff_velocities[i], ff_torques[i]);
}

void set_velocity_all(const std::vector<float> &velocities, const std::vector<float> &ff_torques)
{
    for (size_t i = 0; i < odrives.size(); i++)
        set_velocity(odrives[i], velocities[i], ff_torques[i]);
}

void set_torque_control_all(const std::vector<float> &torques)
{
    for (size_t i = 0; i < odrives.size(); i++)
        set_torque(odrives[i], torques[i]);
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
