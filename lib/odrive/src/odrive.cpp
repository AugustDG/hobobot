#include "hb_odrive/odrive.h"

#include <ODriveCAN.h>
#include <ODriveFlexCAN.hpp>
#include "utils.hpp"

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
            {
                Serial.printf("ODrive(%d) heartbeat received\n", odrives_left[i]->node_id);

                odrives_left.erase(odrives_left.begin() + i);
            }
        }
    }

    return odrives_left.empty();
}

bool wait_for_heartbeat_all(uint32_t timeout)
{
    return wait_for_heartbeat(odrives, timeout);
}

void stop_all()
{
    for (auto &&odrive : odrives)
        odrive->stop();
}

void set_position_all(const std::vector<float> &positions, const std::vector<float> &ff_velocities, const std::vector<float> &ff_torques)
{
    if (positions.size() != odrives.size() || ff_velocities.size() != odrives.size() || ff_torques.size() != odrives.size())
    {
        Serial.println("Error: size of positions, ff_velocities, and ff_torques must match the number of ODrives.");
        return;
    }

    for (size_t i = 0; i < odrives.size(); i++)
        odrives[i]->set_position(positions[i], ff_velocities[i], ff_torques[i]);
}

void set_velocity_all(const std::vector<float> &velocities, const std::vector<float> &ff_torques)
{
    if (velocities.size() != odrives.size() || ff_torques.size() != odrives.size())
    {
        Serial.println("Error: size of velocities and ff_torques must match the number of ODrives.");
        return;
    }

    for (size_t i = 0; i < odrives.size(); i++)
        odrives[i]->set_velocity(velocities[i], ff_torques[i]);
}

void set_torque_control_all(const std::vector<float> &torques)
{
    if (torques.size() != odrives.size())
    {
        Serial.println("Error: size of torques must match the number of ODrives.");
        return;
    }

    for (size_t i = 0; i < odrives.size(); i++)
        odrives[i]->set_torque(torques[i]);
}

void odrive_can_cb(const CAN_message_t &msg)
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

    if (odrive_ptr->error_callback != nullptr && msg.Disarm_Reason & ~ODriveError::ODRIVE_ERROR_NONE)
        odrive_ptr->error_callback(odrive_ptr);
}

odrive_t::odrive_t(const odrive_config_t &config)
{
    node_id = static_cast<odrive_node_id_t>(config.node_id);
    error_callback = config.error_callback;

    odrive_can = new ODriveCAN(wrap_can_intf(interface), config.node_id);

    odrive_can->onFeedback(on_feedback, this);
    odrive_can->onStatus(on_heartbeat, this);
    odrive_can->onBusVI(on_bus_vi, this);
    odrive_can->onTorques(on_torques, this);
    odrive_can->onCurrents(on_currents, this);
    odrive_can->onError(on_error, this);

    // store a copy of the pointer to the ODrive
    odrives.push_back(this);
}

void odrive_t::save_configuration()
{
    Reboot_msg_t msg;
    msg.Action = ODriveCAN::ResetAction::SaveConfiguration;
    odrive_can->send(msg);
}

void odrive_t::reboot()
{
    Reboot_msg_t msg;
    msg.Action = ODriveCAN::ResetAction::Reboot;
    odrive_can->send(msg);
}

void odrive_t::set_idle()
{
    set_state(ODriveAxisState::AXIS_STATE_IDLE);
}

void odrive_t::set_closed_loop_control()
{
    set_state(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
}

void odrive_t::set_position_control()
{
    set_controller_mode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
}

void odrive_t::set_velocity_control()
{
    set_controller_mode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
}

void odrive_t::set_torque_control()
{
    set_controller_mode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
}

void odrive_t::set_state(ODriveAxisState requested_state)
{
    updated_heartbeat = false; // because we need to wait for a new heartbeat, the current one is old

    while (latest_heartbeat.Axis_State != requested_state)
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

void odrive_t::set_controller_mode(ODriveControlMode control_mode, ODriveInputMode input_mode)
{
    odrive_can->setControllerMode(control_mode, input_mode);

    control_mode = control_mode;

    save_configuration();
}

void odrive_t::stop()
{
    if (control_mode == ODriveControlMode::CONTROL_MODE_POSITION_CONTROL)
        set_position(latest_feedback.Pos_Estimate, 0.f, 0.f);
    else if (control_mode == ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL)
        set_velocity(0.f, 0.f);
    else if (control_mode == ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL)
        set_torque(0.f);

    set_idle();
}

void odrive_t::set_position(float position, float ff_velocity, float ff_torque)
{
    updated_feedback = false; // since we're moving the motor, whatever we have now is old

    odrive_can->setPosition(position, ff_velocity, ff_torque);
}

void odrive_t::set_velocity(float velocity, float ff_torque)
{
    updated_feedback = false; // since we're moving the motor, whatever we have now is old

    odrive_can->setVelocity(velocity, ff_torque);
}

void odrive_t::set_velocity_rad(float velocity_rad, float ff_torque)
{
    updated_feedback = false; // since we're moving the motor, whatever we have now is old

    odrive_can->setVelocity(velocity_rad / TWO_PI, ff_torque);
}

void odrive_t::set_torque(float torque)
{
    updated_feedback = false; // since we're moving the motor, whatever we have now is old

    odrive_can->setTorque(torque);
}

float odrive_t::get_position() const
{
    return latest_feedback.Pos_Estimate;
}

float odrive_t::get_velocity() const
{
    return latest_feedback.Vel_Estimate;
}

float odrive_t::get_velocity_rad() const
{
    return latest_feedback.Vel_Estimate * TWO_PI;
}

float odrive_t::get_torque() const
{
    return latest_torques.Torque_Estimate;
}

float odrive_t::get_current() const
{
    return latest_currents.Iq_Measured;
}

float odrive_t::get_bus_voltage() const
{
    return latest_bus_voltage_current.Bus_Voltage;
}

float odrive_t::get_bus_current() const
{
    return latest_bus_voltage_current.Bus_Current;
}
