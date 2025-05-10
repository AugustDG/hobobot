from enum import Enum
import math
import time
import odrive
from odrive.enums import *


class Side(Enum):
    LEFT = 0
    RIGHT = 1


class Operation(Enum):
    CONFIGURE = 0
    CALIBRATE = 1


ODRIVE_SERIALS = ["396034693331", "395934593331"]


def find_odrive(side: Side):
    """
    Find the ODrive device based on the specified side (LEFT or RIGHT).
    """
    serial = ODRIVE_SERIALS[side.value]
    odrv = odrive.find_sync(serial_number=serial, timeout=5)
    if odrv is None:
        raise RuntimeError(f"ODrive with serial {serial} not found.")
    return odrv


def configure_odrive(odrv, side: Side) -> None:
    # Electrical configuration
    odrv.config.dc_bus_overvoltage_trip_level = 50
    odrv.config.dc_bus_undervoltage_trip_level = 10.5
    odrv.config.dc_max_positive_current = math.inf
    odrv.config.dc_max_negative_current = -math.inf
    odrv.config.brake_resistor0.enable = False

    # Motor configuration
    odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
    odrv.axis0.config.motor.pole_pairs = 7
    odrv.axis0.config.motor.torque_constant = 0.053354838709677416
    odrv.axis0.config.motor.current_soft_max = 80
    odrv.axis0.config.motor.current_hard_max = 100
    odrv.axis0.config.motor.calibration_current = 16
    odrv.axis0.config.motor.resistance_calib_max_voltage = 2
    odrv.axis0.config.calibration_lockin.current = 10
    odrv.axis0.motor.motor_thermistor.config.enabled = False

    # Controller configuration
    odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
    odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
    odrv.axis0.controller.config.vel_ramp_rate = 500
    odrv.axis0.controller.config.vel_limit = 50
    odrv.axis0.controller.config.vel_limit_tolerance = 2.0
    odrv.axis0.controller.config.vel_gain = 0.1
    odrv.axis0.controller.config.vel_integrator_gain = 0.3
    odrv.axis0.controller.config.spinout_electrical_power_threshold = 100
    odrv.axis0.controller.config.spinout_mechanical_power_threshold = -100
    odrv.axis0.config.torque_soft_min = -math.inf
    odrv.axis0.config.torque_soft_max = math.inf
    odrv.axis0.trap_traj.config.accel_limit = 500

    # Communication configuration
    odrv.can.config.protocol = Protocol.SIMPLE
    odrv.can.config.baud_rate = 500000
    odrv.axis0.config.can.node_id = 0 if side == Side.LEFT else 1
    odrv.axis0.config.can.heartbeat_msg_rate_ms = 100
    odrv.axis0.config.can.encoder_msg_rate_ms = 10
    odrv.axis0.config.can.iq_msg_rate_ms = 1000
    odrv.axis0.config.can.torques_msg_rate_ms = 1000
    odrv.axis0.config.can.error_msg_rate_ms = 10
    odrv.axis0.config.can.temperature_msg_rate_ms = 1000
    odrv.axis0.config.can.bus_voltage_msg_rate_ms = 1000
    odrv.axis0.config.enable_watchdog = False
    odrv.config.enable_uart_a = False

    # Encoder configuration
    odrv.axis0.config.encoder_bandwidth = 100
    odrv.hall_encoder0.config.enabled = True
    odrv.axis0.config.load_encoder = EncoderId.HALL_ENCODER0
    odrv.axis0.config.commutation_encoder = EncoderId.HALL_ENCODER0


def main(op: Operation, sides: list[Side] = [Side.LEFT, Side.RIGHT]) -> None:
    for side in sides:
        try:
            odrv = find_odrive(side)
        except RuntimeError as e:
            print(f"Error finding ODrive for side {side.name}: {e}")
        except TimeoutError:
            print(f"Finding ODrive timed out for side {side.name}.")
            continue

        if op == Operation.CONFIGURE:
            print(f"Configuring ODrive {odrv.serial_number} on side {side.name}...")

            configure_odrive(odrv, side)
            print("Sent configuration to ODrive.")

            try:
                odrv.save_configuration()
            except odrive.DeviceLostException:
                odrv = find_odrive(side)
            print("Saved configuration to ODrive.")

        elif op == Operation.CALIBRATE:
            print(f"Calibrating ODrive {odrv.serial_number} on side {side.name}...")

            odrv.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
            time.sleep(1.0)

            while odrv.axis0.current_state != AxisState.IDLE:
                pass

            print("Calibration complete.")

            try:
                odrv.save_configuration()
            except odrive.DeviceLostException:
                odrv = find_odrive(side)
            print("Saved calibration to ODrive.")


def parse_args():
    import argparse

    parser = argparse.ArgumentParser(description="Configure ODrive devices.")
    parser.add_argument(
        "--sides",
        nargs="+",
        type=str,
        choices=["LEFT", "RIGHT"],
        default=["LEFT", "RIGHT"],
        help="Specify which sides to configure (LEFT, RIGHT).",
    )
    parser.add_argument(
        "--op",
        type=str,
        choices=["CONFIGURE", "CALIBRATE"],
        default="CONFIGURE",
        help="Specify the operation to perform (CONFIGURE, CALIBRATE).",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    sides = [Side.LEFT if side == "LEFT" else Side.RIGHT for side in args.sides]
    op = Operation.CONFIGURE if args.op == "CONFIGURE" else Operation.CALIBRATE

    try:
        main(op, sides)
    except KeyboardInterrupt:
        print("Operation interrupted by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
