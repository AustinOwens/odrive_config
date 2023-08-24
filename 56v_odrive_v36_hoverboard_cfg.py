#!/usr/bin/env python3
"""
These configurations are based off of the hoverboard tutorial found on the
Odrive Robotics website located here:
https://docs.odriverobotics.com/hoverboard

@author: Austin Owens
@date: 3/14/2021
"""

import argparse
import sys
import time

# from fibre.protocol import ChannelBrokenException
import odrive
from odrive.enums import *


class HBMotorConfig:
    """
    Class for configuring an Odrive axis for a Hoverboard motor.
    Only works with one Odrive at a time.
    """

    # Hoverboard Kv
    HOVERBOARD_KV = 16.0

    # Min/Max phase inductance of motor
    MIN_PHASE_INDUCTANCE = 0
    MAX_PHASE_INDUCTANCE = 0.001

    # Min/Max phase resistance of motor
    MIN_PHASE_RESISTANCE = 0
    MAX_PHASE_RESISTANCE = 0.5

    # Tolerance for encoder offset float
    ENCODER_OFFSET_FLOAT_TOLERANCE = 0.05

    def __init__(self, axis_num, erase_config):
        """
        Initalizes HBMotorConfig class by finding odrive, erase its
        configuration, and grabbing specified axis object.

        :param axis_num: Which channel/motor on the odrive your referring to.
        :type axis_num: int (0 or 1)
        :param erase_config: Erase existing config before setting new config.
        :type erase_config: bool (True or False)
        """

        self.axis_num = axis_num

        self.erase_config = erase_config

        # Connect to Odrive
        print("Looking for ODrive...")
        self._find_odrive()
        print("Found ODrive.")

    def _find_odrive(self):
        # connect to Odrive
        self.odrv = odrive.find_any()
        self.odrv_axis = getattr(self.odrv, "axis{}".format(self.axis_num))

    def configure(self):
        """
        Configures the odrive device for a hoverboard motor.
        """

        if self.erase_config:
            # Erase pre-exsisting configuration
            print("Erasing pre-exsisting configuration...")
            try:
                self.odrv.erase_configuration()
            except Exception:
                pass

        self._find_odrive()

        # Set this to True if using a brake resistor
        self.odrv.config.enable_brake_resistor = True

        # This is the resistance of the brake resistor. You can leave this
        # at the default setting if you are not using a brake resistor. Note
        # that there may be some extra resistance in your wiring and in the
        # screw terminals, so if you are getting issues while braking you may
        # want to increase this parameter by around 0.05 ohm.
        # self.odrv_axis.config.brake_resistance  = 2.0

        # This is the amount of current allowed to flow back into the power supply.
        # The convention is that it is negative. By default, it is set to a
        # conservative value of 10mA. If you are using a brake resistor and getting
        # DC_BUS_OVER_REGEN_CURRENT errors, raise it slightly. If you are not using
        # a brake resistor and you intend to send braking current back to the power
        # supply, set this to a safe level for your power source. Note that in that
        # case, it should be higher than your motor current limit + current limit
        # margin.
        # self.odrv_axis.config.dc_max_negative_current  = -0.01

        # Standard 6.5 inch hoverboard hub motors have 30 permanent magnet
        # poles, and thus 15 pole pairs
        self.odrv_axis.motor.config.pole_pairs = 15

        # Hoverboard hub motors are quite high resistance compared to the hobby
        # aircraft motors, so we want to use a bit higher voltage for the motor
        # calibration, and set up the current sense gain to be more sensitive.
        # The motors are also fairly high inductance, so we need to reduce the
        # bandwidth of the current controller from the default to keep it
        # stable.
        self.odrv_axis.motor.config.resistance_calib_max_voltage = 4
        self.odrv_axis.motor.config.requested_current_range = 25
        self.odrv_axis.motor.config.current_control_bandwidth = 100

        # Estimated KV but should be measured using the "drill test", which can
        # be found here:
        # https://discourse.odriverobotics.com/t/project-hoverarm/441
        self.odrv_axis.motor.config.torque_constant = 8.27 / self.HOVERBOARD_KV

        # Hoverboard motors contain hall effect sensors instead of incremental
        # encorders
        self.odrv_axis.encoder.config.mode = ENCODER_MODE_HALL

        # The hall feedback has 6 states for every pole pair in the motor. Since
        # we have 15 pole pairs, we set the cpr to 15*6 = 90.
        self.odrv_axis.encoder.config.cpr = 90

        # Since hall sensors are low resolution feedback, we also bump up the
        # offset calibration displacement to get better calibration accuracy.
        self.odrv_axis.encoder.config.calib_scan_distance = 150

        # Since the hall feedback only has 90 counts per revolution, we want to
        # reduce the velocity tracking bandwidth to get smoother velocity
        # estimates. We can also set these fairly modest gains that will be a
        # bit sloppy but shouldn’t shake your rig apart if it’s built poorly.
        # Make sure to tune the gains up when you have everything else working
        # to a stiffness that is applicable to your application.
        self.odrv_axis.encoder.config.bandwidth = 100
        self.odrv_axis.controller.config.pos_gain = 6
        self.odrv_axis.controller.config.vel_gain = (
            0.02
            * self.odrv_axis.motor.config.torque_constant
            * self.odrv_axis.encoder.config.cpr
        )
        self.odrv_axis.controller.config.vel_integrator_gain = (
            0.1
            * self.odrv_axis.motor.config.torque_constant
            * self.odrv_axis.encoder.config.cpr
        )
        self.odrv_axis.controller.config.vel_limit = 10

        # Set in position control mode so we can control the position of the
        # wheel
        self.odrv_axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

        # In the next step we are going to start powering the motor and so we
        # want to make sure that some of the above settings that require a
        # reboot are applied first.

        # Motors must be in IDLE mode before saving
        self.odrv_axis.requested_state = AXIS_STATE_IDLE
        try:
            print("Saving manual configuration and rebooting...")
            is_saved = self.odrv.save_configuration()
            if not is_saved:
                print("Error: Configuration not saved. Are all motors in IDLE state?")
            else:
                print("Calibration configuration saved.")

            print("Manual configuration saved.")
        except Exception as e:
            pass

        self._find_odrive()

        input("Make sure the motor is free to move, then press enter...")

        print("Calibrating Odrive for hoverboard motor (you should hear a " "beep)...")

        self.odrv_axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION

        # Wait for calibration to take place
        time.sleep(10)

        if self.odrv_axis.motor.error != 0:
            print(
                "Error: Odrive reported an error of {} while in the state "
                "AXIS_STATE_MOTOR_CALIBRATION. Printing out Odrive motor data for "
                "debug:\n{}".format(self.odrv_axis.motor.error, self.odrv_axis.motor)
            )

            sys.exit(1)

        if (
            self.odrv_axis.motor.config.phase_inductance <= self.MIN_PHASE_INDUCTANCE
            or self.odrv_axis.motor.config.phase_inductance >= self.MAX_PHASE_INDUCTANCE
        ):
            print(
                "Error: After odrive motor calibration, the phase inductance "
                "is at {}, which is outside of the expected range. Either widen the "
                "boundaries of MIN_PHASE_INDUCTANCE and MAX_PHASE_INDUCTANCE (which "
                "is between {} and {} respectively) or debug/fix your setup. Printing "
                "out Odrive motor data for debug:\n{}".format(
                    self.odrv_axis.motor.config.phase_inductance,
                    self.MIN_PHASE_INDUCTANCE,
                    self.MAX_PHASE_INDUCTANCE,
                    self.odrv_axis.motor,
                )
            )

            sys.exit(1)

        if (
            self.odrv_axis.motor.config.phase_resistance <= self.MIN_PHASE_RESISTANCE
            or self.odrv_axis.motor.config.phase_resistance >= self.MAX_PHASE_RESISTANCE
        ):
            print(
                "Error: After odrive motor calibration, the phase resistance "
                "is at {}, which is outside of the expected range. Either raise the "
                "MAX_PHASE_RESISTANCE (which is between {} and {} respectively) or "
                "debug/fix your setup. Printing out Odrive motor data for "
                "debug:\n{}".format(
                    self.odrv_axis.motor.config.phase_resistance,
                    self.MIN_PHASE_RESISTANCE,
                    self.MAX_PHASE_RESISTANCE,
                    self.odrv_axis.motor,
                )
            )

            sys.exit(1)

        # If all looks good, then lets tell ODrive that saving this calibration
        # to persistent memory is OK
        self.odrv_axis.motor.config.pre_calibrated = True

        # Check the alignment between the motor and the hall sensor. Because of
        # this step you are allowed to plug the motor phases in random order and
        # also the hall signals can be random. Just don’t change it after
        # calibration.
        print("Calibrating Odrive for hall encoder...")
        self.odrv_axis.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION

        # Wait for calibration to take place
        time.sleep(15)

        if self.odrv_axis.encoder.error != 0:
            print(
                "Error: Odrive reported an error of {} while in the state "
                "AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION. Printing out Odrive encoder "
                "data for debug:\n{}".format(
                    self.odrv_axis.encoder.error, self.odrv_axis.encoder
                )
            )

            sys.exit(1)

        print("Calibrating Odrive for encoder offset...")
        self.odrv_axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

        # Wait for calibration to take place
        time.sleep(30)

        if self.odrv_axis.encoder.error != 0:
            print(
                "Error: Odrive reported an error of {} while in the state "
                "AXIS_STATE_ENCODER_OFFSET_CALIBRATION. Printing out Odrive encoder "
                "data for debug:\n{}".format(
                    self.odrv_axis.encoder.error, self.odrv_axis.encoder
                )
            )

            sys.exit(1)

        # If all looks good, then lets tell ODrive that saving this calibration
        # to persistent memory is OK
        self.odrv_axis.encoder.config.pre_calibrated = True

        print("Calibrating Odrive for anticogging...")
        self.odrv_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self.odrv_axis.controller.start_anticogging_calibration()

        while self.odrv_axis.controller.config.anticogging.calib_anticogging:
            time.sleep(15)
            print("Still calibrating anticogging...")

        if self.odrv_axis.controller.error != 0:
            print(
                "Error: Odrive reported an error of {} while performing "
                "start_anticogging_calibration(). Printing out Odrive controller "
                "data for debug:\n{}".format(
                    self.odrv_axis.controller.error, self.odrv_axis.controller
                )
            )

            sys.exit(1)

        # If all looks good, then lets tell ODrive that saving this calibration
        # to persistent memory is OK
        self.odrv_axis.controller.config.anticogging.pre_calibrated = True

        # Motors must be in IDLE mode before saving
        self.odrv_axis.requested_state = AXIS_STATE_IDLE
        try:
            print("Saving calibration configuration and rebooting...")
            self.odrv.save_configuration()
            if not is_saved:
                print("Error: Configuration not saved. Are all motors in IDLE state?")
            else:
                print("Calibration configuration saved.")
        except Exception as e:
            pass

        self._find_odrive()

        print("Odrive configuration finished.")

    def mode_idle(self):
        """
        Puts the motor in idle (i.e. can move freely).
        """

        self.odrv_axis.requested_state = AXIS_STATE_IDLE

    def mode_close_loop_control(self):
        """
        Puts the motor in closed loop control.
        """

        self.odrv_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def move_input_pos(self, angle):
        """
        Puts the motor at a certain angle.

        :param angle: Angle you want the motor to move.
        :type angle: int or float
        """

        self.odrv_axis.controller.input_pos = angle / 360.0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Hoverboard Motor Calibration")

    # Argument for axis_num
    parser.add_argument(
        "--axis_num",
        type=int,
        choices=[0, 1],  # Only allow 0 or 1
        required=True,
        help="Motor axis number which can only be 0 or 1.",
    )

    # Argument for erase_config
    parser.add_argument(
        "--erase_config",
        action="store_true",  # If present, set to True. If absent, set to False.
        help="Flag to determine if the config should be erased.",
    )

    # Argument to conduct motor test (make sure motor can move freely)
    parser.add_argument(
        "--motor_test",
        action="store_true",  # If present, set to True. If absent, set to False.
        help="Flag to determine if the config should be erased.",
    )

    args = parser.parse_args()

    hb_motor_config = HBMotorConfig(
        axis_num=args.axis_num, erase_config=args.erase_config
    )
    hb_motor_config.configure()

    if args.motor_test:
        print("Placing motor in close loop. If you move motor, motor will resist you.")
        hb_motor_config.mode_close_loop_control()

        print("CONDUCTING MOTOR TEST")

        # Go from 0 to 360 degrees in increments of 30 degrees
        for angle in range(0, 390, 30):
            print("Setting motor to {} degrees.".format(angle))
            hb_motor_config.move_input_pos(angle)
            time.sleep(5)

        print("Placing motor in idle. If you move motor, motor will move freely")
        hb_motor_config.mode_idle()
