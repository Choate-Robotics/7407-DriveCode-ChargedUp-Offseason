from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.utils.units import radians
from math import pi
import constants
import config
import math
import rev
import wpilib


# cube -> same outwards spin(top up, bottom up), cone -> same inwards spin(top down, bottom down)
class Intake(Subsystem):
    # left = lower roller, right = upper roller
    # lower_back_dist_sensor: rev.AnalogInput() = None
    # upper_back_dist_sensor: rev.AnalogInput() = None

    def __init__(self):
        super().__init__()
        self.lower_intake_motor = SparkMax(
            can_id=config.left_intake_motor_id,
            config=config.INTAKE_CONFIG
        )
        self.upper_intake_motor = SparkMax(
            can_id=config.right_intake_motor_id,
            config=config.INTAKE_CONFIG
        )
        self.wrist_motor = SparkMax(
            can_id=config.wrist_motor_id, inverted=True,
            config=config.WRIST_CONFIG
        )
        self.intake_speed = config.default_intake_speed
        self.disable_rotation: bool = config.disable_wrist_rotation
        self.wrist_abs_encoder = None
        self.dist_sensor: rev.AnalogInput = None

    def init(self):
        self.lower_intake_motor.init()
        self.upper_intake_motor.init()
        self.wrist_motor.init()
        self.dist_sensor = self.upper_intake_motor.motor.getAnalog()
        self.wrist_abs_encoder = self.wrist_motor.motor.getAbsoluteEncoder(
            rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )

    # Modified version of Sid's game piece detection code, change values on testing
    # def get_no_grab_cube_detected(self):
    #     avg_voltage = self.lower_back_dist_sensor.getVoltage()
    #     return 0.3 < avg_voltage

    def get_cube_detected(self):
        """
        :return: True if cube is detected, False if not
        """
        avg_voltage = self.lower_back_dist_sensor.getVoltage()
        return 0.7 < avg_voltage  # put voltage in config

    def get_cone_detected(self):
        """
        :return: True if cone is detected, False if not
        """
        avg_voltage = self.upper_back_dist_sensor.getVoltage() + self.lower_back_dist_sensor.getVoltage() / 2
        return 0.6 < avg_voltage

    def get_double_station_detected(self):
        """
        :return: True if double station is detected, False if not
        """
        avg_voltage = self.upper_back_dist_sensor.getVoltage() + self.lower_back_dist_sensor.getVoltage() / 2
        return 0.7 < avg_voltage

    def set_upper_output(self, speed: float):
        """
        Sets the upper intake motor to the given speed
        :param speed: The speed to set the upper intake motor to(float)
        :return: None
        """
        self.upper_intake_motor.set_raw_output(speed)

    def set_lower_output(self, speed: float):
        """
        Sets the lower intake motor to the given speed
        :param speed: The speed to set the lower intake motor to(float)
        :return: None
        """
        self.lower_intake_motor.set_raw_output(speed)

    def grab_cube(self):
        """
        Sets the intake motors to grab a cube
        :return: None
        """
        self.set_lower_output(-self.intake_speed)
        self.set_upper_output(-self.intake_speed)

    def grab_cone(self):
        """
        Sets the intake motors to grab a cone
        :return: None
        """
        self.set_lower_output(self.intake_speed)
        self.set_upper_output(self.intake_speed)

    def eject_cone(self):
        """
        Sets the intake motors to eject a cone
        :return: None
        """
        self.set_lower_output(-self.intake_speed)
        self.set_upper_output(-self.intake_speed)

    def eject_cube(self):
        """
        Sets the intake motors to eject a cube
        :return: None
        """
        self.set_lower_output(self.intake_speed)
        self.set_upper_output(self.intake_speed)

    def disengage(self):
        """
        Sets the intake motors to disengage
        :return: None
        """
        self.set_lower_output(0.05)
        self.set_upper_output(0.05)

    def stop(self):
        """
        Sets the intake motors to stop
        :return: None
        """
        self.lower_intake_motor.set_raw_output(0)
        self.upper_intake_motor.set_raw_output(0)

    def set_wrist_angle(self, pos: float):
        """
        Sets the wrist angle to the given position
        :param pos: The position to set the wrist to(float)
        :return: None
        """
        if not self.disable_rotation:
            self.wrist_motor.set_target_position((pos / (pi * 2)) * constants.wrist_gear_ratio)

    def get_wrist_angle(self):
        """
        Gets the wrist rotation in radians
        :return:
        """
        return (self.wrist_motor.get_sensor_position() / constants.wrist_gear_ratio) * pi * 2

    def is_at_angle(self, angle: radians, threshold=math.radians(2)):
        """
        Checks if the wrist is at the given angle
        :param angle: The angle to check for
        :param threshold: The threshold to check for
        :return: True if the wrist is at the given angle, False otherwise
        """
        return abs(self.get_wrist_angle() - angle) < threshold

    def zero_wrist(self):
        """
        Zeros the wrist
        :return: None
        """
        abs_encoder_position: float = self.wrist_abs_encoder.getPosition()
        if abs_encoder_position > 0.5:
            abs_encoder_position = -(1 - abs_encoder_position)
        encoder_difference: float = abs_encoder_position - 0
        motor_change: float = encoder_difference * constants.wrist_gear_ratio
        self.wrist_motor.set_sensor_position(-motor_change)
        self.wrist_motor.set_target_position(-motor_change)
