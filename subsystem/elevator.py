import math

import rev
import wpilib
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig

import config
import constants

# TODO: Change config once robot is built
ELEVATOR_CONFIG = SparkMaxConfig(
    1.5, 0, 0.004, 0.00017, (-1, 1), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)

class Elevator(Subsystem):

    length_constant: float = 1 / constants.elevator_length_per_rotation
    angle_constant: float = constants.elevator_rotation_gear_ratio * math.pi / 2

    def __init__(self) -> None:
        super().__init__()
        self.extend_motor: SparkMax = SparkMax(config.elevator_can_id, config=ELEVATOR_CONFIG, inverted=True)

        self.initialized: bool = False
        self.position: float = 0.0

    def init(self) -> None:
        self.extend_motor.init()
        self.initialized = True

    def stop(self) -> None:
        # Stops the rotation of the elevator motor
        self.extend_motor.set_raw_output(0)

    def set_length(self) -> None:
        # Sets length in meters
        self.extend_motor.set_target_position(self.length_constant)

    def get_length(self) -> float:
        # Returns length in meters
        return self.extend_motor.get_sensor_position() / self.length_constant;

    def set_angle(self) -> None:
        # Sets angle in radians
        self.extend_motor.set_target_position(self.angle_constant)

    def get_angle(self) -> float:
        # Returns angle in radians
        return self.extend_motor.get_sensor_position() / self.angle_constant
