import rev

from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.sensors.limit_switches import MagneticLimitSwitch

import config
import constants
from units.SI import meters

# TODO: Change config once robot is built
ELEVATOR_CONFIG = SparkMaxConfig(
    0.1, 0, 0.03, 0.000, (-.5, .75), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)
#-.5, .75

class Elevator(Subsystem):

    motor_extend: SparkMax = SparkMax(
        config.elevator_can_id, config=ELEVATOR_CONFIG, inverted=False
    )

    def __init__(self) -> None:
        super().__init__()
        self.elevator_bottom_sensor = None
        self.zeroed: bool = False

    def init(self) -> None:
        self.motor_extend.init()
        self.motor_extend.motor.setClosedLoopRampRate(config.elevator_ramp_rate)
        self.elevator_bottom_sensor = MagneticLimitSwitch(config.magnetic_limit_switch_port)

    def stop(self) -> None:
        """
        Stops the elevator
        """
        self.motor_extend.set_raw_output(0)

    def set_length(self, length: float) -> None:
        """
        Sets the length of the elevator in meters

        Args:
            length (meters): Length in meters
        """
        self.motor_extend.set_target_position(length)

    def get_length(self) -> float:
        """
        Gets the length of the elevator
        :return:  Length in meters
        "Return Type: meters
        """
        return self.motor_extend.get_sensor_position()

    def set_voltage(self, voltage: float) -> None:
        self.motor_extend.pid_controller.setReference(voltage, rev.CANSparkMax.ControlType.kVoltage)


    def get_voltage(self) -> float:
        return self.motor_extend.motor.getAppliedOutput()
    
