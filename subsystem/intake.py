from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.utils.units import radians
import config
import rev
import wpilib

INTAKE_CONFIG = SparkMaxConfig(k_P=1, k_I=1, k_D=1, k_F=1)


class Intake(Subsystem):
    # left_back_dist_sensor: rev.AnalogInput() = None
    # right_back_dist_sensor: rev.AnalogInput() = None

    def __init__(self):
        super().__init__()
        self.left_intake_motor = SparkMax(
            can_id=config.left_intake_motor_id,
            config=INTAKE_CONFIG
        )
        self.right_intake_motor = SparkMax(
            can_id=config.right_intake_motor_id,
            config=INTAKE_CONFIG
        )
        self.intake_speed = config.default_intake_speed

    def init(self):
        self.left_intake_motor.init()
        self.right_intake_motor.init()
        self.right_back_dist_sensor = self.right_intake_motor.motor.getAnalog()
        self.left_back_dist_sensor = self.left_intake_motor.motor.getAnalog()

    # Modified version of Sid's game piece detection code, change values on testing
    def get_no_grab_cube_detected(self):
        avg_voltage = self.right_back_dist_sensor.getVoltage() + self.left_back_dist_sensor.getVoltage() / 2
        return 0.3 < avg_voltage

    def get_cube_detected(self):
        avg_voltage = self.right_back_dist_sensor.getVoltage() + self.left_back_dist_sensor.getVoltage() / 2
        return 0.7 < avg_voltage

    def get_cone_detected(self):
        avg_voltage = self.right_back_dist_sensor.getVoltage() + self.left_back_dist_sensor.getVoltage() / 2
        return 0.6 < avg_voltage

    def get_double_station_detected(self):
        avg_voltage = self.right_back_dist_sensor.getVoltage() + self.left_back_dist_sensor.getVoltage() / 2
        return 0.7 < avg_voltage

    def set_output(self, speed: float):
        self.left_intake_motor.set_raw_output(speed)
        self.right_intake_motor.set_raw_output(speed)
