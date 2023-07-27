from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.utils.units import radians
import config
import rev
import wpilib

INTAKE_CONFIG = SparkMaxConfig(k_P=1, k_I=1, k_D=1, k_F=1)


class Intake(Subsystem):
    # left = lower roller, right = upper roller
    # lower_back_dist_sensor: rev.AnalogInput() = None
    # upper_back_dist_sensor: rev.AnalogInput() = None

    def __init__(self):
        super().__init__()
        self.lower_intake_motor = SparkMax(
            can_id=config.left_intake_motor_id,
            config=INTAKE_CONFIG
        )
        self.upper_intake_motor = SparkMax(
            can_id=config.right_intake_motor_id,
            config=INTAKE_CONFIG
        )
        self.intake_speed = config.default_intake_speed

    def init(self):
        self.lower_intake_motor.init()
        self.upper_intake_motor.init()
        self.upper_back_dist_sensor = self.right_intake_motor.motor.getAnalog()
        self.lower_back_dist_sensor = self.left_intake_motor.motor.getAnalog()

    # Modified version of Sid's game piece detection code, change values on testing
    def get_no_grab_cube_detected(self):
        avg_voltage = self.lower_back_dist_sensor.getVoltage()
        return 0.3 < avg_voltage

    def get_cube_detected(self):
        avg_voltage = self.lower_back_dist_sensor.getVoltage()
        return 0.7 < avg_voltage

    def get_cone_detected(self):
        avg_voltage = self.upper_back_dist_sensor.getVoltage() + self.lower_back_dist_sensor.getVoltage() / 2
        return 0.6 < avg_voltage

    def get_double_station_detected(self):
        avg_voltage = self.upper_back_dist_sensor.getVoltage() + self.lower_back_dist_sensor.getVoltage() / 2
        return 0.7 < avg_voltage

    def set_upper_output(self, speed: float):
        self.upper_intake_motor.set_raw_output(speed)

    def set_lower_output(self, speed: float):
        self.lower_intake_motor.set_raw_output(speed)

    def grab_cube(self):
        self.set_lower_output(self.intake_speed)

    def grab_cone(self):
        self.set_lower_output(self.intake_speed)
        self.set_upper_output(self.intake_speed)

    def eject_piece(self):
        self.set_lower_output(-self.intake_speed)
        self.set_upper_output(-self.intake_speed)

    def disengage(self):
        self.set_lower_output(0.05)
        self.set_upper_output(0.05)

    def stop(self):
        self.lower_intake_motor.set_raw_output(0)
        self.upper_intake_motor.set_raw_output(0)
