import config
import subsystem
from sensors import Limelight
import wpilib
import constants
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveGyro

class Robot:

    intake = subsystem.Intake()

    drivetrain = subsystem.Drivetrain()



class Pneumatics:
    pass


class Sensors:
    limeLight_F = Limelight(constants.limelight_offset['front'], "limelight-F")
    limeLight_B = Limelight(constants.limelight_offset['back'], "limelight-B")

    gyro: SwerveGyro

