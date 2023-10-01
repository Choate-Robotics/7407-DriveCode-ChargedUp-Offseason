import config
import subsystem
from sensors import Limelight, LimelightController
import wpilib
import constants
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveGyro

class Robot:

    intake = subsystem.Intake()

    drivetrain = subsystem.Drivetrain()
    puncher = subsystem.Puncher()


class Pneumatics:
    pass


class Sensors:
    limeLight_F: Limelight = Limelight(constants.limelight_offset['front'], "limelight-f")
    limeLight_B: Limelight = Limelight(constants.limelight_offset['back'], "limelight-b")

    odometry: LimelightController = LimelightController([limeLight_F, limeLight_B])

    gyro: SwerveGyro

