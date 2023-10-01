from robotpy_toolkit_7407.utils import logger
from oi.keymap import Keymap
from robot_systems import Robot, Sensors
import command
from commands2 import InstantCommand
logger.info("Hi, I'm OI!")


class OI:
    @staticmethod
    def init() -> None:
        logger.info("Initializing OI...")

    @staticmethod
    def map_controls():
        logger.info("Mapping controls...")
    
        Keymap.Puncher.PUNCH_EXTEND.whenActive(command.ExtendPuncher(Robot.puncher))

        Keymap.Puncher.PUNCH_RETRACT.whenActive(command.RetractPuncher(Robot.puncher))
        
        Keymap.Drivetrain.RESET_GYRO.onTrue(command.DrivetrainZero(Robot.drivetrain)).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        Keymap.Drivetrain.X_MODE.onTrue(InstantCommand(lambda: Robot.drivetrain.x_mode()))
        
        Keymap.Drivetrain.AUTO_PICKUP.onTrue(command.LineupSwerve(Robot.drivetrain, Sensors.limeLight_B, 0))#.onFalse(command.DriveSwerveCustom(Robot.drivetrain))
