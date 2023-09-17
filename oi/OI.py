from robotpy_toolkit_7407.utils import logger
from oi.keymap import Keymap
from robot_systems import Robot
import command
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
