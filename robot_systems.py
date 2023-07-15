import subsystem
from sensors import Limelight
import wpilib
import constants

class Robot:
    pass


class Pneumatics:
    pass


class Sensors:
    limeLight_F = Limelight(constants.limelight_offset['front'], "limelight-F")
    limeLight_B = Limelight(constants.limelight_offset['back'], "limelight-B")
