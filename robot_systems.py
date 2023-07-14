import subsystem
from sensors import Limelight
import wpilib
import constants

class Robot:
    pass


class Pneumatics:
    pass


class Sensors:
    limeLight_F = Limelight(constants.front_limelight_offset, "limelight-F")
    limeLight_B = Limelight(constants.back_limelight_offset, "limelight-B")
