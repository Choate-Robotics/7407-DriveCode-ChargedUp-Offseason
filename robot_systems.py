import config, utils
import subsystem
from sensors import Limelight, LimelightController, FieldOdometry, ALeds
import wpilib

import constants
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveGyro

class Robot:

    intake = subsystem.Intake()
    
    elevator = subsystem.Elevator()

    drivetrain = subsystem.Drivetrain()
    puncher = subsystem.Puncher()

class Pneumatics:
    pass

class PowerDistribution:
    pdh = wpilib.PowerDistribution()
    
class LEDs:
    
    elevator = ALeds(config.elevator_leds_id, config.elevator_leds_size)
    

class Sensors:
    limeLight_F: Limelight = Limelight(constants.limelight_offset['front'], "limelight-f")
    limeLight_B: Limelight = Limelight(constants.limelight_offset['back'], "limelight-b")

    l_c: LimelightController 
    
    odometry: FieldOdometry
    
    poses: utils.Poses = utils.Poses()

    gyro: SwerveGyro

