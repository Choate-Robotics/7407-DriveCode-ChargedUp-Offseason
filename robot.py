import commands2
import ctre
import wpilib
import command
import config
import constants
from robot_systems import Robot, Pneumatics, Sensors
import sensors
import subsystem
import utils
from oi.OI import OI
import ntcore
import math


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Initialize Operator Interface
        OI.init()
        OI.map_controls()
        period = .03
        commands2.CommandScheduler.getInstance().setPeriod(period)
        
        Robot.drivetrain.init()
        Robot.intake.init()
        
        # for i in range(15):
        #     Robot.drivetrain.n_front_left.initial_zero()
        #     Robot.drivetrain.n_front_right.initial_zero()
        #     Robot.drivetrain.n_back_left.initial_zero()
        #     Robot.drivetrain.n_back_right.initial_zero()
        
        Sensors.gyro = Robot.drivetrain.gyro
        
    def robotPeriodic(self):
        
        nt = ntcore.NetworkTableInstance.getDefault()
        
        # try:
        #     commands2.CommandScheduler.getInstance().run()
        # except Exception as e:
        #     nt.getTable("Command Scheduler").putString("Last Error", str(e))
        
        commands2.CommandScheduler.getInstance().run()

        Sensors.limeLight_F.update()
        Sensors.limeLight_B.update()
        
        
        
        nts = nt.getTable("Swerve Analog")
        
        nts.putNumber("ABS VAL Back Left", config.back_left_zeroed_pos)
        
        nts.putNumber('MAX VEL', Robot.drivetrain.max_vel)
        
        nts.putNumber("BACK LEFT ENCODER", (Robot.drivetrain.n_back_left.get_turn_motor_angle() / (2 * math.pi) * constants.drivetrain_turn_gear_ratio))
        
        nts.putNumber("Front Left", config.front_left_encoder.getAbsolutePosition())
        nts.putNumber("Front Right", config.front_right_encoder.getAbsolutePosition())
        nts.putNumber("Back Left", config.back_left_encoder.getAbsolutePosition())
        nts.putNumber("Back Right", config.back_right_encoder.getAbsolutePosition())
        
        nti = nt.getTable("Intake")
        
        calculation = ((Robot.drivetrain.n_back_left.get_turn_motor_angle()/ (-2 * math.pi)) + config.back_left_zeroed_pos) - config.back_left_encoder.getAbsolutePosition()
        
        ntcore.NetworkTableInstance.getDefault().getTable("Swerve Difference").putNumber("back left calculation", calculation)
        
        nti.putNumber("Motor Encoder", Robot.intake.wrist_motor.get_sensor_position() / constants.wrist_gear_ratio)
        nti.putNumber("ABS Encoder", Robot.intake.wrist_abs_encoder.getPosition())
        
    # Initialize subsystems


    # Pneumatics

    def teleopInit(self):
        
        Robot.intake.zero_wrist()
        
        # Robot.intake.set_lower_output(-1)
        # Robot.intake.set_upper_output(-1)
        
        commands2.CommandScheduler.getInstance().schedule(command.DriveSwerveCustom(Robot.drivetrain))

    def teleopPeriodic(self):
        pass
    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    

if __name__ == "__main__":
    wpilib.run(_Robot)
