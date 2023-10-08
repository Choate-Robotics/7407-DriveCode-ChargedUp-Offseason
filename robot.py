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

        # Initialize subsystems

        # Initialize Operator Interface
        
        period = .03
        commands2.CommandScheduler.getInstance().setPeriod(period)
        
        Robot.drivetrain.init()
        Robot.intake.init()
        Robot.elevator.init()
        
        # for i in range(15):
        #     Robot.drivetrain.n_front_left.initial_zero()
        #     Robot.drivetrain.n_front_right.initial_zero()
        #     Robot.drivetrain.n_back_left.initial_zero()
        #     Robot.drivetrain.n_back_right.initial_zero()
        
        Sensors.gyro = Robot.drivetrain.gyro
        
        OI.init()
        OI.map_controls()
        
    def robotPeriodic(self):
        
        nt = ntcore.NetworkTableInstance.getDefault()
        
        if config.DEBUG_MODE: 
            commands2.CommandScheduler.getInstance().run()
        else:
            try:
                commands2.CommandScheduler.getInstance().run()
            except Exception as e:
                nt.getTable("Command Scheduler").putString("Last Error", str(e))
        

        Sensors.limeLight_F.update()
        Sensors.limeLight_B.update()
        
        Sensors.odometry.update()
        
        
    # Initialize subsystems


    # Pneumatics

    def teleopInit(self):
        

        Robot.intake.zero_wrist()
        Robot.intake.wrist_zeroed = True
        # Robot.intake.set_lower_output(-1)
        # Robot.intake.set_upper_output(-1)
        commands2.CommandScheduler.getInstance().schedule(command.DriveSwerveCustom(Robot.drivetrain))
        # commands2.CommandScheduler.getInstance().schedule(command.ZeroElevator(Robot.elevator))

    def teleopPeriodic(self):
        Robot.intake.rumble_if_detected()
        
        
        
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
