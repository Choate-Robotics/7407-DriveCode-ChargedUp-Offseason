import command
from robot_systems import Robot
from commands2 import CommandScheduler, WaitCommand, PrintCommand, ParallelCommandGroup, SequentialCommandGroup, InstantCommand
import math, random

Robot.drivetrain.init()

def test_DriveSwerveCustom():
    # checks if command runs without error under changes in axis_dx, axis_dy, and axis_rotation
    drive = command.DriveSwerveCustom(Robot.drivetrain)
    drive.initialize()
    drive.execute()
    drive.end(False)
    
def test_DrivetrainZero():
    # checks if command runs without error
    Robot.drivetrain.gyro.reset_angle(random.randrange(0, 360, 1))
    zero = command.DrivetrainZero(Robot.drivetrain)
    zero.initialize()
    zero.execute()
    zero.end(False)
    assert zero.isFinished() == True
    assert Robot.drivetrain.gyro.get_robot_heading() == 0.0