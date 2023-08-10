import command
from robot_systems import Robot
from commands2 import CommandScheduler, WaitCommand, PrintCommand, ParallelCommandGroup, SequentialCommandGroup, InstantCommand
import math, random



def test_extension():
    puncher = command.ExtendPuncher(Robot.puncher)
    puncher.initialize()
    puncher.execute()
    puncher.end(False)

def test_retraction():
    puncher = command.RetractPuncher(Robot.puncher)
    puncher.initialize()
    puncher.execute()
    puncher.end(False)