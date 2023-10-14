from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from command import Target
from robot_systems import Robot
import config
from autos.auto_routine import AutoRoutine

def set_target(target):
    config.active_target = target
    
def eject():
    if config.active_piece == config.GamePiece.cube:
        Robot.intake.eject_cube()
    else:
        Robot.intake.eject_cone()

auto = SequentialCommandGroup(
    # InstantCommand(Robot.intake.zero_wrist),
    WaitCommand(2),
    # InstantCommand(Robot.elevator.set_auto_position),
    # InstantCommand(lambda: set_target(config.Target.high)),
    # # ZeroElevator(Robot.elevator),
    # Target(Robot.intake, Robot.elevator, auto=True),
    # WaitCommand(2),
    # InstantCommand(eject),
    # WaitCommand(3),
    # InstantCommand(Robot.intake.stop),
    # InstantCommand(lambda: set_target(config.Target.idle)),
    # Target(Robot.intake, Robot.elevator, auto=True),
)

routine = AutoRoutine(auto)