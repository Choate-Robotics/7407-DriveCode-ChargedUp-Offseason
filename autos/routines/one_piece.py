from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from command import Target
from robot_systems import Robot, Sensors
import config, constants
from auto_routine import AutoRoutine

def set_target(self, target):
    config.active_target = target
    
def eject(self):
    if config.active_piece == config.GamePiece.cube:
        self.eject_cube()
    else:
        self.eject_cone()

auto = SequentialCommandGroup(
    InstantCommand(lambda: set_target(config.Target.elevator)),
    InstantCommand(Robot.elevator.set_auto_position),
    Target(Robot.intake, Robot.elevator),
    WaitCommand(.5),
    InstantCommand(eject),
    WaitCommand(1.5),
    InstantCommand(Robot.intake.stop),
    InstantCommand(lambda: set_target(config.Target.idle)),
    Target(Robot.intake, Robot.elevator),
)

routine = AutoRoutine(auto)