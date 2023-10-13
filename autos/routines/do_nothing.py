from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from auto_routine import AutoRoutine


auto = SequentialCommandGroup(
    WaitCommand(1),
)

routine = AutoRoutine(auto)