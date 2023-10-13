import one_piece
from robot_systems import Robot, Sensors
from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
import config, constants
from auto_routine import AutoRoutine

auto = SequentialCommandGroup(
    one_piece.auto,
    InstantCommand(lambda: Robot.drivetrain.set_driver_centric((.5 * config.calculated_max_vel, 0), 0)),
    WaitCommand(1),
    InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0)),
)

routine = AutoRoutine(auto)