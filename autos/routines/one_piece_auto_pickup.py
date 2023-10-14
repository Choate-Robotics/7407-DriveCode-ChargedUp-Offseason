from autos.routines.one_piece import auto as one_piece
from robot_systems import Robot, Sensors
from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
import config, constants
from autos.auto_routine import AutoRoutine
from command import AutoPickup

auto = SequentialCommandGroup(
    one_piece,
    InstantCommand(lambda: Robot.drivetrain.set_driver_centric((.5 * config.calculated_max_vel, 0), 0)),
    WaitCommand(1),
    AutoPickup(Robot.drivetrain, Robot.intake, Robot.elevator, Sensors.limeLight_B, config.GamePiece.cone),
)

routine = AutoRoutine(auto)