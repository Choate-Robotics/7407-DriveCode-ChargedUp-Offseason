from autos.routines.one_piece import auto as one_piece
from robot_systems import Robot, Sensors
from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
import config, constants
from autos.auto_routine import AutoRoutine
from wpimath.geometry import Pose2d, Rotation2d, Transform2d
from command import FollowPathCustom, Path, AutoPickup

start_pose: Pose2d = Pose2d(0, 0, Rotation2d.fromDegrees(0))

far_left_stage = Pose2d(constants.Poses.far_left_piece_auto['red'], Rotation2d.fromDegrees(0))



path_grid_to_far_left_stage = Path(
    start_pose,
    far_left_stage,
    [],
    constants.drivetrain_max_vel,
    constants.drivetrain_max_target_accel,
    False,
).generate()

auto = SequentialCommandGroup(
    one_piece,
    FollowPathCustom(path_grid_to_far_left_stage),
    AutoPickup(Robot.drivetrain, Robot.intake, Robot.elevator, Sensors.limeLight_B, config.GamePiece.cone),
    
)

routine = AutoRoutine(auto)