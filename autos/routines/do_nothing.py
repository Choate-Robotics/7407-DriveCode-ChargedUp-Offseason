from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from autos.auto_routine import AutoRoutine
from robot_systems import Robot
from command import Target, ZeroElevator, FollowPathCustom, Path, SetPosition
from robot_systems import Robot
import config, constants
from wpimath.geometry import Pose2d, Translation2d

def set_target(target):
    config.active_target = target
    
def eject():
    if config.active_piece == config.GamePiece.cube:
        Robot.intake.eject_cube()
    else:
        Robot.intake.eject_cone()

# auto = SequentialCommandGroup(
#     WaitCommand(1),
# )

right_node = config.active_poses.Nodes[9]
right_cone = config.active_poses.Auto_Pieces[4]
left_node = config.active_poses.Nodes[7]
center_node = config.active_poses.Nodes[8]
middle_right_cube = config.active_poses.Auto_Pieces[3]

max_vel = constants.drivetrain_max_vel

max_accel = constants.drivetrain_max_target_accel

# path1 = Path(
#     Pose2d(1.887, 0.4020, 0),
#     Pose2d(6.466, 0.892, 0),
#     [],
#     constants.drivetrain_max_vel,
#     constants.drivetrain_max_target_accel,
# )

path1 = Path(
    right_node,
    right_cone,
    [],
    max_vel,
    max_accel
)
    

# path2 = Path(
#     Pose2d(6.466, 0.892, 0),
#     Pose2d(1.7808, 1.05, 0),
#     [],
#     constants.drivetrain_max_vel,
#     constants.drivetrain_max_target_accel
# )

path2 = Path(
    right_cone,
    left_node,
    [],
    max_vel,
    max_accel
)

# path3 = Path(
#     Pose2d(1.7808, 1.05, 0),
#     Pose2d(6.465, 2.106, 0),
#     [Translation2d(4.879, 0.971)],
#     constants.drivetrain_max_vel,
#     constants.drivetrain_max_target_accel
# )

path3 = Path(
    left_node,
    middle_right_cube,
    [],
    max_vel,
    max_accel
)

# path4 = Path(
#     Pose2d(6.465, 2.106, 0),
#     Pose2d(1.7808, 1.05, 0),
#     [Translation2d(4.879, 0.971)],
#     constants.drivetrain_max_vel,
#     constants.drivetrain_max_target_accel
# )

path4 = Path(
    middle_right_cube,
    center_node,
    [],
    max_vel,
    max_accel
)

def generate_paths(paths):
    for path in paths:
        path.generate()
        path.getPoses()

auto = SequentialCommandGroup(
    InstantCommand(generate_paths, [path1, path2, path3, path4]),
    # SetPosition(Robot.drivetrain, Pose2d(1.887, 0.4020, 0)),
    SetPosition(Robot.drivetrain, right_node),
    WaitCommand(1),
    FollowPathCustom(Robot.drivetrain, path1),
    FollowPathCustom(Robot.drivetrain, path2),
    FollowPathCustom(Robot.drivetrain, path3),
    FollowPathCustom(Robot.drivetrain, path4)
    # InstantCommand(Robot.elevator.set_auto_position),
    # # InstantCommand(lambda: )
    # InstantCommand(lambda: set_target(config.Target.high)),
    # # ZeroElevator(Robot.elevator),
    # WaitCommand(3),
    # Target(Robot.intake, Robot.elevator, force=True),
    # WaitCommand(2),
    # InstantCommand(eject),
    # WaitCommand(3),
    # InstantCommand(Robot.intake.stop),
    # # InstantCommand(lambda: Robot.drivetrain.)
    # InstantCommand(lambda: set_target(config.Target.idle)),
    # Target(Robot.intake, Robot.elevator, force=True),
)

routine = AutoRoutine(auto)