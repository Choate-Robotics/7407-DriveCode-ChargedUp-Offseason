from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand, ParallelCommandGroup
from autos.auto_routine import AutoRoutine
from robot_systems import Robot, Sensors
from command import Target, ZeroElevator, FollowPathCustom, Path, SetPosition, SetElevatorPosition, SetIntake
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
        
def change_cube():
    config.active_piece = config.GamePiece.cube
    
def change_cone():
    config.active_piece = config.GamePiece.cone

# auto = SequentialCommandGroup(
#     WaitCommand(1),
# )


pse = Sensors.poses

right_node = (pse.Type.KNodes, 9)
right_cone = (pse.Type.KAutoPieces, 4)
left_node = (pse.Type.KNodes, 7)
center_node = (pse.Type.KNodes, 8)
middle_right_cube = (pse.Type.KAutoPieces, 3)

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
    

path2 = Path(
    right_cone,
    left_node,
    [],
    max_vel,
    max_accel
)

path3 = Path(
    left_node,
    middle_right_cube,
    [],
    max_vel,
    max_accel
)

path4 = Path(
    middle_right_cube,
    center_node,
    [],
    max_vel,
    max_accel
)

score = InstantCommand(eject)

set_cube = InstantCommand(change_cube)

set_cone = InstantCommand(change_cone)

auto = SequentialCommandGroup(
    InstantCommand(Sensors.poses.init),
    set_cone,
    SetPosition(Robot.drivetrain, right_node),
    SetElevatorPosition(Robot.elevator),
    Target(Robot.intake, Robot.elevator, config.Target.high),
    score,
    ParallelCommandGroup(
        FollowPathCustom(Robot.drivetrain, path1),
        Target(Robot.intake, Robot.elevator, config.Target.floor_down)
    ),
    ParallelCommandGroup(
        FollowPathCustom(Robot.drivetrain, path2),
        Target(Robot.intake, Robot.elevator, config.Target.mid)
    ),
    Target(Robot.intake, Robot.elevator, config.Target.high),
    score,
    set_cube,
    ParallelCommandGroup(
        FollowPathCustom(Robot.drivetrain, path3),
        Target(Robot.intake, Robot.elevator, config.Target.floor_down)
    ),
    ParallelCommandGroup(
        FollowPathCustom(Robot.drivetrain, path4),
        Target(Robot.intake, Robot.elevator, config.Target.mid)
    ),
    Target(Robot.intake, Robot.elevator, config.Target.high),
    score,
    Target(Robot.intake, Robot.elevator, config.Target.idle),
)

routine = AutoRoutine(auto)