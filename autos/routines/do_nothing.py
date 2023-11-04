from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from autos.auto_routine import AutoRoutine
from robot_systems import Robot
from command import Target, ZeroElevator, FollowPathCustom, Path
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

# path1 = Path(
#     Pose2d(2.0496, 4.8022, -0.02115),
#     Pose2d(6.24661, 4.1777, -0.0616),
#     [],
#     config.calculated_max_vel,
#     constants.drivetrain_max_target_accel,
# )

auto = SequentialCommandGroup(
    # InstantCommand(Robot.intake.zero_wrist),
    WaitCommand(1),
    # FollowPathCustom(Robot.drivetrain, path1)
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