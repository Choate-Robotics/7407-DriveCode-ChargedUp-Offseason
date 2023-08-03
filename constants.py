
from wpimath.geometry import Pose3d, Translation3d, Rotation3d

# limelight offsets from robot origin (in meters)
limelight_offset: Pose3d = {
    "front": Pose3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0.0, 0.0, 0.0)),
    "back": Pose3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0.0, 0.0, 0.0)),
}
import math

from robotpy_toolkit_7407.utils.units import hour, m, mile, rad, rev, s
from wpimath.geometry import Pose3d, Rotation3d

from config import field_length, field_width
from units.SI import (
    inches_to_meters,
    meters,
    meters_per_second,
    meters_per_second_squared,
    radians_per_second,
    rotations,
    rotations_per_minute,
)

#TODO Change drivetrain track width
#TODO update drivetrain wheel speeds
#TODO update drivetrain gear ratio per meter

period = 0.03

# --- DRIVETRAIN ---

drivetrain_turn_gear_ratio = 12.8
drivetrain_move_gear_ratio = 6.86

#TODO: Change this
drivetrain_move_gear_ratio_as_rotations_per_meter = 1.0

track_width: meters = 0.50156
robot_length: meters = 0.495

# TODO Maybe change these
drivetrain_accel = True
drivetrain_max_vel: meters_per_second = (15 * mile / hour).asNumber(m / s)  # 15 11
drivetrain_max_accel_tele: meters_per_second_squared = (45 * mile / hour).asNumber(m / s)
drivetrain_max_target_accel: meters_per_second_squared = (
    1.5 * mile / hour
).asNumber(  # 10
    m / s
)
drivetrain_target_max_vel: meters_per_second = (2 * mile / hour).asNumber(m / s)  # 3
drivetrain_max_angular_vel: radians_per_second = (1 * rev / s).asNumber(rad / s)  # 5
drivetrain_max_correction_vel: radians_per_second = (2 * rev / s).asNumber(rad / s)
drivetrain_max_climb_vel: meters_per_second = (5 * mile / hour).asNumber(m / s)

ApriltagPositionDictRed = {
    1: Pose3d(
        (field_length - inches_to_meters * 610.77),
        (field_width - inches_to_meters * 42.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, 0),
    ),
    2: Pose3d(
        (field_length - inches_to_meters * 610.77),
        (field_width - inches_to_meters * 108.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, 0),
    ),
    3: Pose3d(
        (field_length - inches_to_meters * 610.77),
        (field_width - inches_to_meters * 174.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, 0),
    ),
    4: Pose3d(
        (field_length - inches_to_meters * 636.96),
        (field_width - inches_to_meters * 265.74),
        (inches_to_meters * 27.38),
        Rotation3d(0.0, 0.0, 0),
    ),
    5: Pose3d(
        (field_length - inches_to_meters * 14.25),
        (field_width - inches_to_meters * 265.74),
        (inches_to_meters * 27.38),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    6: Pose3d(
        (field_length - inches_to_meters * 40.45),
        (field_width - inches_to_meters * 174.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    7: Pose3d(
        (field_length - inches_to_meters * 40.45),
        (field_width - inches_to_meters * 108.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    8: Pose3d(
        (field_length - inches_to_meters * 40.45),
        (field_width - inches_to_meters * 42.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
}

ApriltagPositionDictBlue = {
    1: Pose3d(
        (inches_to_meters * 610.77),
        (inches_to_meters * 42.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    2: Pose3d(
        (inches_to_meters * 610.77),
        (inches_to_meters * 108.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    3: Pose3d(
        (inches_to_meters * 610.77),
        (inches_to_meters * 174.19),  # FIRST's diagram has a typo (it says 147.19)
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    4: Pose3d(
        (inches_to_meters * 636.96),
        (inches_to_meters * 265.74),
        (inches_to_meters * 27.38),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    5: Pose3d(
        (inches_to_meters * 14.25),
        (inches_to_meters * 265.74),
        (inches_to_meters * 27.38),
        Rotation3d(),
    ),
    6: Pose3d(
        (inches_to_meters * 40.45),
        (inches_to_meters * 174.19),  # FIRST's diagram has a typo (it says 147.19)
        (inches_to_meters * 18.22),
        Rotation3d(),
    ),
    7: Pose3d(
        (inches_to_meters * 40.45),
        (inches_to_meters * 108.19),
        (inches_to_meters * 18.22),
        Rotation3d(),
    ),
    8: Pose3d(
        (inches_to_meters * 40.45),
        (inches_to_meters * 42.19),
        (inches_to_meters * 18.22),
        Rotation3d(),
    ),
}

# Puncher Ratio
# 0.00764 m/r
# 1.53m
puncher_ratio = 0.00764
puncher_length = 1.53
puncher_extend = puncher_length / puncher_ratio
puncher_retract = 0
puncher_init_pos = 0