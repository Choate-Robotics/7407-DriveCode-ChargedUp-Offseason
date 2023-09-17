
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig

left_intake_motor_id = 15  # change this to the left intake motor id
right_intake_motor_id = 16  # change this to the right intake motor id
default_intake_speed = 0.5  # change this to the default intake speed
wrist_motor_id = 17  # change this to the wrist motor id
INTAKE_CONFIG = SparkMaxConfig(k_P=1, k_I=1, k_D=1, k_F=1)
WRIST_CONFIG = SparkMaxConfig(k_P=1, k_I=1, k_D=1, k_F=1)
disable_wrist_rotation = False  # change this to disable wrist rotation
game_piece = {
    'cube': 1,
    'cone': 0
}

limelight_pipeline: int = {
    'retroreflective': 0,
    'feducial': 1,
    'neural': 2
}

limelight_led_mode: int = {
    'pipeline_default': 0,
    'force_off': 1,
    'force_blink': 2,
    'force_on': 3
    
}

team:int = {
    'red': 0,
    'blue': 1
}

from units.SI import (
    inches_to_meters,
    degrees_to_radians
)

driver_centric: bool = True

# Field
field_length = 651.25 * inches_to_meters
# field_width = 315.5 * inches_to_meters
field_width = 8.075

# Drivetrain
front_left_move = 16
front_left_turn = 15
front_left_encoder = 24
front_left_zeroed_pos = 174.638 * degrees_to_radians

front_right_move = 14
front_right_turn = 13
front_right_encoder = 23
front_right_zeroed_pos = 282.304 * degrees_to_radians

back_left_move = 3
back_left_turn = 4
back_left_encoder = 21
back_left_zeroed_pos = 313.769 * degrees_to_radians

back_right_move = 5
back_right_turn = 6
back_right_encoder = 22
back_right_zeroed_pos = 136.58 * degrees_to_radians

# Sensors
gyro_id = 20

drivetrain_reversed = False

