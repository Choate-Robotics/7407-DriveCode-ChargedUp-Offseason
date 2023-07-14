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
