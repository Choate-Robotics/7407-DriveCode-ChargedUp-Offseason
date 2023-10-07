
elevator_can_id = 11 # 11 can ID # Change later, this isn't accurate
magnetic_limit_switch_port = 0 # Change later, this is from last year
elevator_ramp_rate = .3
elevator_intake_threshold = .4 # multiply this by max rotations, this is the threshold for the elevator and intake to move at the same time



from robotpy_toolkit_7407.motors.rev_motors import SparkMaxConfig
from rev import CANSparkMax
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ProfiledPIDController
from wpilib import AnalogEncoder
import math

enable_pid_tuning = True  # change this to enable pid tuning

from robotpy_toolkit_7407.utils.units import hour, m, mile, rad, rev, s

from units.SI import (
    inches_to_meters,
    meters,
    meters_per_second,
    meters_per_second_squared,
    radians_per_second,
    rotations,
    rotations_per_minute,
)

#----Intake and Wrist----

left_intake_motor_id = 1  # correct motor id
right_intake_motor_id = 20  # correct motor id
intake_ramp_rate = .1
default_intake_speed = .85 # change this to the default intake speed
idle_intake_speed = .1
wrist_motor_id = 18  # correct motor id

INTAKE_CONFIG = SparkMaxConfig(k_P=1, k_I=1, k_D=1, k_F=0, output_range=(-1,1), idle_mode=CANSparkMax.IdleMode.kBrake)

WRIST_CONFIG = SparkMaxConfig(k_P=.31, k_I=0, k_D=.08, k_F=0, output_range=(-.9,.9), idle_mode=CANSparkMax.IdleMode.kBrake)
#.9

balance_pid = ProfiledPIDController(1, 1, 1, TrapezoidProfile.Constraints(1, 0))

disable_wrist_rotation = False  # change this to disable wrist rotation

class GamePiece:
    
    cone = 0
    cube = 1
    
class IntakeActive:
    
    kIn = 0
    kOut = 1
    kIdle = 2

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

class Target:
    
    high = {
        'length': 1,
        'angle': math.radians(-45),
        'goal': 'score',
    }
    mid = {
        'length': .6,
        'angle': math.radians(-45),
        'goal': 'score',
    }
    low = {
        'length': .45,
        'angle': math.radians(-50),
        'goal': 'score',
    }
    floor_up = {
        'length': 0,
        'angle': math.radians(110),
        'angle-cube': math.radians(110),
        'goal': 'pickup'
    }
    floor_down = {
        'length': 0,
        'angle': math.radians(175),
        'angle-cube': math.radians(152),
        'goal': 'pickup',
    }
    single = {
        'length': .1,
        'angle': math.radians(90),
        'angle-cube': math.radians(45),
        'goal': 'pickup',
    }
    double = {
        'length': 1,
        'angle': math.radians(-30),
        'angle-cube': math.radians(-20),
        'goal': 'pickup',
    }
    
    idle = {
        'length': .2,
        'angle': math.radians(0),
        'goal': 'idle',
    }
    
    
active_piece: GamePiece = GamePiece.cone

active_target: Target = Target.single

active_grid: int = 1


game_piece_targeting_constraints = {
    'cube': {
        'tx': [0,0],
        'ty': [0,0],
        'ta': [0,0]
    },
    'cone': {
        'tx': [-3.5, 3.5], #left to right
        'ty': [-12, -10], #top to bottom
        'ta': [12, 14] #area
    }
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

drivetrain_encoder_filtered: bool = True

# Field
field_length = 651.25 * inches_to_meters
# field_width = 315.5 * inches_to_meters
field_width = 8.075

# Drivetrain
front_left_move = 3
front_left_turn = 4
front_left_encoder = AnalogEncoder(3)
front_left_zeroed_pos = 0.56253 # * 360 * degrees_to_radians

front_right_move = 14
front_right_turn = 15
front_right_encoder = AnalogEncoder(0)
front_right_zeroed_pos = 0.27433 #* 360 * degrees_to_radians

back_left_move = 6
back_left_turn = 5
back_left_encoder = AnalogEncoder(1)
back_left_zeroed_pos = 0.79222 #* 360 * degrees_to_radians

back_right_move = 13
back_right_turn = 12
back_right_encoder = AnalogEncoder(2)
back_right_zeroed_pos = 0.61598 #* 360 * degrees_to_radians

calculated_max_vel = (15 * mile / hour).asNumber(m / s)  

# Sensors
gyro_id = 26

drivetrain_reversed = True

# Puncher
punch_left = 30
punch_right = 50
PUNCHER_CONFIG = SparkMaxConfig(1,0, .5, 0, (-1,1), CANSparkMax.IdleMode.kBrake)
