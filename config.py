
DEBUG_MODE = True

elevator_can_id = 11 # 11 can ID # Change later, this isn't accurate
magnetic_limit_switch_port = 0 # Change later, this is from last year
elevator_ramp_rate = .5 # .3
elevator_intake_threshold = .4 # multiply this by max rotations, this is the threshold for the elevator and intake to move at the same time
elevator_wall_threshold = .3 # multiply this by max rotations, this is the threshold for the elevator and intake to move at the same time when the robot is against the wall
elevator_auto_position = .2

from robotpy_toolkit_7407.motors.rev_motors import SparkMaxConfig
from rev import CANSparkMax
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ProfiledPIDController
from wpilib import AnalogEncoder
import math
from wpimath.geometry import Translation2d
# from sensors import ALeds

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

elevator_leds_id = 0
elevator_leds_size = 39

#----Intake and Wrist----

left_intake_motor_id = 1  # correct motor id
right_intake_motor_id = 20  # correct motor id
intake_ramp_rate = .1
default_intake_speed = .35 # change this to the default intake speed
idle_intake_speed = .25
wrist_motor_id = 18  # correct motor id
intake_current_threshold = 30 # change this to the intake current threshold

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
    kShoot = 3

class LimelightPipeline:
    
    feducial = 0.0
    neural = 1.0
    retroreflective = 2.0

limelight_led_mode: int = {
    'pipeline_default': 0,
    'force_off': 1,
    'force_blink': 2,
    'force_on': 3
    
}

class Target:
    
    high = {
        'length': 1,
        'angle': math.radians(-40),
        'goal': 'score',
    }
    mid = {
        'length': .7,
        'angle': math.radians(-50),
        'goal': 'score',
    }
    low = {
        'length': .45,
        'angle': math.radians(-50),
        'goal': 'score',
    }
    shoot = {
        'length': .3,
        'length-cube': .25,
        'angle': math.radians(0),
        'angle-cube': math.radians(45),
        'goal': 'shoot'
    }
    floor_up = {
        'length': 0,
        'length-cube': .07,
        'angle': math.radians(110),
        'angle-cube': math.radians(140),
        'goal': 'pickup'
    }
    floor_down = {
        'length': 0,
        'length-cube': .08,
        'angle': math.radians(175),
        'angle-cube': math.radians(155),
        'goal': 'pickup',
    }
    single = {
        'length': .4,
        'length-cube': .365,
        'angle': math.radians(145),
        'angle-cube': math.radians(0),
        'wall': False,
        'wall-cube': False,
        'goal': 'pickup',
    }
    double = {
        'length': 1,
        'length-cube': .9,
        'angle': math.radians(-42),
        'angle-cube': math.radians(-33),
        'goal': 'pickup',
    }
    
    idle = {
        'length': .2,
        'angle': math.radians(0),
        'goal': 'idle',
    }
    
pickup_wall: bool = False
    
class Team:
        
    red = 0
    blue = 1
    
class Station:
    
    single = 0
    double_left = 1
    double_right = 2
    auto = 3
    
class Route:
    
    grid = 0
    station = 1
    auto = 2

class Type():
        
        def KStatic(r, g, b):
            return {
                'type': 1,
                'color': {
                    'r': r,
                    'g': g,
                    'b': b
                }
            }
        
        def KRainbow():
            return {
                'type': 2
            }
        
        def KTrack(r1, g1, b1, r2, g2, b2):
            return {
                'type': 3,
                'color': {
                    'r1': r1,
                    'g1': g1,
                    'b1': b1,
                    'r2': r2,
                    'g2': g2,
                    'b2': b2
                }
            }
        
        def KBlink(r,g,b):
            return {
                'type': 4,
                'color': {
                    'r': r,
                    'g': g,
                    'b': b
                }
            }
            
        def KLadder(typeA,typeB,percent,speed):
            return {
                'type': 5,
                'percent': percent, # 0-1
                'typeA': typeA,
                'typeB': typeB,
                'speed': speed
            }
        
    
class LedType(Type):
    
    def __init__():
        super().__init__()
        

auto_target: bool = False
    
active_piece: GamePiece = GamePiece.cone

active_target: Target = Target.single

previous_target: Target = Target.single

active_grid: int = 1

active_station: Station = Station.single

active_team: Team = Team.blue

active_route: Route = Route.grid

active_leds: tuple[LedType, float, float] = (LedType.KStatic(255, 0, 0), 1, 5)

led_cone: LedType = LedType.KStatic(255, 255, 0)

led_cube: LedType = LedType.KStatic(255, 0, 255)

led_piece: LedType = led_cone

led_elevator: LedType = LedType.KRainbow()

auto_led_elevator: bool = True

game_piece_targeting_constraints = {
    'cube': {
        'tx': [-3.5,-3.5],
        'ty': [-12,-10],
        'ta': [12,14]
    },
    'cone': {
        'tx': [-3.5, 3.5], #left to right
        'ty': [-12, -10], #top to bottom
        'ta': [12, 14] #area
    }
}



from units.SI import (
    inches_to_meters,
    degrees_to_radians
)

driver_centric: bool = True

drivetrain_encoder_filtered: bool = True

# Field
field_length = 651.25 * inches_to_meters # 54 ft 3.25 in
# field_width = 315.5 * inches_to_meters
# field_width = 8.075 
field_width = 315.25 * inches_to_meters # 26 ft 3.25 in

# Drivetrain
front_left_move = 3
front_left_turn = 4
front_left_encoder = AnalogEncoder(3)
front_left_zeroed_pos = 0.558524 #0.56253 # * 360 * degrees_to_radians

front_right_move = 14
front_right_turn = 15
front_right_encoder = AnalogEncoder(0)
front_right_zeroed_pos = 0.770884 # 0.27433 #* 360 * degrees_to_radians

back_left_move = 6
back_left_turn = 5
back_left_encoder = AnalogEncoder(1)
back_left_zeroed_pos = 0.770884 #0.79222 #* 360 * degrees_to_radians

back_right_move = 13
back_right_turn = 12
back_right_encoder = AnalogEncoder(2)
back_right_zeroed_pos = 0.113321 # 0.61598 #* 360 * degrees_to_radians

calculated_max_vel = (20 * mile / hour).asNumber(m / s)  
calculated_max_angular_vel: radians_per_second = (1 * rev / s).asNumber(rad / s)  # 5
drivetrain_ramp_rate = 0.1 # seconds from 0 to full speed
# Sensors
gyro_id = 26

drivetrain_reversed = True

# Puncher
punch_left = 30
punch_right = 50
PUNCHER_CONFIG = SparkMaxConfig(1,0, .5, 0, (-1,1), CANSparkMax.IdleMode.kBrake)
