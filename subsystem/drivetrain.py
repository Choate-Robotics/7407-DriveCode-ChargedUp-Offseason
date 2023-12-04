import math
from dataclasses import dataclass
from wpilib import AnalogEncoder
from wpimath.filter import LinearFilter

import ntcore

import rev
from ctre.sensors import CANCoder
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig

from robotpy_toolkit_7407.utils.units import (
    meters,
    meters_per_second,
    radians,
    radians_per_second,
)
from wpimath.geometry import Pose2d

import config
import constants
from oi.keymap import Keymap
from units.SI import degrees, meters_per_second_squared
from utils import SwerveDrivetrain, SwerveNode



TURN_CONFIG = SparkMaxConfig(
    0.22, 0, 0.003, 0.00015, (-0.5, 0.5), rev.CANSparkMax.IdleMode.kBrake
)
MOVE_CONFIG = SparkMaxConfig(
    0.00005, 0, 0.0004, 0.00017, idle_mode=rev.CANSparkMax.IdleMode.kBrake
)

# THE WHEELS NEED TO BE ZEROED FACING THE LEFT OR RIGHT.
# FOR GOOBER, BEVELS FACE AWAY FROM RIO

@dataclass
class SparkMaxSwerveNode(SwerveNode):
    m_move: SparkMax
    m_turn: SparkMax
    encoder: AnalogEncoder
    absolute_encoder_zeroed_pos: float = 0
    name: str = "DefaultNode"
    motor_reversed: bool = False
    analog_filter: LinearFilter = LinearFilter.movingAverage(20)

    def init(self):
        super().init()
        self.m_move.init()
        # self.m_move.motor.setClosedLoopRampRate(config.drivetrain_ramp_rate)
        self.m_turn.init()
        if self.m_move.motor.getLastError() == rev.REVLibError.kOk and self.m_turn.motor.getLastError() == rev.REVLibError.kOk:
            print(self.name, "init ok")
        elif self.m_move.motor.getLastError() != rev.REVLibError.kOk:
            print(self.name, "move error", self.m_move.motor.getLastError())
        else:
            print(self.name, "move ok")
        if self.m_turn.motor.getLastError() != rev.REVLibError.kOk:
            print(self.name, "turn error", self.m_turn.motor.getLastError())
        else:
            print(self.name, "turn ok")

    def initial_zero(self):
        self.m_turn.set_sensor_position(0)
        abs_encoder_position: float = self.encoder.getAbsolutePosition()
        if config.drivetrain_encoder_filtered:
            try:
                self.analog_filter.reset()
                for i in range(25):
                    self.analog_filter.calculate(abs_encoder_position)
                abs_encoder_position = self.analog_filter.calculate(abs_encoder_position)
            except:
                abs_encoder_position = self.encoder.getAbsolutePosition()
                
        
        encoder_difference: float = (abs_encoder_position ) - (self.absolute_encoder_zeroed_pos)
        
        if encoder_difference > .5:
            encoder_difference -= 1
        elif encoder_difference < -.5:
            encoder_difference += 1
            

        motor_change = encoder_difference * constants.drivetrain_turn_gear_ratio

        
            
        self.m_turn.set_sensor_position(-motor_change)
    
    def zero(self):
        '''
        Zeros the node'''
        self.initial_zero()
        self.m_turn.set_target_position(0)
        
        # self.m_turn.set_target_position(0)

    def raw_output(self, power):
        self.m_move.set_raw_output(power)

    def set_motor_angle(self, pos: radians):
        self.m_turn.set_target_position(
            (pos / (2 * math.pi)) * constants.drivetrain_turn_gear_ratio * -1
        )

    def get_current_motor_angle(self) -> radians:
        return (
            (self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio)
            * 2
            * math.pi * -1
        )

    def set_motor_velocity(self, vel: meters_per_second):
        self.m_move.set_target_velocity(vel * constants.drivetrain_move_gear_ratio * -1)

    def get_motor_velocity(self) -> radians_per_second:
        
        # sign = 1 if self.motor_reversed else -1
        
        return (
            self.m_move.get_sensor_velocity()
            / constants.drivetrain_move_gear_ratio * (-1)
        )

    def get_drive_motor_traveled_distance(self) -> meters:
        sensor_position = self.m_move.get_sensor_position()
        # print(self.name, sensor_position / constants.drivetrain_move_gear_ratio_as_rotations_per_meter * -1)
        return (
            (sensor_position
            / constants.drivetrain_move_gear_ratio_as_rotations_per_meter)
        )

    def get_turn_motor_angle(self) -> radians:
        return (
            (self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio)
            * 2
            * math.pi * -1
        )


class Drivetrain(SwerveDrivetrain):
    n_front_left = SparkMaxSwerveNode(
        SparkMax(config.front_left_move, config=MOVE_CONFIG),
        SparkMax(config.front_left_turn, config=TURN_CONFIG),
        config.front_left_encoder,
        absolute_encoder_zeroed_pos=config.front_left_zeroed_pos,
        name="n_front_left",
    )
    n_front_right = SparkMaxSwerveNode(
        SparkMax(config.front_right_move, config=MOVE_CONFIG),
        SparkMax(config.front_right_turn, config=TURN_CONFIG),
        config.front_right_encoder,
        absolute_encoder_zeroed_pos=config.front_right_zeroed_pos,
        name="n_front_right",
        # motor_reversed=True,
        
    )
    n_back_left = SparkMaxSwerveNode(
        SparkMax(config.back_left_move, config=MOVE_CONFIG),
        SparkMax(config.back_left_turn, config=TURN_CONFIG),
        config.back_left_encoder,
        absolute_encoder_zeroed_pos=config.back_left_zeroed_pos,
        name="n_back_left",
    )
    n_back_right = SparkMaxSwerveNode(
        SparkMax(config.back_right_move, config=MOVE_CONFIG),
        SparkMax(config.back_right_turn, config=TURN_CONFIG),
        config.back_right_encoder,
        absolute_encoder_zeroed_pos=config.back_right_zeroed_pos,
        name="n_back_right",
        # motor_reversed=True,
    )

    gyro: PigeonIMUGyro_Wrapper = PigeonIMUGyro_Wrapper(config.gyro_id)
    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
    track_width: meters = constants.track_width
    # max_vel: meters_per_second = constants.drivetrain_max_vel
    max_vel: meters_per_second = config.calculated_max_vel
    max_target_accel: meters_per_second_squared = constants.drivetrain_max_target_accel
    max_angular_vel: radians_per_second = constants.drivetrain_max_angular_vel
    deadzone_velocity: meters_per_second = 0.1
    deadzone_angular_velocity: radians_per_second = math.radians(5)
    start_angle: degrees = 0
    start_pose: Pose2d = Pose2d(
        0,
        0,
        math.radians(start_angle),
    )
    gyro_start_angle: radians = math.radians(start_angle)
    gyro_offset: radians = math.radians(0)

    def x_mode(self):
        self.n_front_left.set_motor_angle(math.radians(45))
        self.n_front_right.set_motor_angle(math.radians(-45))
        self.n_back_left.set_motor_angle(math.radians(-45))
        self.n_back_right.set_motor_angle(math.radians(45))
        
        
