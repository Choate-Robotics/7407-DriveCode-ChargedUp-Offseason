import logging
from robotpy_toolkit_7407.command import SubsystemCommand
from wpimath.filter import SlewRateLimiter
from wpimath.controller import ProfiledPIDController, PIDController

import config
import constants
from subsystem import Drivetrain


def curve_abs(x):
    return x ** 2


def curve(x):
    if x < 0:
        return -curve_abs(-x)
    return curve_abs(x)


class DriveSwerveCustom(SubsystemCommand[Drivetrain]):
    driver_centric = True
    driver_centric_reversed = False

    def initialize(self) -> None:
        # self.ramp_limit_x = SlewRateLimiter(constants.drivetrain_max_accel_tele, -constants.drivetrain_max_accel_tele,
        #                                     0.0)
        # self.ramp_limit_y = SlewRateLimiter(constants.drivetrain_max_accel_tele, -constants.drivetrain_max_accel_tele,
        #                                     0.0)
        pass

    def execute(self) -> None:
        dx, dy, d_theta = (
            self.subsystem.axis_dx.value * (-1 if config.drivetrain_reversed else 1),
            self.subsystem.axis_dy.value * (-1 if config.drivetrain_reversed else 1),
            -self.subsystem.axis_rotation.value,
        )

        if abs(d_theta) < 0.11:
            d_theta = 0

        dx = curve(dx)
        dy = curve(dy)
        d_theta = curve(d_theta)

        dx *= self.subsystem.max_vel
        dy *= -self.subsystem.max_vel
        d_theta *= self.subsystem.max_angular_vel

        # if constants.drivetrain_accel:
        #     dx_scale = dx
        #     dy_scale = dy
        #
        #     dx = self.ramp_limit_x.calculate(dx)
        #     dy = self.ramp_limit_y.calculate(dy)
        #
        #     # deceleration
        #     if abs(dx) > abs(dx_scale):
        #         self.ramp_limit_x.reset(dx_scale)
        #
        #     if abs(dy) > abs(dy_scale):
        #         self.ramp_limit_y.reset(dy_scale)

        if config.driver_centric:
            self.subsystem.set_driver_centric((-dy, dx), -d_theta)
        elif self.driver_centric_reversed:
            self.subsystem.set_driver_centric((dy, -dx), d_theta)
        else:
            self.subsystem.set_robot_centric((dy, -dx), d_theta)

    def end(self, interrupted: bool) -> None:
        self.subsystem.n_front_left.set_motor_velocity(0)
        self.subsystem.n_front_right.set_motor_velocity(0)
        self.subsystem.n_back_left.set_motor_velocity(0)
        self.subsystem.n_back_right.set_motor_velocity(0)
        # self.ramp_limit_x.reset(0)
        # self.ramp_limit_y.reset(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False

class DrivetrainZero(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        print("ZEROING DRIVETRAIN")
        self.subsystem.gyro.reset_angle()
        self.subsystem.n_front_left.zero()
        self.subsystem.n_front_right.zero()
        self.subsystem.n_back_left.zero()
        self.subsystem.n_back_right.zero()

    def execute(self) -> None:
        ...

    def isFinished(self) -> bool:
        ...
        return True

    def end(self, interrupted: bool) -> None:
        logging.info("Successfully re-zeroed swerve pods.")
        ...

class BalanceDrivetrain(SubsystemCommand[Drivetrain]):
    '''
    Balances the drivetrain using the gyro. Assumes that the robot is on a flat surface.'''
    def __init__(self, subsystem: Drivetrain, speed: float = 0.5, timeout: float = 5.0):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.gyro = self.subsystem.gyro
        self.speed_percent = speed
        self.timeout = timeout
        self.pid = config.balance_pid
        self.pid.enableContinuousInput(-180, 180)
        self.pid.setTolerance(5)
        self.climbing: bool = False
        
    def initialize(self) -> None:
        self.pid.reset()
        self.pid.setGoal(0)
    
    def execute(self) -> None:
        if self.gyro.get_robot_pitch() < 20 and not self.climbing:
            self.pid.setGoal(30)
        else:
            self.climbing = True
            self.pid.setGoal(0)

        measurement = self.gyro.get_robot_pitch()
        calculate = self.pid.calculate(measurement)
        
        calculate = calculate / 90
        
        vel = self.speed_percent * calculate * self.subsystem.max_vel
        
        self.subsystem.set_driver_centric((0, vel), 0)
    
    def isFinished(self) -> bool:
        return self.pid.atGoal()
    
    def end(self, interrupted: bool) -> None:
        pass