from builtins import type
from dataclasses import dataclass
from typing import Optional
import wpilib
import commands2
from wpimath.controller import PIDController
from rev import CANSparkMax, SparkMaxPIDController, REVLibError 

from robotpy_toolkit_7407.motor import PIDMotor
from robotpy_toolkit_7407.utils.units import rev, minute, radians, radians_per_second, rad, s, rotations_per_second, \
    rotations

from robotpy_toolkit_7407.unum import Unum

from robotpy_toolkit_7407.motors.ctre_motors import hundred_ms


@dataclass
class SparkMaxConfig:
    """
    Configuration for a SparkMax motor controller

    Args:
        kP: Proportional gain
        kI: Integral gain
        kD: Derivative gain
        kF: Feedforward gain
        output_range: The minimum and maximum output of the controller as (min: float, max: float)
        idle_mode: Whether to brake or coast when the motor is not moving
    """

    k_P: Optional[float] = None
    k_I: Optional[float] = None
    k_D: Optional[float] = None
    k_F: Optional[float] = None
    output_range: Optional[tuple[float, float]] = None
    idle_mode: Optional[CANSparkMax.IdleMode] = None


# rev_sensor_unit = Unum.unit("rev_sensor_u", rev / 4096, "rev sensor unit")
# rev_sensor_vel_unit = rev_sensor_unit / hundred_ms
# rev_sensor_accel_unit = rev_sensor_vel_unit / s

# k_sensor_pos_to_radians = rev.asNumber(rad)
# k_radians_to_sensor_pos = rad.asNumber(rev)
# k_sensor_vel_to_rad_per_sec = (rev / minute).asNumber(rad / s)
# k_rad_per_sec_to_sensor_vel = (rad / s).asNumber(rev / minute)

if wpilib.RobotBase.isSimulation():
    
    class Spark(wpilib.Spark):
        def __init__(self, channel: int) -> None:
            super().__init__(channel)
            
        def getLastError(self):
            return REVLibError.kOk
        
        def setClosedLoopRampRate(rate:float):
            print('Passing on setClosedLoopRampRate', rate)
            

    class SparkMaxRelativeEncoder:
        def __init__(self) -> None:
            self._velocity = 0
            self._position = 0

        def getVelocity(self):
            return self._velocity
        
        def getPosition(self):
            return self._position
            
    class SparkMaxPIDController():
        def __init__(self, motor: wpilib.Spark, sensor: SparkMaxRelativeEncoder) -> None:
            self._kp = 0
            self._ki = 0
            self._kd = 0
            self._ff = 0
            self._arbff = 0
            self._output_range = (-1, 1)
            self._motor = motor
            self._sensor = sensor
            self._internal_controller = PIDController(self._kp, self._ki, self._kd)
            self._measurement = self._sensor.getPosition()
            
        def setReference(self, reference: float, controlType: CANSparkMax.ControlType, arbFeedforward: float = 0):
            self._internal_controller.setSetpoint(reference)
            self._arbff = arbFeedforward
            if controlType == CANSparkMax.ControlType.kPosition:
                self._measurement = self._sensor.getPosition()
            elif controlType == CANSparkMax.ControlType.kVelocity:
                self._measurement = self._sensor.getVelocity()
            self.enable()
            
        def _getMeasurement(self) -> float:
            return self._sensor.getVelocity()
        
        def _useOutput(self, output: float, setpoint: float) -> None:
            
            
            main_output = output + self._ff + self._arbff
            
            if self._output_range is not None:
                main_output = max(self._output_range[0], min(self._output_range[1], main_output))
            
            self._motor.set(main_output)
            
        def setP(self, kP: float):
            self._kp = kP
            self._internal_controller.setP(kP)
            
        def setI(self, kI: float):
            self._ki = kI
            self._internal_controller.setI(kI)
            
        def setD(self, kD: float):
            self._kd = kD
            self._internal_controller.setD(kD)
            
            
        def setFF(self, kF: float):
            self._ff = kF

        def setOutputRange(self, min: float, max: float):
            self._output_range = (min, max)
        

    class SparkMax():
        def __init__(self, channel: int, inverted: bool = True, brushless: bool = True, config: SparkMaxConfig = None) -> None:
            
            self.id = channel
            self._inverted = inverted
            self._config = config
            
        def init(self):
            print('Simulating SparkMax', self.id)
            self.motor = Spark(self.id)
            self._encoder = SparkMaxRelativeEncoder()
            self.motor.setInverted(self._inverted)
            # print('Creating fake pid controller', self.id)
            self.pid_controller = SparkMaxPIDController(self.motor, self._encoder)
            self.encoder = self.getEncoder()
            # print('Setting config', self.id)
            self._set_config(self._config)
            # print('error validation for', self.id)
            if self.motor.getLastError() != REVLibError.kOk:
                print(self.motor.getLastError())
            else:
                print("SparkMax", self.id, "ok")

        def getEncoder(self):
            return self._encoder

        def setIdleMode(self, mode):
            pass
        
        def set_raw_output(self, x: float):
            """
            Sets the raw output of the motor controller

            Args:
                x (float): The output of the motor controller (between -1 and 1)
            """
            self.motor.set(x)
        
        def get_raw_output(self, x: float):
            """
            Sets the raw output of the motor controller

            Args:
                x (float): The output of the motor controller (between -1 and 1)
            """
            self.motor.get(x)

        def set_target_position(self, pos: rotations):
            """
            Sets the target position of the motor controller in rotations

            Args:
                pos (float): The target position of the motor controller in rotations
            """
            self.pid_controller.setReference(pos, CANSparkMax.ControlType.kPosition)

        def set_target_velocity(self, vel: rotations_per_second):  # Rotations per minute??
            """
            Sets the target velocity of the motor controller in rotations per second

            Args:
                vel (float): The target velocity of the motor controller in rotations per second
            """
            self.pid_controller.setReference(vel, CANSparkMax.ControlType.kVelocity)

        def get_sensor_position(self) -> rotations:
            """
            Gets the sensor position of the motor controller in rotations

            Returns:
                (rotations): The sensor position of the motor controller in rotations
            """
            return self.encoder.getPosition()

        def set_sensor_position(self, pos: rotations):
            """
            Sets the sensor position of the motor controller in rotations

            Args:
                pos (rotations): The sensor position of the motor controller in rotations
            """
            self.encoder.setPosition(pos)

        def get_sensor_velocity(self) -> rotations_per_second:
            """
            Gets the sensor velocity of the motor controller in rotations per second

            Returns:
                (rotations_per_second): The sensor velocity of the motor controller in rotations per second
            """
            return self.encoder.getVelocity()

        def _set_config(self, config: SparkMaxConfig):
            if config is None:
                print('No config', self.id)
                return
            if config.k_P is not None:
                # print('Setting P', config.k_P)
                self.pid_controller.setP(config.k_P)
            if config.k_I is not None:
                # print('Setting I', config.k_I)
                self.pid_controller.setI(config.k_I)
            if config.k_D is not None:
                # print('Setting D', config.k_D)
                self.pid_controller.setD(config.k_D)
            if config.k_F is not None:
                # print('Setting F', config.k_F)
                self.pid_controller.setFF(config.k_F)
            if config.output_range is not None:
                # print('Setting output range', config.output_range)
                self.pid_controller.setOutputRange(config.output_range[0], config.output_range[1])
            
        def getLastError(self):
            return REVLibError.kOk
else:
    from rev import SparkMaxRelativeEncoder

    class SparkMax(PIDMotor):
        """
        Wrapper class for the SparkMax motor controller
        """
        motor: CANSparkMax
        encoder: SparkMaxRelativeEncoder
        pid_controller: SparkMaxPIDController

        def __init__(self, can_id: int, inverted: bool = True, brushless: bool = True, config: SparkMaxConfig = None):
            """

            Args:
                can_id (int): The CAN ID of the motor controller
                inverted (bool, optional): Whether the motor is inverted. Defaults to True.
                brushless (bool, optional): Whether the motor is brushless. Defaults to True.
                config (SparkMaxConfig, None): The configuration for the motor controller. Defaults to None.
            """
            super().__init__()
            self._can_id = can_id
            self._inverted = inverted
            self._brushless = brushless
            self._config = config

        def init(self):
            """
            Initializes the motor controller, pid controller, and encoder
            """
            self.motor = CANSparkMax(
                self._can_id,
                CANSparkMax.MotorType.kBrushless if self._brushless else CANSparkMax.MotorType.kBrushed
            )
            self.motor.setInverted(self._inverted)
            self.pid_controller = self.motor.getPIDController()
            self.encoder = self.motor.getEncoder()
            self._set_config(self._config)
            if self.motor.getLastError() != REVLibError.kOk:
                print(self.motor.getLastError())
            else:
                print("SparkMax", self._can_id, "ok")

        def set_raw_output(self, x: float):
            """
            Sets the raw output of the motor controller

            Args:
                x (float): The output of the motor controller (between -1 and 1)
            """
            self.motor.set(x)
            
        def get_raw_output(self, x: float):
            """
            Sets the raw output of the motor controller

            Args:
                x (float): The output of the motor controller (between -1 and 1)
            """
            self.motor.get(x)

        def set_target_position(self, pos: rotations):
            """
            Sets the target position of the motor controller in rotations

            Args:
                pos (float): The target position of the motor controller in rotations
            """
            self.pid_controller.setReference(pos, CANSparkMax.ControlType.kPosition)

        def set_target_velocity(self, vel: rotations_per_second):  # Rotations per minute??
            """
            Sets the target velocity of the motor controller in rotations per second

            Args:
                vel (float): The target velocity of the motor controller in rotations per second
            """
            self.pid_controller.setReference(vel, CANSparkMax.ControlType.kVelocity)

        def get_sensor_position(self) -> rotations:
            """
            Gets the sensor position of the motor controller in rotations

            Returns:
                (rotations): The sensor position of the motor controller in rotations
            """
            return self.encoder.getPosition()

        def set_sensor_position(self, pos: rotations):
            """
            Sets the sensor position of the motor controller in rotations

            Args:
                pos (rotations): The sensor position of the motor controller in rotations
            """
            self.encoder.setPosition(pos)

        def get_sensor_velocity(self) -> rotations_per_second:
            """
            Gets the sensor velocity of the motor controller in rotations per second

            Returns:
                (rotations_per_second): The sensor velocity of the motor controller in rotations per second
            """
            return self.encoder.getVelocity()

        def _set_config(self, config: SparkMaxConfig):
            if config is None:
                return
            if config.k_P is not None:
                self.pid_controller.setP(config.k_P)
            if config.k_I is not None:
                self.pid_controller.setI(config.k_I)
            if config.k_D is not None:
                self.pid_controller.setD(config.k_D)
            if config.k_F is not None:
                self.pid_controller.setFF(config.k_F)
            if config.output_range is not None:
                self.pid_controller.setOutputRange(config.output_range[0], config.output_range[1])
            if config.idle_mode is not None:
                self.motor.setIdleMode(config.idle_mode)
