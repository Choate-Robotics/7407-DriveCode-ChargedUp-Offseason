from wpilib import Timer
import utils

from subsystem import Elevator
from robotpy_toolkit_7407.command import SubsystemCommand
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
import ntcore, constants, config
import commands2


class ZeroElevator(SubsystemCommand[Elevator]):

    def __init__(self, subsystem: Elevator):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.start: Timer

    def initialize(self) -> None:
        """
        Zeros the elevator
        """
        self.subsystem.zeroed = False
        ntcore.NetworkTableInstance.getDefault().getTable('Arm Voltage').putBoolean('zeroed', self.subsystem.zeroed)
        # Why was this -0.05 in the old code, I am setting as zero now assuming it was a misconfiguration with the
        # hardware. If not, please switch back.
        self.subsystem.motor_extend.set_raw_output(-0.1)
        self.time = Timer()
        self.time.start()

    def execute(self):
        ...

    def isFinished(self) -> bool:
        """
        Checks if the elevator is zeroed
        :return: Boolean
        """
        return (
                self.subsystem.elevator_bottom_sensor.get_value()
                or self.time.hasElapsed(5)
        )

    def end(self, interrupted=False) -> None:
        """
        Ends the zero process and checks if it is finished or interrupted
        :param interrupted: (OPTIONAL) Boolean
        """
        config.calculated_max_vel = constants.drivetrain_max_vel
        if not interrupted:
            self.subsystem.zeroed = True
            ntcore.NetworkTableInstance.getDefault().getTable('Arm Voltage').putBoolean('zeroed', self.subsystem.zeroed)
            self.subsystem.stop()
            self.subsystem.motor_extend.set_sensor_position(0)


class SetElevator(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: Elevator, length: float, force: bool = False):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.length = length
        # self.constraints = TrapezoidProfile.Constraints(12, 6)
        # self.pid = ProfiledPIDController(6, 0, 0.01, self.constraints)
        self.force = force
        self.goal: float

    def initialize(self) -> None:
        
        if self.length > 1:
            self.length = 1
        elif self.length < 0:
            self.length = 0
        
        # if self.subsystem.zeroed == False and self.force == False:
        #     commands2.CommandScheduler.getInstance().schedule(ZeroElevator(self.subsystem))
        #     return
        
        # self.pid.reset(self.subsystem.get_length())
        
            
        self.goal = self.length * constants.elevator_max_rotation
        
        # ntcore.NetworkTableInstance.getDefault().getTable("Arm Voltage").putNumber("goal", self.goal)
            
        # self.pid.setGoal(self.goal)
        self.subsystem.set_length(self.goal)

    def execute(self):
        
        if self.subsystem.elevator_bottom_sensor.get_value():
            self.subsystem.motor_extend.set_sensor_position(0)
            
        if self.subsystem.get_length() > constants.elevator_speed_threshold:
            #limit the speed more the higher the elevator is
            config.calculated_max_vel = constants.drivetrain_max_vel * max((1 - self.subsystem.get_length() / constants.elevator_max_rotation),.1)
        else:
            config.calculated_max_vel = constants.drivetrain_max_vel
        # ntcore.NetworkTableInstance.getDefault().getTable("Arm Voltage").putNumber("position", self.subsystem.get_length())
        
        # voltage = self.pid.calculate(self.subsystem.get_length())
        # ntcore.NetworkTableInstance.getDefault().getTable("Arm Voltage").putNumber("voltage", voltage)

        # if voltage < 0:
        #     voltage = max(voltage, -8)
        # else:
        #     voltage = min(voltage, 8)

        # self.subsystem.set_voltage(voltage)

    def isFinished(self) -> bool:
        return abs(self.subsystem.get_length() - (self.goal)) < 0.5

    def end(self, interrupted=False) -> None:
        # if not interrupted:
        #     utils.logger.debug("ELEVATOR", "Elevator Successfully Set.")
        # else:
        #     utils.logger.debug("ELEVATOR", "Elevator Set Command Interrupted.")
        self.subsystem.set_length(self.subsystem.get_length())
