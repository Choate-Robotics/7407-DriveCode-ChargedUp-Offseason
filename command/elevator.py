import time
import utils

from subsystem import Elevator
from robotpy_toolkit_7407.command import SubsystemCommand
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile


class ZeroElevator(SubsystemCommand[Elevator]):

    def __init__(self, subsystem: Elevator):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.start_time = None

    def initialize(self) -> None:
        """
        Zeros the elevator
        """
        # Why was this -0.05 in the old code, I am setting as zero now assuming it was a misconfiguration with the
        # hardware. If not, please switch back.
        self.subsystem.motor_extend.set_raw_output(0)
        self.start_time = time.time()

    def execute(self):
        ...

    def isFinished(self) -> bool:
        """
        Checks if the elevator is zeroed
        :return: Boolean
        """
        return (
                self.subsystem.elevator_bottom_sensor.get_value()
                or (time.time() - self.start_time) > 5
        )

    def end(self, interrupted=False) -> None:
        """
        Ends the zero process and checks if it is finished or interrupted
        :param interrupted: (OPTIONAL) Boolean
        """
        if not interrupted:
            self.subsystem.motor_extend.set_raw_output(0)
            self.subsystem.motor_extend.set_sensor_position(0)
            utils.logger.debug("ELEVATOR", "Elevator Successfully Zeroed.")
        else:
            utils.logger.debug("ELEVATOR", "Elevator Zero Command Interrupted.")


class SetElevator(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: Elevator, length: float):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.length = length
        self.constraints = TrapezoidProfile.Constraints(1, 1)
        self.pid = ProfiledPIDController(0.1, 0, 0, self.constraints)

    def initialize(self) -> None:
        self.pid.reset()
        self.pid.setGoal(self.length)
        self.subsystem.set_length(self.length)

    def execute(self):
        voltage = self.pid.calculate(self.subsystem.get_length())

        self.subsystem.set_voltage(voltage)

    def isFinished(self) -> bool:
        return abs(self.subsystem.get_length() - self.length) < 0.03

    def end(self, interrupted=False) -> None:
        if not interrupted:
            utils.logger.debug("ELEVATOR", "Elevator Successfully Set.")
        else:
            utils.logger.debug("ELEVATOR", "Elevator Set Command Interrupted.")
        self.subsystem.set_length(self.subsystem.get_length())
