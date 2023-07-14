import time
import utils

from subsystem import Elevator
from robotpy_toolkit_7407.command import SubsystemCommand
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