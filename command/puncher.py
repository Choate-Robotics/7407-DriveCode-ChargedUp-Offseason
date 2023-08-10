from robotpy_toolkit_7407.command import SubsystemCommand
from subsystem import Puncher

class RetractPuncher(SubsystemCommand[Puncher]):


    def initialize(self) -> None:
        self.subsystem.retract()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return abs(self.subsystem.getMotorPosition - self.subsystem.getTargetPosition) < self.subsystem.tolerance
    
    def end(self, interrupted: bool) -> None:
        pass


class ExtendPuncher(SubsystemCommand[Puncher]):

    def initialize(self) -> None:
        self.subsystem.extend()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return abs(self.subsystem.getMotorPosition - self.subsystem.getTargetPosition) < self.subsystem.tolerance
    
    def end(self, interrupted: bool) -> None:
        pass