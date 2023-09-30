import config
import wpilib
from robotpy_toolkit_7407 import SubsystemCommand
from oi.keymap import Controllers
from subsystem import Intake
from robotpy_toolkit_7407.utils.units import radians
import constants


class SetIntake(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake, intake_active: bool, game_piece: dict):
        super().__init__(subsystem)
        self.intake_active = intake_active
        self.finished = False
        self.go_cube = game_piece["cube"]
        self.go_cone = game_piece["cone"]

    def initialize(self) -> None:
        """
        Sets the intake to the desired state
        :return:
        """
        if self.intake_active and self.go_cube:
            self.subsystem.grab_cube()
        elif self.intake_active and self.go_cone:
            self.subsystem.grab_cone()
        else:
            # change output dynamically to cone and cubes w/ obj variable to hold on to piece
            self.subsystem.set_lower_output(0.025)
            self.subsystem.set_upper_output(0.025)

    def execute(self) -> None:
        """
        Checks if the intake has detected a game piece
        :return:
        """
        if self.go_cube and self.subsystem.get_cube_detected():
            self.finished = True
            Controllers.OPERATOR_CONTROLLER.setRumble(wpilib.Joystick.RumbleType.kBothRumble, 0.5)
        elif self.go_cone and self.subsystem.get_cone_detected():
            self.finished = True
            Controllers.OPERATOR_CONTROLLER.setRumble(wpilib.Joystick.RumbleType.kBothRumble, 0.5)
        elif self.go_cube and self.subsystem.get_no_grab_cube_detected():
            self.finished = True
            Controllers.OPERATOR_CONTROLLER.setRumble(wpilib.Joystick.RumbleType.kBothRumble, 0.5)

    def isFinished(self) -> bool:
        """
        Checks if the intake has finished
        :return:
        """
        return self.finished and self.subsystem.is_at_angle()

    def end(self, interrupted: bool) -> None:
        """
        Stops the intake
        :return:
        """
        self.subsystem.stop_intake()
        Controllers.OPERATOR_CONTROLLER.setRumble(wpilib.Joystick.RumbleType.kBothRumble, 0)
        self.finished = False
        

