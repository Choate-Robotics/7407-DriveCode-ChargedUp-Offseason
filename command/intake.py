import config
import wpilib
from robotpy_toolkit_7407 import SubsystemCommand
from oi.keymap import Controllers
from subsystem import Intake
from robotpy_toolkit_7407.utils.units import radians
import constants, math
from commands2 import SequentialCommandGroup


class SetIntake(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake, intake_active: bool, game_piece: config.GamePiece):
        super().__init__(subsystem)
        self.intake_active = intake_active
        self.finished = False
        self.piece = ''
        if game_piece == config.GamePiece.cone:
            self.piece = 'cone'
        elif game_piece == config.GamePiece.cube:
            self.piece = 'cube'

    def initialize(self) -> None:
        """
        Sets the intake to the desired state
        :return:
        """
        if self.intake_active and self.piece == 'cube':
            self.subsystem.grab_cube()
        elif self.intake_active and self.piece == 'cone':
            self.subsystem.grab_cone()
        else:
            # change output dynamically to cone and cubes w/ obj variable to hold on to piece
            if self.piece == 'cone':
                self.subsystem.hold_cone()
            else:
                self.subsystem.hold_cube()
            

    def execute(self) -> None:
        """
        Checks if the intake has detected a game piece
        :return:
        """
        if self.piece == 'cube' and self.subsystem.get_cube_detected():
            self.finished = True
            Controllers.OPERATOR_CONTROLLER.setRumble(wpilib.Joystick.RumbleType.kBothRumble, 0.5)
        elif self.piece == 'cone' and self.subsystem.get_cone_detected():
            self.finished = True
            Controllers.OPERATOR_CONTROLLER.setRumble(wpilib.Joystick.RumbleType.kBothRumble, 0.5)
        elif self.piece == 'cube' and self.subsystem.get_no_grab_cube_detected():
            self.finished = True
            Controllers.OPERATOR_CONTROLLER.setRumble(wpilib.Joystick.RumbleType.kBothRumble, 0.5)

    def isFinished(self) -> bool:
        """
        Checks if the intake has finished
        :return:
        """
        return self.finished

    def end(self, interrupted: bool) -> None:
        """
        Stops the intake
        :return:
        """
        self.subsystem.stop_intake()
        Controllers.OPERATOR_CONTROLLER.setRumble(wpilib.Joystick.RumbleType.kBothRumble, 0)
        self.finished = False
        
        
class SetWrist(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake, wrist_angle: radians, game_piece: config.GamePiece, intake_active: config.IntakeActive):
        super().__init__(subsystem)
        self.wrist_angle = wrist_angle
        self.subsystem = subsystem
        self.game_piece = game_piece
        self.intake_active = intake_active
        
    def initialize(self):
        
        if self.intake_active == config.IntakeActive.kIn:
            self.wrist_angle -= math.radians(30)
        
        self.subsystem.set_wrist_angle(self.wrist_angle)
        
    def execute(self):
        pass
    
    def isFinished(self):
        return self.subsystem.is_at_angle(self.wrist_angle, math.radians(2))
    
    def end(self, interrupted):
        if interrupted:
            self.subsystem.set_wrist_angle(self.subsystem.get_wrist_angle())
    
    
class SetCarriage(SequentialCommandGroup):
    
    def __init__(self, subsystem: Intake, wrist_angle: radians, intake_active: config.IntakeActive, game_piece: config.GamePiece):
        super().__init__(
            SetWrist(subsystem, wrist_angle, game_piece, intake_active),
            SetIntake(subsystem, intake_active, game_piece)
        )
