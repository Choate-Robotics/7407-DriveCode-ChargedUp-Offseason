import config
import wpilib
from robotpy_toolkit_7407 import SubsystemCommand
from oi.keymap import Controllers
from subsystem import Intake
from robotpy_toolkit_7407.utils.units import radians
import constants, math, commands2
from commands2 import SequentialCommandGroup


class SetIntake(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake, intake_active: config.IntakeActive, game_piece: config.GamePiece):
        super().__init__(subsystem)
        self.intake_active = intake_active
        self.finished = False
        self.piece = game_piece

    def initialize(self) -> None:
        """
        Sets the intake to the desired state
        :return:
        """
        if self.intake_active == config.IntakeActive.kIn:
            if self.piece == config.GamePiece.cone:
                self.subsystem.grab_cone()
            else:
                self.subsystem.grab_cube()
        elif self.intake_active == config.IntakeActive.kOut: 
            if self.piece == config.GamePiece.cone:
                self.subsystem.eject_cone()
            else:
                self.subsystem.eject_cube()
        elif self.intake_active == config.IntakeActive.kShoot:
            if self.piece == config.GamePiece.cone:
                self.subsystem.shoot_cone()
            else:
                self.subsystem.shoot_cube()
        else:
            # change output dynamically to cone and cubes w/ obj variable to hold on to piece
            if self.piece == config.GamePiece.cone:
                self.subsystem.hold_cone()
            else:
                self.subsystem.hold_cube()
            

    def execute(self) -> None:
        """
        Checks if the intake has detected a game piece
        :return:
        """
        # if self.piece == 'cube' and self.subsystem.get_cube_detected():
        #     self.finished = True
        #     Controllers.OPERATOR_CONTROLLER.setRumble(wpilib.Joystick.RumbleType.kBothRumble, 0.5)
        # elif self.piece == 'cone' and self.subsystem.get_cone_detected():
        #     self.finished = True
        #     Controllers.OPERATOR_CONTROLLER.setRumble(wpilib.Joystick.RumbleType.kBothRumble, 0.5)
        # elif self.piece == 'cube' and self.subsystem.get_no_grab_cube_detected():
        #     self.finished = True
        #     Controllers.OPERATOR_CONTROLLER.setRumble(wpilib.Joystick.RumbleType.kBothRumble, 0.5)
        # if self.subsystem.get_avg_current() > 20:
        #     self.finished = True
        #     Controllers.OPERATOR_CONTROLLER.setRumble(wpilib.Joystick.RumbleType.kBothRumble, 1)
        #     Controllers.DRIVER_CONTROLLER.setRumble(wpilib.Joystick.RumbleType.kBothRumble, 1)

    def isFinished(self) -> bool:
        """
        Checks if the intake has finished
        :return:
        """
        return True

    def end(self, interrupted: bool) -> None:
        """
        Stops the intake
        :return:
        """
        if self.intake_active == config.IntakeActive.kOut:
                self.subsystem.stop()
        Controllers.OPERATOR_CONTROLLER.setRumble(wpilib.Joystick.RumbleType.kBothRumble, 0)
        Controllers.DRIVER_CONTROLLER.setRumble(wpilib.Joystick.RumbleType.kBothRumble, 0)
        self.finished = False
        
class ZeroWrist(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.zero_wrist()
        
    def execute(self) -> None:
        pass
    
    def isFinished(self) -> bool:
        return self.subsystem.is_at_angle(0)
    
    def end(self, interrupted):
        if not interrupted:
            self.subsystem.wrist_zeroed = True
        
                
class SetWrist(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake, wrist_angle: radians, game_piece: config.GamePiece, intake_active: config.IntakeActive):
        super().__init__(subsystem)
        self.wrist_angle = wrist_angle
        self.subsystem = subsystem
        self.game_piece = game_piece
        self.intake_active = intake_active
        
    def initialize(self):
        
        
        self.subsystem.set_wrist_angle(self.wrist_angle)
        
    def execute(self):
        pass
    
    def isFinished(self):
        return self.subsystem.is_at_angle(self.wrist_angle, math.radians(2))
    
    def end(self, interrupted):
        self.subsystem.set_wrist_angle(self.subsystem.get_wrist_angle())
    
class SetCarriage(SequentialCommandGroup):
    
    def __init__(self, subsystem: Intake, wrist_angle: radians, intake_active: config.IntakeActive, game_piece: config.GamePiece):
        super().__init__(
            SetIntake(subsystem, config.IntakeActive.kIdle, game_piece),
            SetWrist(subsystem, wrist_angle, game_piece, intake_active),
            SetIntake(subsystem, intake_active, game_piece)
        )
