from robotpy_toolkit_7407.utils import logger
from oi.keymap import Keymap
from robot_systems import Robot, Sensors
import command, math, config, constants
from commands2 import InstantCommand
logger.info("Hi, I'm OI!")

active_piece: config.GamePiece = config.GamePiece.cube

active_target: config.Target = config.Target.idle

active_grid: int = 1
class OI:
    @staticmethod
    def init() -> None:
        logger.info("Initializing OI...")
        
    def set_active_piece(piece: config.GamePiece):
        global active_piece
        active_piece = piece
        
    def set_active_target(target: config.Target):
        global active_target
        active_target = target
    
    def raise_active_grid():
        global active_grid
        active_grid += 1
        if active_grid > 9:
            active_grid = 9
            
    def lower_active_grid():
        global active_grid
        active_grid -= 1
        if active_grid < 1:
            active_grid = 1
    
    def no_active_grid():
        global active_grid
        active_grid = 0
        
    @staticmethod
    def map_controls():
        logger.info("Mapping controls...")
        
        Keymap.Target.CONE_ACTIVE.debounce(0.3).whenActive(InstantCommand(lambda: OI.set_active_piece(config.GamePiece.cone)))
        
        Keymap.Target.CUBE_ACTIVE.debounce(0.3).whenActive(InstantCommand(lambda: OI.set_active_piece(config.GamePiece.cube)))
        
        
        Keymap.Grid.RAISE_GRID.whenActive(InstantCommand(lambda: OI.raise_active_grid()))
        
        Keymap.Grid.LOWER_GRID.whenActive(InstantCommand(lambda: OI.lower_active_grid()))
        
        Keymap.Grid.NO_GRID.whenActive(InstantCommand(lambda: OI.no_active_grid()))
        
        # Keymap.Grid.AUTO_ALIGN.whenActive().whenInactive()
        
        
        Keymap.Target.SET_LOW.debounce(0.2).whenActive(InstantCommand(lambda: OI.set_active_target(config.Target.low)))
        
        Keymap.Target.SET_MIDDLE.debounce(0.2).whenActive(InstantCommand(lambda: OI.set_active_target(config.Target.mid)))
        
        Keymap.Target.SET_HIGH.debounce(0.2).whenActive(InstantCommand(lambda: OI.set_active_target(config.Target.high)))
        
        Keymap.Target.SET_SINGLE.debounce(0.2).whenActive(InstantCommand(lambda: OI.set_active_target(config.Target.single)))
        
        Keymap.Target.SET_DOUBLE.debounce(0.2).whenActive(InstantCommand(lambda: OI.set_active_target(config.Target.double)))
        
        Keymap.Target.SET_FLOOR.debounce(0.2).whenActive(InstantCommand(lambda: OI.set_active_target(config.Target.floor_up)))
        
        Keymap.Target.RUN_TARGET.whenActive(command.Target(Robot.intake, Robot.elevator, active_target, active_piece))\
            .whenInactive(command.Target(Robot.intake, Robot.elevator, config.Target.idle, active_piece))
        
        
        
        Keymap.Puncher.PUNCH_EXTEND.whenActive(command.ExtendPuncher(Robot.puncher))

        Keymap.Puncher.PUNCH_RETRACT.whenActive(command.RetractPuncher(Robot.puncher))
        
        
        
        Keymap.Drivetrain.RESET_GYRO.onTrue(command.DrivetrainZero(Robot.drivetrain)).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        Keymap.Drivetrain.X_MODE.onTrue(InstantCommand(lambda: Robot.drivetrain.x_mode()))
        
        Keymap.Drivetrain.AUTO_PICKUP.onTrue(command.LineupSwerve(Robot.drivetrain, Sensors.limeLight_B, 0))#.onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        # Keymap.Drivetrain.TEST_WRIST.whenPressed(command.SetCarriage(Robot.intake, math.radians(90), True, config.game_piece['cone'])).whenReleased(command.SetCarriage(Robot.intake, math.radians(0), False, config.game_piece['cone']))
        
        Keymap.Drivetrain.TEST_WRIST.whenPressed(command.SetElevator(Robot.elevator, 1)).whenReleased(command.SetElevator(Robot.elevator, 0))
        
