from robotpy_toolkit_7407.utils import logger
from oi.keymap import Keymap
from robot_systems import Robot, Sensors
import command, math, config, constants, commands2
from sensors import ALeds
from commands2 import InstantCommand, StartEndCommand, RunCommand, RepeatCommand, ParallelCommandGroup
logger.info("Hi, I'm OI!")

class OI:
    @staticmethod
    def init() -> None:
        logger.info("Initializing OI...")
        OI.map_controls()
        
        
    @staticmethod
    def map_controls():
        
        
        logger.info("Mapping controls...")
        
        def set_active_route(route: config.Route):
            config.active_route = route
            print('route', f'{config.active_route=}')
        
        def set_active_station(station: config.Station):
            config.active_station = station
            print('station', f'{config.active_station=}')
        
        def set_active_target(target: config.Target):
            config.previous_target = config.active_target
            config.active_target = target
            print('target', f'{config.active_target=}')
            config.active_leds = (config.Type.KStatic(0, 255, 0), 1, 3)
        
        def raise_active_grid():
            active_grid = config.active_grid
            active_grid += 1
            if active_grid > 9:
                active_grid = 9
            config.active_grid = active_grid
            print('grid', active_grid)
            config.active_leds = (config.LedType.KBlink(0, 0, 255), 1, 1)
            
        def lower_active_grid():
            active_grid = config.active_grid
            active_grid -= 1
            if active_grid < 1:
                active_grid = 1
            config.active_grid = active_grid
            print('grid', active_grid)
            config.active_leds = (config.LedType.KBlink(0, 0, 255), 1, 5)
        
        def no_active_grid():
            active_grid = config.active_grid
            active_grid = 0
            config.active_grid = active_grid
            print('grid', 'off')
            
        def set_active_piece(piece: config.GamePiece):
            
            config.active_piece = piece
            if piece == config.GamePiece.cone:
                # yellow
                config.active_leds = (config.Type.KBlink(255, 255, 0), 1, 3)
                config.led_piece = config.led_cone
                print('piece', 'cone')
            else:
                # Purple
                config.active_leds = (config.Type.KBlink(255, 0, 255), 1, 3)
                config.led_piece = config.led_cube
                print('piece', 'cube')
                
        def set_led_shoot():
            config.active_leds = (config.Type.KBlink(255, 140, 0), 1, 3)
                
        def intake_drop():
            if config.active_piece == config.GamePiece.cone:
                Robot.intake.eject_cone()
            else:
                Robot.intake.eject_cube()
                
        def intake_idle():
            if config.active_piece == config.GamePiece.cone:
                config.active_leds = (config.Type.KStatic(255, 255, 0), 1, 3)
                Robot.intake.hold_cone()
            else:
                config.active_leds = (config.Type.KStatic(255, 0, 255), 1, 3)
                Robot.intake.hold_cube()
                
        def set_led_piece():
            config.active_leds = (config.led_piece, 1, 5)
            
        # def save_target():
        #     config.previous_target = config.active_target
            
        # def set_previous_target():
        #     config.active_target = config.previous_target
            
        set_idle = ParallelCommandGroup(
            command.Target(Robot.intake, Robot.elevator, config.Target.idle),
            InstantCommand(set_led_piece)
        )
        
        
        
        Keymap.Target.CONE_ACTIVE.debounce(0.05).onTrue(InstantCommand(lambda: set_active_piece(config.GamePiece.cone))).\
            onFalse(InstantCommand(set_led_piece))
        
        Keymap.Target.CUBE_ACTIVE.debounce(0.05).onTrue(InstantCommand(lambda: set_active_piece(config.GamePiece.cube))).\
            onFalse(InstantCommand(set_led_piece))
        
        
        Keymap.Grid.RAISE_GRID.onTrue(InstantCommand(raise_active_grid)).\
            onFalse(InstantCommand(set_led_piece))
        
        Keymap.Grid.LOWER_GRID.onTrue(InstantCommand(lower_active_grid)).\
            onFalse(InstantCommand(set_led_piece))
        
        Keymap.Grid.NO_GRID.onTrue(InstantCommand(no_active_grid)).\
            onFalse(InstantCommand(set_led_piece))
        
        
        Keymap.Station.SINGLE_STATION.onTrue(InstantCommand(lambda: set_active_station(config.Station.single))).\
            onFalse(InstantCommand(set_led_piece))
        
        Keymap.Station.DOUBLE_STATION_LEFT.onTrue(InstantCommand(lambda: set_active_station(config.Station.double_left))).\
            onFalse(InstantCommand(set_led_piece))
        
        Keymap.Station.DOUBLE_STATION_RIGHT.onTrue(InstantCommand(lambda: set_active_station(config.Station.double_right))).\
            onFalse(InstantCommand(set_led_piece))
        
        Keymap.Route.SET_STATION_ROUTE.onTrue(InstantCommand(lambda: set_active_route(config.Route.station))).\
            onFalse(InstantCommand(set_led_piece))
        
        Keymap.Route.SET_GRID_ROUTE.onTrue(InstantCommand(lambda: set_active_route(config.Route.grid))).\
            onFalse(InstantCommand(set_led_piece))
        
        Keymap.Route.SET_AUTO_ROUTE.onTrue(InstantCommand(lambda: set_active_route(config.Route.auto))).\
            onFalse(InstantCommand(set_led_piece))
        
        
        Keymap.Route.RUN_ROUTE.onTrue(command.RunRoute(Robot.drivetrain, Sensors.odometry))\
            .onFalse(command.DriveSwerveCustom(Robot.drivetrain).alongWith(InstantCommand(set_led_piece)))
        
        
        Keymap.Target.SET_LOW.debounce(0.05).onTrue(command.Target(Robot.intake, Robot.elevator, config.Target.low))\
            .onFalse(set_idle)
            
        
        Keymap.Target.SET_MIDDLE.debounce(0.05).onTrue(command.Target(Robot.intake, Robot.elevator, config.Target.mid))\
            .onFalse(set_idle)
        
        Keymap.Target.SET_HIGH.debounce(0.05).onTrue(command.Target(Robot.intake, Robot.elevator, config.Target.high))\
            .onFalse(set_idle)
        
        Keymap.Target.SET_SINGLE.debounce(0.1).onTrue(command.Target(Robot.intake, Robot.elevator, config.Target.single))\
            .onFalse(set_idle)
        
        Keymap.Target.SET_DOUBLE.debounce(0.05).onTrue(command.Target(Robot.intake, Robot.elevator, config.Target.double))\
            .onFalse(set_idle)
        
        Keymap.Target.SET_FLOOR.debounce(0.05).onTrue(command.Target(Robot.intake, Robot.elevator, config.Target.floor_down))\
            .onFalse(set_idle)
        
        Keymap.Target.RUN_SHOOT.onTrue(InstantCommand(set_led_shoot).alongWith(command.Target(Robot.intake, Robot.elevator, config.Target.shoot)))\
            .onFalse(set_idle)
        
        
        
        Keymap.Intake.DROP_PIECE.whileTrue(InstantCommand(intake_drop)).onFalse(InstantCommand(intake_idle))
        
        Keymap.Drivetrain.RESET_GYRO.onTrue(command.DrivetrainZero(Robot.drivetrain)).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        Keymap.Drivetrain.X_MODE.onTrue(InstantCommand(lambda: Robot.drivetrain.x_mode()))
        
        Keymap.Drivetrain.AUTO_PICKUP.onTrue(
            command.AutoPickup(Robot.drivetrain, Robot.intake, Robot.elevator, Sensors.limeLight_B)
            ).onFalse(
                command.DriveSwerveCustom(Robot.drivetrain)\
                    .alongWith(set_idle)
                )
            
        Keymap.Drivetrain.SQUARE_DRIVE.onTrue(
            command.SquareDrivetrain(Robot.drivetrain)
            ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        Keymap.Drivetrain.ZERO_ELEVATOR.onTrue(
            command.ZeroElevator(Robot.elevator)
        )
        
        
        # Keymap.Target.RUN_TARGET.onTrue(command.Target(Robot.intake, Robot.elevator))\
        #     .onFalse(command.Idle(Robot.intake, Robot.elevator).alongWith(InstantCommand(set_led_piece)))
