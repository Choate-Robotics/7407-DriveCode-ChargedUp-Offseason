from robotpy_toolkit_7407.utils import logger
from oi.keymap import Keymap
from robot_systems import Robot, Sensors
import command, math, config, constants, commands2
from sensors import ALeds
from commands2 import InstantCommand, StartEndCommand, RunCommand, RepeatCommand
logger.info("Hi, I'm OI!")

class OI:
    @staticmethod
    def init() -> None:
        logger.info("Initializing OI...")
        
        
        
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
                
        def set_led_shoot(self):
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
            
        def save_target():
            config.previous_target = config.active_target
            
        def set_previous_target():
            config.active_target = config.previous_target
        
        
        Keymap.Target.CONE_ACTIVE.debounce(0.05).whenActive(InstantCommand(lambda: set_active_piece(config.GamePiece.cone))).\
            whenInactive(InstantCommand(set_led_piece))
        
        Keymap.Target.CUBE_ACTIVE.debounce(0.05).whenActive(InstantCommand(lambda: set_active_piece(config.GamePiece.cube))).\
            whenInactive(InstantCommand(set_led_piece))
        
        
        Keymap.Grid.RAISE_GRID.whenActive(InstantCommand(raise_active_grid)).\
            whenInactive(InstantCommand(set_led_piece))
        
        Keymap.Grid.LOWER_GRID.whenActive(InstantCommand(lower_active_grid)).\
            whenInactive(InstantCommand(set_led_piece))
        
        Keymap.Grid.NO_GRID.whenActive(InstantCommand(no_active_grid)).\
            whenInactive(InstantCommand(set_led_piece))
        
        
        Keymap.Station.SINGLE_STATION.whenActive(InstantCommand(lambda: set_active_station(config.Station.single))).\
            whenInactive(InstantCommand(set_led_piece))
        
        Keymap.Station.DOUBLE_STATION_LEFT.whenActive(InstantCommand(lambda: set_active_station(config.Station.double_left))).\
            whenInactive(InstantCommand(set_led_piece))
        
        Keymap.Station.DOUBLE_STATION_RIGHT.whenActive(InstantCommand(lambda: set_active_station(config.Station.double_right))).\
            whenInactive(InstantCommand(set_led_piece))
        
        Keymap.Route.SET_STATION_ROUTE.whenActive(InstantCommand(lambda: set_active_route(config.Route.station))).\
            whenInactive(InstantCommand(set_led_piece))
        
        Keymap.Route.SET_GRID_ROUTE.whenActive(InstantCommand(lambda: set_active_route(config.Route.grid))).\
            whenInactive(InstantCommand(set_led_piece))
        
        Keymap.Route.SET_AUTO_ROUTE.whenActive(InstantCommand(lambda: set_active_route(config.Route.auto))).\
            whenInactive(InstantCommand(set_led_piece))
        
        
        # Keymap.Route.RUN_ROUTE.whenActive(command.RunRoute(Robot.drivetrain, Sensors.odometry))\
        #     .onFalse(command.DriveSwerveCustom(Robot.drivetrain).alongWith(InstantCommand(set_led_piece)))
        
        # Keymap.Grid.AUTO_ALIGN.whenActive().whenInactive()
        
        
        Keymap.Target.SET_LOW.debounce(0.05).whenActive(InstantCommand(lambda: set_active_target(config.Target.low))).\
            whenInactive(InstantCommand(set_led_piece))
        
        Keymap.Target.SET_MIDDLE.debounce(0.05).whenActive(InstantCommand(lambda: set_active_target(config.Target.mid))).\
            whenInactive(InstantCommand(set_led_piece))
        
        Keymap.Target.SET_HIGH.debounce(0.05).whenActive(InstantCommand(lambda: set_active_target(config.Target.high))).\
            whenInactive(InstantCommand(set_led_piece))
        
        Keymap.Target.SET_SINGLE.debounce(0.2).whenActive(InstantCommand(lambda: set_active_target(config.Target.single))).\
            whenInactive(InstantCommand(set_led_piece))
        
        Keymap.Target.SET_DOUBLE.debounce(0.05).whenActive(InstantCommand(lambda: set_active_target(config.Target.double))).\
            whenInactive(InstantCommand(set_led_piece))
        
        Keymap.Target.SET_FLOOR.debounce(0.05).whenActive(InstantCommand(lambda: set_active_target(config.Target.floor_down))).\
            whenInactive(InstantCommand(set_led_piece))
        
        Keymap.Target.RUN_SHOOT.onTrue(InstantCommand(set_led_shoot).alongWith(command.Shoot(Robot.intake, Robot.elevator))).\
            onFalse(command.Idle(Robot.intake, Robot.elevator).alongWith(InstantCommand(set_led_piece)))
        
        Keymap.Target.RUN_TARGET.onTrue(command.Target(Robot.intake, Robot.elevator))\
            .onFalse(command.Idle(Robot.intake, Robot.elevator).alongWith(InstantCommand(set_led_piece)))
        
        Keymap.Intake.DROP_PIECE.whileTrue(InstantCommand(intake_drop)).onFalse(InstantCommand(intake_idle))
        
        
        # Keymap.Puncher.PUNCH_EXTEND.whenActive(command.ExtendPuncher(Robot.puncher))

        # Keymap.Puncher.PUNCH_RETRACT.whenActive(command.RetractPuncher(Robot.puncher))
        
        
        
        Keymap.Drivetrain.RESET_GYRO.onTrue(command.DrivetrainZero(Robot.drivetrain)).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        Keymap.Drivetrain.X_MODE.onTrue(InstantCommand(lambda: Robot.drivetrain.x_mode()))
        
        Keymap.Drivetrain.AUTO_PICKUP.onTrue(
            command.AutoPickup(Robot.drivetrain, Robot.intake, Robot.elevator, Sensors.limeLight_B, config.GamePiece.cube)
            ).onFalse(
                command.DriveSwerveCustom(Robot.drivetrain)\
                    .alongWith(command.Idle(Robot.intake, Robot.elevator))\
                    .alongWith(InstantCommand(set_led_piece))\
                    # .alongWith(InstantCommand(lambda: Sensors.limeLight_B.set_pipeline_mode(config.LimelightPipeline.feducial)))
                )
            
        Keymap.Drivetrain.SQUARE_DRIVE.onTrue(
            command.SquareDrivetrain(Robot.drivetrain)
            ).onFalse(command.DriveSwerveCustom(Robot.drivetrain))
        
        Keymap.Drivetrain.ZERO_ELEVATOR.onTrue(
            command.ZeroElevator(Robot.elevator)
        )
        
        # Keymap.Drivetrain.TEST_WRIST.whenPressed(command.SetCarriage(Robot.intake, math.radians(90), True, config.game_piece['cone'])).whenReleased(command.SetCarriage(Robot.intake, math.radians(0), False, config.game_piece['cone']))
        
        # Keymap.Drivetrain.TEST_WRIST.whenPressed(command.SetElevator(Robot.elevator, 1)).whenReleased(command.SetElevator(Robot.elevator, 0))
        
