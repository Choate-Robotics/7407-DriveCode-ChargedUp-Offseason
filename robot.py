import commands2
import ctre
import wpilib
import command
import config
import constants
import autos
# from autos.auto_routine import AutoRoutine
from robot_systems import Robot, Sensors, LEDs
from sensors import LimelightController, FieldOdometry
import utils
from oi.OI import OI
import ntcore
import math

class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.previous_led = None
        self.detected_c = False
        self.team: wpilib.SendableChooser
        self.sensor_odometry: wpilib.SendableChooser
        self.auto: wpilib.SendableChooser
        self.debug: wpilib.SendableChooser

    def robotInit(self):

        # Initialize subsystems

        # Initialize Operator Interface
        
        period = .05
        commands2.CommandScheduler.getInstance().setPeriod(period)
        
        Robot.drivetrain.init()
        Robot.intake.init()
        Robot.elevator.init()
        LEDs.elevator.init()
        Sensors.limeLight_F.init()
        Sensors.limeLight_B.init()
        
        # for i in range(15):
        #     Robot.drivetrain.n_front_left.initial_zero()
        #     Robot.drivetrain.n_front_right.initial_zero()
        #     Robot.drivetrain.n_back_left.initial_zero()
        #     Robot.drivetrain.n_back_right.initial_zero()
        Sensors.l_c = LimelightController([Sensors.limeLight_F, Sensors.limeLight_B])
        Sensors.odometry = FieldOdometry(Robot.drivetrain, Sensors.l_c)
        Sensors.gyro = Robot.drivetrain.gyro
        
        OI.init()
        OI.map_controls()
        
        # Team
        self.team = wpilib.SendableChooser()
        
        self.team.setDefaultOption("Red", config.Team.red)
        self.team.addOption("Blue", config.Team.blue)
        wpilib.SmartDashboard.putData("Team", self.team)
        
        # Sensor Odometry
        self.sensor_odometry = wpilib.SendableChooser()
        self.sensor_odometry.setDefaultOption('On', True)
        self.sensor_odometry.addOption('Off', False)
        wpilib.SmartDashboard.putData('Sensor Odometry', self.sensor_odometry)
        
        if self.team.getSelected() == config.Team.blue:
            config.active_team = config.Team.blue
        else:
            config.active_team = config.Team.red
            
        # Autos
        self.auto = wpilib.SendableChooser()
        
        self.auto.setDefaultOption("Do Nothing", autos.do_nothing)
        self.auto.addOption('One Piece', autos.one_piece)
        self.auto.addOption('One Piece Drive', autos.one_piece_drive)
        wpilib.SmartDashboard.putData("Auto", self.auto)
        
        # Debug
        self.debug = wpilib.SendableChooser()
        self.debug.setDefaultOption('Off', False)
        self.debug.addOption('On', True)
        
        wpilib.SmartDashboard.putData('Debug', self.debug)
        
        Robot.drivetrain.n_back_left.initial_zero()
        Robot.drivetrain.n_back_right.initial_zero()
        Robot.drivetrain.n_front_left.initial_zero()
        Robot.drivetrain.n_front_right.initial_zero()
        
    def robotPeriodic(self):
        
        config.DEBUG_MODE = self.debug.getSelected()
        
        elevator_percentage = Robot.elevator.get_length() / constants.elevator_max_rotation
        # print('elevator percent:', elevator_percentage)
        if self.previous_led != config.active_leds or config.auto_led_elevator:
            if config.auto_led_elevator:
                LEDs.elevator.setLED(
                    config.LedType.KLadder(config.active_leds[0], config.led_elevator, elevator_percentage, 5),
                    config.active_leds[1],
                    config.active_leds[2]
                    )
            else:
                LEDs.elevator.setLED(*config.active_leds)
            self.previous_led = config.active_leds
        
        
        if config.DEBUG_MODE:
            LEDs.elevator.cycle()
        else:
            try:
                LEDs.elevator.cycle()
            except Exception as e:
                print('LED Error:', str(e))
                self.nt.getTable("Errors").putString("LEDS", str(e))
        
        
        if config.DEBUG_MODE: 
            commands2.CommandScheduler.getInstance().run()
        else:
            try:
                commands2.CommandScheduler.getInstance().run()
            except Exception as e:
                print('Command Scheduler Error:', str(e))
                self.nt.getTable("Errors").putString("Command Scheduler", str(e))
        

        Sensors.limeLight_F.update()
        Sensors.limeLight_B.update()
        
        Sensors.odometry.update()
        
        # pose = Sensors.odometry.getPose()
        pose = Robot.drivetrain.odometry.getPose()
        
        self.nt.getTable("Odometry").putNumberArray("pose", [
            pose.X(),
            pose.Y(),
            pose.rotation().radians()
        ])
        
        try:
            if Sensors.limeLight_F.get_bot_pose() is not None:
                self.nt.getTable('Odometry').putNumberArray('Limelight-F', [
                    Sensors.limeLight_F.get_bot_pose().X(),
                    Sensors.limeLight_F.get_bot_pose().Y(),
                    Sensors.limeLight_F.get_bot_pose().toPose2d().rotation().radians()
                ])
        except:
            pass
        
        try:
            if Sensors.limeLight_B.get_bot_pose() is not None:
                self.nt.getTable('Odometry').putNumberArray('Limelight-B', [
                    Sensors.limeLight_B.get_bot_pose().X(),
                    Sensors.limeLight_B.get_bot_pose().Y(),
                    Sensors.limeLight_B.get_bot_pose().toPose2d().rotation().radians()
                ])
        except:
            pass
        
        n_1, n_2, n_3, n_4 = Robot.drivetrain.node_states
        
        # fl, fr, bl, br
        self.nt.getTable('Odometry').putNumberArray("NODE_STATES_EXP", [
            n_1.angle.radians(), n_1.speed,
            n_2.angle.radians(), n_2.speed,
            n_3.angle.radians(), n_3.speed,
            n_4.angle.radians(), n_4.speed
        ])   
        
        # n_fl = Robot.drivetrain.n_front_left
        # n_fr = Robot.drivetrain.n_front_right
        # n_bl = Robot.drivetrain.n_back_left
        # n_br = Robot.drivetrain.n_back_right
        
        # # fl, fr, bl, br
        # self.nt.getTable('Odometry').putNumberArray("NODE_STATES_TRUE", [
        #     n_fl.get_turn_motor_angle(), n_fl.get_motor_velocity(),
        #     n_fr.get_turn_motor_angle(), n_fr.get_motor_velocity(),
        #     n_bl.get_turn_motor_angle(), n_bl.get_motor_velocity(),
        #     n_br.get_turn_motor_angle(), n_br.get_motor_velocity()
        #     ])
    
        rotation = Robot.drivetrain.get_heading()
        
        self.nt.getTable("Odometry").putNumber("Heading", rotation.radians())
        
        
        wpilib.SmartDashboard.putNumber("Active Grid", config.active_grid)
        wpilib.SmartDashboard.putNumber("Active Station", config.active_station)
        route_name = ('grid' if config.active_route == config.Route.grid else 'station' if config.active_route == config.Route.station else 'auto')
        wpilib.SmartDashboard.putString("Active Route", route_name)
        target_name = (
            'low' if config.active_target == config.Target.low 
            else 'mid' if config.active_target == config.Target.mid 
            else 'high' if config.active_target == config.Target.high 
            else 'single' if config.active_target == config.Target.single 
            else 'double' if config.active_target == config.Target.double 
            else 'floor' if config.active_target == config.Target.floor_down 
            else 'floor up' if config.active_target == config.Target.floor_up
            else 'idle')
        wpilib.SmartDashboard.putString("Active Target", target_name)
        game_piece = ('cone' if config.active_piece == config.GamePiece.cone else 'cube')
        wpilib.SmartDashboard.putString("Active Game Piece", game_piece)

        # print('pipeline:', Sensors.limeLight_B.pipeline)
    # Initialize subsystems
    


    # Pneumatics

    def teleopInit(self):
        
        
        Robot.intake.zero_wrist()
        Robot.intake.wrist_zeroed = True
        # Robot.intake.set_lower_output(-1)
        # Robot.intake.set_upper_output(-1)
        commands2.CommandScheduler.getInstance().schedule(
            command.DrivetrainZero(Robot.drivetrain).andThen(
                command.DriveSwerveCustom(Robot.drivetrain)
            )
            )
        # commands2.CommandScheduler.getInstance().schedule(command.ZeroElevator(Robot.elevator))

    def teleopPeriodic(self):
        
        if self.team.getSelected() == config.Team.blue:
            config.active_team = config.Team.blue
        else:
            config.active_team = config.Team.red
        
        if Robot.intake.rumble_if_detected():
            self.detected_c = True
            config.auto_led_elevator = False
            config.active_leds = (config.LedType.KBlink(255, 255, 255), 1, 30)
        else:
            if self.detected_c:
                config.active_leds = (config.led_piece, 1, 5)
                self.detected_c = False
                config.auto_led_elevator = True
        
        
        
    def autonomousInit(self):
        
        
        
        if self.team.getSelected() == config.Team.blue:
            config.active_team = config.Team.blue
        else:
            config.active_team = config.Team.red
        
        if self.sensor_odometry.getSelected():
            Sensors.odometry.enable()
        else:
            Sensors.odometry.disable()
            
        
            
        self.auto.getSelected().run()
        
        config.active_leds = (config.LedType.KStatic(0, 255, 0), 1, 5)

    def autonomousPeriodic(self):
        pass

    def disabledInit(self) -> None:
        print(config.active_team)
        if config.active_team == config.Team.blue:
            config.active_leds = (config.LedType.KBlink(0, 0, 255), 1, 30)
        else:
            config.active_leds = (config.LedType.KBlink(255, 0, 0), 1, 30)

    def disabledPeriodic(self) -> None:
        pass

    

if __name__ == "__main__":
    wpilib.run(_Robot)
