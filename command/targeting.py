from subsystem import Drivetrain, Intake, Elevator
from sensors import Limelight
from robotpy_toolkit_7407.command import BasicCommand, SubsystemCommand 
import command
from commands2 import SequentialCommandGroup, InstantCommand, ParallelCommandGroup, WaitCommand, RunCommand, CommandBase, WaitUntilCommand, ParallelDeadlineGroup, PrintCommand
import config, constants, math, ntcore, commands2
from wpimath.controller import PIDController

class LineupSwerve(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain, LimeLight: Limelight):
        '''
        Lines up the robot with the target
        :param drivetrain: Drivetrain subsystem
        :param LimeLight: Limelight subsystem
        :param target: (cube/cone) target to line up with'''
        super().__init__(subsystem)
        self.drivetrain = subsystem
        self.limelight = LimeLight
        self.target_exists = False
        self.target_constrained = False
        self.v_pid = PIDController(0.1, 0, 0.001)
        self.h_pid = PIDController(0.08, 0, 0.001)
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("Targetting command")
        self.target: config.GamePiece
        self.is_pipeline: bool = False
        self.tclass: str
        
        
    def initialize(self):
        
        self.tclass = 'cube'
        if config.active_piece == config.GamePiece.cone:
            self.tclass = 'cone'
        self.constraints = config.game_piece_targeting_constraints[self.tclass]
        
        self.nt.putString('tracking', self.tclass)
        self.limelight.set_pipeline_mode(config.LimelightPipeline.neural)
        self.v_pid.reset()
        self.h_pid.reset()
        self.v_pid.setTolerance(4)
        self.h_pid.setTolerance(2)
        
    def execute(self):
        if self.limelight.get_pipeline_mode() != config.LimelightPipeline.neural:
            self.limelight.update()
            print('cant see')
        elif self.limelight.get_pipeline_mode() == config.LimelightPipeline.neural and self.limelight.t_class == self.tclass:
            self.limelight.update()
            self.is_pipeline = True
            print('can see')
            if self.limelight.target_exists() == False or self.limelight.get_target() == None:
                self.target_exists = False
                print('no target')
                # print(self.limelight.table.getNumber('tv', 3))
                self.drivetrain.set_robot_centric((0,0),0)
                # commands2.CommandScheduler.getInstance().schedule(command.DriveSwerveCustom(self.drivetrain))
                # self.nt.putBoolean('see target', False)
            else:
                # self.nt.putBoolean('see target', True)
                print("target")
                tx, ty, ta = self.limelight.get_target(True)
                
                self.nt.putNumber("tx", tx)
                self.nt.putNumber("ty", ty)
                self.nt.putNumber('ta', ta)
                
                
                # to line up the game piece properly, we need to constrain the tx, ty, and ta values to certain ranges
                # tx needs to be between -3.5 and 3.5
                # ty needs to be between -9 and -11
                # ta needs to be between 9 and 13
                # these values are relative to the limelight, so we need to convert them to the robot's perspective
                # the ta, or area, increases as the target gets closer to the robot
                # the tx value is the left and right motion of the robot
                
                def avg(a):
                    return (a[0] + a[1]) / 2
                
                def constrained(a, b) -> bool:
                    return a >= b[0] and a <= b[1]
                
                if self.target_exists == False and self.target_exists:
                    self.target_exists = True
                    
                print("Tracking...")
                    
                dy = self.v_pid.calculate(ty, -9)
                dx = self.h_pid.calculate(tx, avg(self.constraints['tx']))
                
                self.nt.putNumber('PID dx', dx)
                self.nt.putNumber('PID dy', dy)
                
                # if config.drivetrain_reversed:
                #     dx *= -1
                # else:
                #     dy *= -1
                    
                dx *= config.calculated_max_vel * .25
                dy *= config.calculated_max_vel * .25
                    
                self.drivetrain.set_robot_centric((dy, -dx), 0)
                self.nt.putBoolean('tx constrained', constrained(tx, self.constraints['tx']))
                self.nt.putBoolean('ty constrained', constrained(ty, self.constraints['ty']))
                
                    # self.drivetrain.set_driver_centric((0,0), 0)
                
    def isFinished(self):
        return self.h_pid.atSetpoint() and self.v_pid.atSetpoint() and self.is_pipeline
        return False
    
    def end(self, interrupted: bool = False):
        self.limelight.set_pipeline_mode(config.LimelightPipeline.feducial)
        self.drivetrain.set_robot_centric((0, 0), 0)

        
class ZeroTarget(SequentialCommandGroup):
    
    def __init__(self, intake: Intake, elevator: Elevator):
        super().__init__(
            command.ZeroWrist(intake),
            WaitCommand(1),
            command.ZeroElevator(elevator)
        )
        
class Target(commands2.CommandBase):
    '''
    Command for controlling the elevator, wrist, and intake
    Includes logic for moving the intake out of the way of the elevator
    
    :param intake: Intake subsystem
    
    :param elevator: Elevator subsystem
    
    
    :param force: (OPTIONAL) Force the elevator to move even if it is not zeroed (bool)
    '''
    
    def __init__(self, intake: Intake, elevator: Elevator, force: bool = False, auto: bool = False):
        super().__init__()
        self.intake = intake
        self.elevator = elevator
        self.target: config.active_target
        self.piece: config.active_piece
        self.force = force
        self.auto = auto
        
    def initialize(self) -> None:
        
        if self.elevator.zeroed == False or self.intake.wrist_zeroed == False:
            commands2.CommandScheduler.getInstance().schedule(ZeroTarget(self.intake, self.elevator))
            return
        
        self.piece = config.active_piece
        
        self.target = config.active_target
    
    
        command_list = []
        command_goal = []
        save = self.target['angle']
        save_l = self.target['length']
        # wall = False
        # if self.target['wall']:
        #     wall = True
        
        
        if self.piece == config.GamePiece.cube and (self.target['goal'] == 'pickup' or self.target['goal'] == 'shoot'):
            self.target['angle'] = self.target['angle-cube']
            self.target['length'] = self.target['length-cube']
            # if self.target['wall-cube']:
            #     wall = True
            command_goal.append('cube angle/length')
        else:
            self.target['angle'] = save
            command_goal.append('cone angle/length')
        
        if self.piece == config.GamePiece.cone:
            command_goal.append('cone')
        elif self.piece == config.GamePiece.cube:
            command_goal.append('cube')
        command_goal.append( 'Obj: ' + self.target['goal'])
        # if the intake is in the way of the elevator if its moving down, move it out of the way
        if self.intake.get_wrist_angle() < math.radians(-.2) \
            and self.elevator.get_length() <= config.elevator_intake_threshold * constants.elevator_max_rotation + 5 \
            and self.target['length'] < config.elevator_intake_threshold:
            
            command_list.append(command.SetCarriage(self.intake, math.radians(0), config.IntakeActive.kIdle, self.piece))
            command_goal.append(f'move wrist to zero (BLOCKING) {self.intake.get_wrist_angle()} ')
            
        # if the intake is in the way of the elevator if its moving up, move it out of the way
        if self.intake.get_wrist_angle() > math.radians(110) \
            and self.target['length'] * constants.elevator_max_rotation > self.elevator.get_length():
            # command_list.append(command.SetCarriage(self.intake, math.radians(100), config.IntakeActive.kIdle, self.piece))
            
            command_list.append(
                ParallelDeadlineGroup(
                    WaitUntilCommand(lambda: self.intake.get_wrist_angle() < math.radians(145)),
                    command.SetCarriage(self.intake, math.radians(0), config.IntakeActive.kIdle, self.piece)
                )
            )
            command_list.append(
                ParallelCommandGroup(
                    command.SetElevator(self.elevator, self.target['length'], self.force),
                    command.SetCarriage(self.intake, self.target['angle'], config.IntakeActive.kIdle, self.piece)
                )
            )
            print(self.intake.get_wrist_angle())
            command_goal.append(f'move wrist to 100 (BLOCKING)')
        # # if the wrist rotation is past 120, config.wall is true, and elevator is under the threshold, move the elevator up and then move the wrist
        # elif self.intake.get_wrist_angle() > math.radians(120) \
        #     and config.pickup_wall \
        #     and self.elevator.get_length() < config.elevator_wall_threshold * constants.elevator_max_rotation:
            
        #     command_list.append(
        #         InstantCommand(command.SetElevator(self.elevator, self.target['length'], self.force)),
        #         WaitUntilCommand(lambda: self.elevator.get_length() > config.elevator_wall_threshold * constants.elevator_max_rotation),
        #         command.SetCarriage(self.intake, self.target['angle'], config.IntakeActive.kIdle, self.piece)
        #     )
        
        # if the wrist target rotation is past 0 degrees and the elevator is below the threshold, move the elevator up and then move the wrist
        elif self.target['angle'] < math.radians(0) \
            and self.intake.get_wrist_angle() > math.radians(-10) \
            and self.elevator.get_length() < config.elevator_intake_threshold * constants.elevator_max_rotation:

            command_list.append(
                ParallelDeadlineGroup(
                    WaitUntilCommand(lambda: self.elevator.get_length() > config.elevator_intake_threshold * constants.elevator_max_rotation),
                    command.SetElevator(self.elevator, self.target['length'], self.force),
                    command.SetCarriage(self.intake, math.radians(0), config.IntakeActive.kIdle, self.piece)
                    )
            )
            command_list.append(
                ParallelCommandGroup(
                    command.SetElevator(self.elevator, self.target['length'], self.force),
                    command.SetCarriage(self.intake, self.target['angle'], config.IntakeActive.kIdle, self.piece)
                )
            )
            command_goal.append(str(self.intake.get_wrist_angle()))
            command_goal.append('intake moving past 0')
            command_goal.append(f'run elevator up to threshold (BLOCKING), wrist goes to zero')
            command_goal.append('move wrist to desired rotation continue elevator to desired position')
        else:
            # Normal Operation
            command_list.append(
                ParallelCommandGroup(
                        command.SetElevator(self.elevator, self.target['length'], self.force),
                        command.SetCarriage(self.intake, self.target['angle'], config.IntakeActive.kIdle, self.piece)
                    )
            )
            command_goal.append('run elevator and wrist to desired setpoint')
        
        # if the target goal is to pickup game pieces, set intake to automatically run in
        if self.target['goal'] == 'pickup':
            command_list.append(
                command.SetIntake(self.intake, config.IntakeActive.kIn, self.piece),
            )
            command_goal.append('run intake in')
            
        if self.target['goal'] == 'shoot':
            command_list.append(
                command.SetIntake(self.intake, config.IntakeActive.kShoot, self.piece),
            )
            command_goal.append('shoot intake out')
        
        action = ''
        for i in command_goal:
            action += i + ', '    
        
        self.target['angle'] = save
        self.target['length'] = save_l
        
        print(action)
        
        command_final = ParallelCommandGroup(
                PrintCommand(action),
                SequentialCommandGroup(
                    *command_list
                )
            )
        
        if self.auto:
            return command_final
        
        # config.pickup_wall = wall
        else:
            commands2.CommandScheduler.getInstance().schedule(
                command_final
            )
        
    def isFinished(self) -> bool:
        return True
    
    def end(self, interrupted):
        pass
        

class Idle(commands2.CommandBase):
    
    def __init__(self, intake: Intake, elevator: Elevator) -> None:
        super().__init__()
        self.intake = intake
        self.elevator = elevator
        
    def initialize(self) -> None:
        target = config.active_target
        config.active_target = config.Target.idle
        commands2.CommandScheduler.getInstance().schedule(Target(self.intake, self.elevator))
        config.active_target = target
        
    def isFinished(self) -> bool:
        return True
    
    def end(self, interrupted):
        pass
    

class Shoot(commands2.CommandBase):
    
    def __init__(self, intake: Intake, elevator: Elevator) -> None:
        super().__init__()
        self.intake = intake
        self.elevator = elevator
        
    def initialize(self) -> None:
        target = config.active_target
        config.active_target = config.Target.shoot
        commands2.CommandScheduler.getInstance().schedule(Target(self.intake, self.elevator))
        config.active_target = target
        
    def isFinished(self) -> bool:
        return True

    def end(self, interrupted):
        pass
    
    
def set_target(target: config.Target):
    config.active_target = target
    
def set_piece(piece: config.GamePiece):
    config.active_piece = piece

def set_auto():
    config.active_leds = (config.LedType.KBlink(0, 255, 0), 1, 5)

class TargetAuto(commands2.CommandBase):
    
    def __init__(self, intake: Intake, elevator: Elevator, target: config.Target = config.Target.idle):
        super().__init__()
        self.target = target
        self.intake = intake
        self.elevator = elevator
    
    def initialize(self):
        target = config.active_target
        config.active_target = self.target
        commands2.CommandScheduler.getInstance().schedule(Target(self.intake, self.elevator))
        config.active_target = target
        
    def isFinished(self) -> bool:
        return True

class AutoPickup(SequentialCommandGroup):
    def __init__(self, drivetrain: Drivetrain, intake: Intake, elevator: Elevator, limelight: Limelight, target: config.GamePiece):
        super().__init__(
            ParallelCommandGroup(
                InstantCommand(set_auto),
                LineupSwerve(drivetrain, limelight),
                InstantCommand(lambda: set_target(config.Target.floor_up)),
                Target(intake, elevator)
                # TargetAuto(intake, elevator, config.Target.floor_up)
                ),
            InstantCommand(lambda: set_target(config.Target.floor_down)),
            Target(intake, elevator),
            # TargetAuto(intake, elevator, config.Target.floor_down),
            WaitUntilCommand(lambda: intake.get_wrist_angle() > math.radians(155)),
            InstantCommand(lambda: drivetrain.set_robot_centric((-.5 * constants.drivetrain_max_vel, 0), 0)),
            WaitUntilCommand(lambda: intake.get_avg_current() > 30), # wait until intake has game piece,
            InstantCommand(lambda: drivetrain.set_robot_centric((0,0),0)),
            InstantCommand(lambda: set_target(config.Target.idle)),
            Target(intake, elevator),
            # TargetAuto(intake, elevator),
            )
