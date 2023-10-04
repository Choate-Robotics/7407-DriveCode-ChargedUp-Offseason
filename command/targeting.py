from subsystem import Drivetrain, Intake, Elevator
from sensors import Limelight
from robotpy_toolkit_7407.command import BasicCommand, SubsystemCommand 
import command
from commands2 import SequentialCommandGroup, InstantCommand, ParallelCommandGroup, WaitCommand, RunCommand, CommandBase, WaitUntilCommand
import config, constants, math, ntcore
from wpimath.controller import PIDController

class LineupSwerve(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain, LimeLight: Limelight, target: config.GamePiece):
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
        self.v_pid = PIDController(0.1, 0, 0)
        self.h_pid = PIDController(0.1, 0, 0.01)
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("Targetting command")
        tn = 'cube'
        if target == config.GamePiece.cone:
            tn = 'cone'
        self.constraints = config.game_piece_targeting_constraints[tn]
        
    def initialize(self):
        
        
        
        self.nt.putNumber('tracking', 2)
        self.limelight.set_pipeline_mode(0)
        # self.v_pid.setTolerance(4)
        # self.h_pid.setTolerance(2)
        
    def execute(self):
        
        
        if self.limelight.target_exists() == False or self.limelight.get_target() == None:
            self.target_exists = False
            print('no target')
            print(self.limelight.table.getNumber('tv', 3))
            self.drivetrain.set_robot_centric((0,0),0)
            # self.nt.putBoolean('see target', False)
        else:
            # self.nt.putBoolean('see target', True)
            print("target")
            tx, ty, ta = self.limelight.get_target()
            
            self.nt.putNumber("tx", tx)
            self.nt.putNumber("ty", ty)
            self.nt.putNumber('ta', ta)
            
                # command.DriveSwerveCustom.execute(self)
            
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
                self.target_constrained = False
                self.v_pid.setSetpoint(-9)
                self.h_pid.setSetpoint(avg(self.constraints['tx']))
                
            print("Tracking...")
                
            dy = self.v_pid.calculate(ty)
            dx = self.h_pid.calculate(tx)
            
            self.nt.putNumber('PID dx', dx)
            self.nt.putNumber('PID dy', dy)
            
            # if config.drivetrain_reversed:
            #     dx *= -1
            # else:
            #     dy *= -1
                
            dx *= self.drivetrain.max_vel * .25
            dy *= self.drivetrain.max_vel * .25
                
            self.drivetrain.set_robot_centric((-dy, -dx), 0)
            self.nt.putBoolean('tx constrained', constrained(tx, self.constraints['tx']))
            self.nt.putBoolean('ty constrained', constrained(ty, self.constraints['ty']))
            
            if constrained(tx, self.constraints['tx']) and constrained(ty, self.constraints['ty']):
                self.target_constrained = True
                # self.drivetrain.set_driver_centric((0,0), 0)
            
    def isFinished(self):
        return self.target_constrained and self.target_exists
        return False
    
    def end(self, interrupted: bool = False):
        self.drivetrain.set_robot_centric((0, 0), 0)

           
            
class Target(SequentialCommandGroup):
    def __init__(self, intake: Intake, elevator: Elevator, target: config.Target, piece: config.GamePiece = None, force: bool = False):
        if intake.get_wrist_angle() < math.radians(-3) \
            and elevator.get_length() < config.elevator_intake_threshold * constants.elevator_max_rotation \
            and target['length'] < config.elevator_intake_threshold * constants.elevator_max_rotation:
            super().__init__(
                command.SetCarriage(intake, 0, False, piece),
            )
        elif intake.get_wrist_angle() > math.radians(160) \
            and target['length'] > elevator.get_length():
            super().__init__(
                command.SetCarriage(intake, math.radians(120), False, piece),
            )
        super().__init__(
            ParallelCommandGroup(
                    command.SetElevator(elevator, target['length'], force),
                    command.SetWrist(intake, target['angle'])
                )
        )
        if target['goal'] == 'pickup':
            super().__init__(
                command.SetCarriage(intake, target['angle'], config.IntakeActive.kIn, piece),
            )
        
class AutoPickup(SequentialCommandGroup):
    def __init__(self, drivetrain: Drivetrain, intake: Intake, elevator: Elevator, limelight: Limelight, target: config.GamePiece):
        super().__init__(
            ParallelCommandGroup(
                LineupSwerve(drivetrain, limelight, target),
                Target(intake, elevator, config.Target.floor_up)
                ),
            Target(intake, elevator, config.Target.floor_down),
            InstantCommand(lambda: drivetrain.set_driver_centric(-.4 * constants.drivetrain_move_gear_ratio, 0), 0),
            WaitUntilCommand(lambda: Intake.get_detected(intake, target)), # wait until intake has game piece,
            Target(intake, elevator, config.Target.floor_up),
            )