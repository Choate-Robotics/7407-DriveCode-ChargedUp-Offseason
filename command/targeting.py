from subsystem import Drivetrain, Intake
from sensors import Limelight
from robotpy_toolkit_7407.command import BasicCommand, SubsystemCommand 
import command
from commands2 import SequentialCommandGroup, InstantCommand, ParallelCommandGroup, WaitCommand, RunCommand, CommandBase, WaitUntilCommand
import config, constants, math
from wpimath.controller import PIDController

class LineupSwerve(SubsystemCommand[Drivetrain]):
    def __init__(self, drivetrain: Drivetrain, LimeLight: Limelight, target: config.game_piece):
        '''
        Lines up the robot with the target
        :param drivetrain: Drivetrain subsystem
        :param LimeLight: Limelight subsystem
        :param target: (cube/cone) target to line up with'''
        super().__init__()
        self.drivetrain = drivetrain
        self.limelight = LimeLight
        self.target_exists = False
        self.target_constrained = False
        self.v_pid = PIDController(0.1, 0, 0)
        self.h_pid = PIDController(0.1, 0, 0)
        tn = 'cube'
        if target == 0:
            tn = 'cone'
        self.constraints = config.game_piece_targeting_constraints[tn]
        
    def initialize(self):
        self.limelight.set_pipeline_mode(config.limelight_pipeline['ML'])
        self.v_pid.setTolerance(abs(self.constraints['ty'][1] - self.constraints['ty'][0]))
        self.h_pid.setTolerance(abs(self.constraints['tx'][1] - self.constraints['tx'][0]))
        
    def execute(self):
        tx, ty, ta = self.limelight.get_target()
        
        if self.limelight.target_exists() == False:
            self.target_exists = False
            command.DriveSwerveCustom.execute(self)
        
        # to line up the game piece properly, we need to constrain the tx, ty, and ta values to certain ranges
        # tx needs to be between -3.5 and 3.5
        # ty needs to be between -9 and -11
        # ta needs to be between 9 and 13
        # these values are relative to the limelight, so we need to convert them to the robot's perspective
        # the ta, or area, increases as the target gets closer to the robot
        # the tx value is the left and right motion of the robot
        
        def avg(a):
            return (a[0] + a[1]) / 2
        
        def constrained(a, b):
            return a >= b[0] and a <= b[1]
        
        if self.target_exists == False and self.target_exists():
            self.target_exists = True
            self.target_constrained = False
            self.v_pid.setSetpoint(avg(self.constraints['ty']))
            self.h_pid.setSetpoint(avg(self.constraints['tx']))
            
            
        dy = self.v_pid.calculate(ty)
        dx = self.h_pid.calculate(tx)
        
        if config.drivetrain_reversed:
            dx *= -1
        else:
            dy *= -1
            
        dx *= constants.drivetrain_move_gear_ratio
        dy *= constants.drivetrain_move_gear_ratio
            
        self.drivetrain.set_driver_centric((dx, dy), 0)
        
        if self.target_constrained == False and self.target_exists:
            if constrained(tx, self.constraints['tx']) and constrained(ty, self.constraints['ty']) and constrained(ta, self.constraints['ta']):
                self.target_constrained = True
        
    def isFinished(self):
        return self.target_constrained and self.target_exists
    
    def end(self, interrupted: bool = False):
        self.drivetrain.set_driver_centric((0, 0), 0)

class AutoPickup(SequentialCommandGroup):
    def __init__(self, drivetrain: Drivetrain, intake: Intake, limelight: Limelight, target: config.game_piece):
        super().__init__(
            ParallelCommandGroup(
                LineupSwerve(drivetrain, limelight, target),
                # bring elevator and intake down
                ),
            # lower intake to floor,
            InstantCommand(lambda: drivetrain.set_driver_centric(0, -.4 * constants.drivetrain_move_gear_ratio), 0),
            WaitUntilCommand() # wait until intake has game piece,
            # raise intake back up
            )
            
            