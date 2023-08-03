from robotpy_toolkit_7407.motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407 import Subsystem
import config
import rev
import constants


PID = SparkMaxConfig(1,0, .5, 0, (-1,1), rev.CANSparkMax.IdleMode.kBrake)

class Puncher(Subsystem):

    left_motor: SparkMax = SparkMax(config.punch_left, config=PID)
    right_motor: SparkMax = SparkMax(config.punch_right, config=PID)
    extended: bool
    left_target: float
    right_target: float

    def __init__(self):
        super().__init__()
        self.left_target, self.right_target = constants.puncher_init_pos, constants.puncher_init_pos
        self.extended = False

    def setMotorTarget(self, pos):
        self.left_motor.set_target_position(pos)
        self.right_motor.set_target_position(pos)
        self.left_target, self.right_target = pos, pos

    def setMotorPosition(self, pos):
        self.left_motor.set_sensor_position(pos)
        self.right_motor.set_sensor_position(pos)

    def init(self):
        self.left_motor.init()
        self.right_motor.init()
        self.setMotorPosition(constants.puncher_init_pos)

    def extend(self):
        self.setMotorTarget(constants.puncher_extend)
        self.extended = True

    def retract(self):
        self.setMotorTarget(constants.puncher_retract)
        self.extended = False

    def getMotorPosition(self):
        l = self.left_motor.get_sensor_position()
        r = self.right_motor.get_sensor_position()
        return [l, r]
    
    def getTargetPosition(self):
        return [self.left_target, self.right_target]