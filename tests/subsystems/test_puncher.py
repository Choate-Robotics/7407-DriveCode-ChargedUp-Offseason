from robot_systems import Robot
import constants
puncher = Robot.puncher

puncher.init()

def test_init_pos():
    assert puncher.getMotorPosition() == [constants.puncher_init_pos, constants.puncher_init_pos]

def test_extension():
    puncher.extend()
    assert puncher.getTargetPosition() == [constants.puncher_extend, constants.puncher_extend]

def test_retraction():
    puncher.retract()
    assert puncher.getTargetPosition() == [constants.puncher_retract, constants.puncher_retract]