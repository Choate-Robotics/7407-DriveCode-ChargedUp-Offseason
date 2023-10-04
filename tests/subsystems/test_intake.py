import pytest
from robot_systems import Robot
from command import SetIntake
import config
from math import pi
import constants
import time

Robot.intake.init()

intake = Robot.intake


def test_init():
    assert intake.upper_intake_motor is not None
    assert intake.lower_intake_motor is not None
    assert intake.intake_speed is not None


def test_set_upper_output():
    intake.set_upper_output(0.5)
    assert intake.upper_intake_motor.motor.get() == 0.5


def test_setIntake():
    command = SetIntake(intake, 0.5, config.GamePiece)
    command.initialize()


def test_grabCube():
    intake.grab_cube()
    # shouldn't this be the sensor velocity and not raw output????
    assert intake.lower_intake_motor.motor.get() == -intake.intake_speed
    assert intake.upper_intake_motor.motor.get() == -intake.intake_speed


def test_grabCone():
    intake.grab_cone()
    assert intake.lower_intake_motor.motor.get() == intake.intake_speed
    assert intake.upper_intake_motor.motor.get() == -intake.intake_speed


def test_ejectCube():
    intake.eject_cube()
    assert intake.lower_intake_motor.motor.get() == intake.intake_speed
    assert intake.upper_intake_motor.motor.get() == intake.intake_speed


def test_ejectCone():
    intake.eject_cone()
    assert intake.lower_intake_motor.motor.get() == -intake.intake_speed
    assert intake.upper_intake_motor.motor.get() == intake.intake_speed


