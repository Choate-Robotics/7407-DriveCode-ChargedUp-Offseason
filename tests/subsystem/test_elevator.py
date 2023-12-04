import pytest
from unittest.mock import MagicMock
from robot_systems import Robot
import constants
from subsystem import Elevator
from wpilib import DataLogManager


@pytest.fixture
def subsystem():
    subsystem = Robot()
    


@pytest.fixture
def elevator() -> Elevator:
    elevator = Elevator()
    elevator.motor_extend = MagicMock()
    return elevator


def test_init(elevator):
    init = elevator.motor_extend.init
    elevator.init()
    init.assert_called_once()


def test_stop(elevator):
    elevator.stop()

    elevator.motor_extend.set_raw_output.assert_called_once_with(0)


def test_set_length(elevator: Elevator):
    elevator.set_length(1)
    elevator.motor_extend.set_target_position.assert_called_once_with(1 / constants.elevator_length_per_rotation)


def test_get_length(elevator: Elevator):
    elevator.motor_extend.get_sensor_position.return_value = 1
    assert elevator.get_length() == constants.elevator_length_per_rotation
    elevator.motor_extend.get_sensor_position.assert_called_once()
