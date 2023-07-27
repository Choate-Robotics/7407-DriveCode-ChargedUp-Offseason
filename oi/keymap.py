import wpilib
from robotpy_toolkit_7407.oi import (
    XBoxController,
    LogitechController,
    JoystickAxis,
    DefaultButton,
)
from robotpy_toolkit_7407.oi.joysticks import Joysticks

controllerDRIVER = XBoxController
controllerOPERATOR = XBoxController
controllerNUMPAD = XBoxController


class Controllers:
    DRIVER = 0
    OPERATOR = 1
    NUMPAD = 2

    DRIVER_CONTROLLER = wpilib.Joystick(0)
    OPERATOR_CONTROLLER = wpilib.Joystick(1)
    NUMPAD_CONTROLLER = wpilib.Joystick(2)


class Keymap:
    pass
