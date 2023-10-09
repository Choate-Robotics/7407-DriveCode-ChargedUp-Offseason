import wpilib

import commands2.button

from robotpy_toolkit_7407.oi import (
    XBoxController,
    JoystickAxis,
)
from robotpy_toolkit_7407.oi.joysticks import Joysticks

controllerDRIVER = XBoxController
controllerOPERATOR = XBoxController
controllerNUMPAD = XBoxController

class Controllers:

    DRIVER = 0
    OPERATOR = 1
    NUMPAD = 2
    NUMPAD_CONTROLLER = wpilib.Joystick(2)


    DRIVER_CONTROLLER = wpilib.Joystick(0)
    OPERATOR_CONTROLLER = wpilib.Joystick(1)

class Keymap:
    class Drivetrain:
        DRIVE_X_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.L_JOY[0])
        DRIVE_Y_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.L_JOY[1])
        DRIVE_ROTATION_AXIS = JoystickAxis(
            Controllers.DRIVER, controllerDRIVER.R_JOY[0]
        )
        RESET_GYRO = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.B
        )
        X_MODE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.X
        )
        
        AUTO_PICKUP = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.LB
        )
        
        ZERO_ELEVATOR = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.Y
        )
    
    class Target:
        CONE_ACTIVE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.START
        )
        
        CUBE_ACTIVE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.SELECT
        )
        
        SET_LOW = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.A
        )
        
        SET_MIDDLE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.X
        )
        
        SET_HIGH = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.Y
        )
        
        SET_SINGLE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.LB
        )
        
        SET_DOUBLE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.RB
        )
        
        SET_FLOOR = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.B
        )
        
        NO_GRID = commands2.button.Button(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 0
        )
        
        RUN_TARGET = commands2.button.Button(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.RT) > 0.5
        )
        
        
    class Grid:
        
        RAISE_GRID = commands2.button.Button(
            lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerDRIVER.RT) > 0.5
        )
        
        LOWER_GRID = commands2.button.Button(
            lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerDRIVER.LT) > 0.5
        )
        
        NO_GRID = commands2.button.Button(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 0
        )
        
    class Station:
        
        DOUBLE_STATION_LEFT = commands2.button.Button(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 270
        )
        
        DOUBLE_STATION_RIGHT = commands2.button.Button(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 90
        )
        
        SINGLE_STATION = commands2.button.Button(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 180
        )
        
    class Route:
        
        SET_STATION_ROUTE = commands2.button.Button(
            lambda: Controllers.DRIVER_CONTROLLER.getPOV() == 270
        )
        
        SET_GRID_ROUTE = commands2.button.Button(
            lambda: Controllers.DRIVER_CONTROLLER.getPOV() == 90
        )
        
        SET_AUTO_ROUTE = commands2.button.Button(
            lambda: Controllers.DRIVER_CONTROLLER.getPOV() == 0
        )
        
        RUN_ROUTE = commands2.button.Button(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.LT) > 0.5
        )
    
    class Intake:
        
        DROP_PIECE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.A
        )
        
        RUN_INTAKE = JoystickAxis(Controllers.OPERATOR, controllerOPERATOR.L_JOY[1])

    # class Puncher:
    #     PUNCH_EXTEND = commands2.button.JoystickButton(Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.SELECT)
    #     PUNCH_RETRACT = commands2.button.JoystickButton(Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.START)


