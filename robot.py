import commands2
import ctre
import wpilib
import command
import config
import constants
from robot_systems import Robot, Pneumatics, Sensors
import sensors
import subsystem
import utils
from oi.OI import OI
from oi.PID_Tune import PIDConfigEditor


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Initialize Operator Interface
        OI.init()
        OI.map_controls()
        period = .03
        commands2.CommandScheduler.getInstance().setPeriod(period)
        
    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

        Sensors.limeLight_F.update()
        Sensors.limeLight_B.update()
    # Initialize subsystems

    if config.enable_pid_tuning:
        tuner = PIDConfigEditor()
        
        tuner.editSparkMaxConfig(config.WRIST_CONFIG, "wrist")
        tuner.editProfiledPIDController(config.balance_pid, 1, 0, "balance")

    # Pneumatics

    def teleopInit(self):
        a = ctre.VictorSPX(1)
        a.set(ctre.ControlMode.PercentOutput, .5)

    def teleopPeriodic(self):
        pass
    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    

if __name__ == "__main__":
    wpilib.run(_Robot)
