from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, Trajectory, TrajectoryUtil 
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from subsystem import Drivetrain
from robotpy_toolkit_7407.command import subsystemCommand

class Path():
    def __init__(self, start, end, waypoints, max_velocity, max_acceleration, reversed=False):
        self.start = start
        self.end = end
        self.waypoints = waypoints
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.reversed: bool = reversed

    def generate(self):
        config = TrajectoryConfig(self.max_velocity, self.max_acceleration)
        config.setReversed(self.reversed)
        return TrajectoryGenerator.generateTrajectory(self.start, self.waypoints, self.end, config)
    
