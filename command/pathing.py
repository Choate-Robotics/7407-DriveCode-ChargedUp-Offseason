from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, Trajectory
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Pose3d
from wpimath.trajectory.constraint import RectangularRegionConstraint, TrajectoryConstraint
from subsystem import Drivetrain
from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveDrivetrain
from wpilib import Timer, SmartDashboard, Field2d
import config, commands2, constants
from sensors import FieldOdometry
import ntcore, math
from robot_systems import Sensors
from utils import Poses

class SetPosition(SubsystemCommand[SwerveDrivetrain]):
    
    def __init__(self, subsystem: Drivetrain, position: Pose2d | tuple[Poses.Type, int]):
        super().__init__(subsystem)    
        self.subsystem = subsystem
        self.new_pose: position = position
    
    def initialize(self) -> None:
        
        if not isinstance(self.new_pose, Pose2d):
            self.new_pose = Sensors.poses.get_selected_POI(self.new_pose)
        Sensors.poses.init()
        self.subsystem.reset_odometry(self.new_pose)
    
    def isFinished(self) -> bool:
        return True
    
    def end(self, interrupted: bool) -> None:
        pass

class Path():
    def __init__(self, start: Pose2d | tuple[Poses.Type, int], end: Pose2d | tuple[Poses.Type, int], waypoints: list[Translation2d], max_velocity:float, max_acceleration:float, reversed=False, grid_speed: tuple = (0, 0)):
        self.start:Pose2d | tuple[Poses.Type, int] = start
        self.end:Pose2d | tuple[Poses.Type, int] = end
        self.waypoints:list[Translation2d] = waypoints
        self.max_velocity:float = max_velocity
        self.max_acceleration:float = max_acceleration
        self.reversed: bool = reversed
        self.grid_speed: tuple = grid_speed
        self.poses = []
        self.trajectory: Trajectory = None
        # if self.grid_speed[0] == 0:
        #     self.grid_speed[0] = self.max_velocity
        # if self.grid_speed[1] == 0:
        #     self.grid_speed[1] = self.max_velocity

    def generate(self):
        
        
        if not isinstance(self.start, Pose2d):
            self.start = Sensors.poses.get_selected_POI(self.start)
        if not isinstance(self.end, Pose2d):
            self.end = Sensors.poses.get_selected_POI(self.end)
        
        # for waypoint in self.waypoints:
        #     if not isinstance(waypoint, Translation2d):
            
        config = TrajectoryConfig(constants.drivetrain_max_vel, constants.drivetrain_max_target_accel * 2)
        # config.addConstraint(community)
        config.setReversed(self.reversed)
        self.trajectory = TrajectoryGenerator.generateTrajectory(self.start, self.waypoints, self.end, config)
        return self.trajectory
    
    def getPoses(self):
        if self.trajectory == None:
            self.generate()
        traj = self.trajectory
        self.poses = []
        for state in traj.states():
            self.poses.append(state.pose.X())
            self.poses.append(state.pose.Y())
            self.poses.append(state.pose.rotation().radians())
        return self.poses
        
    def log(self, name):
        if len(self.poses) == 0:
            self.getPoses()
        ntcore.NetworkTableInstance.getDefault().getTable('auto').putNumberArray(
            name, self.poses
        )
        
    def logActive(self, name = 'none'):
        if name == 'none':
            name = 'active_trajectory'
        self.log(name)
        
    
    
    
class FollowPathCustom(SubsystemCommand[SwerveDrivetrain]):
    
    def __init__(self, subsystem: Drivetrain, path: Path):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.path: Path = path
        self.y_pid: PIDController = PIDController(1, 0, 0)
        self.x_pid: PIDController = PIDController(1, 0, 0)
        self.w_pid: ProfiledPIDControllerRadians = PIDController(0.2, 0, 0.0008)
        self.y_pid.setTolerance(.2)
        self.x_pid.setTolerance(.2)
        self.w_pid.setTolerance(.2)
        self.trajectory: Trajectory = None
        self.t_total: float = 0
        self.t_delta: Timer = Timer()
        self.finished = False
        
    def initialize(self):
        self.finished = False
        self.y_pid.reset()
        self.x_pid.reset()
        self.w_pid.reset()
        self.trajectory = self.path.generate()
        self.path.logActive()
        self.t_total = self.trajectory.totalTime()
        self.t_delta.reset()
        self.t_delta.start()
        
    def execute(self):
        odometry = self.subsystem.odometry.getPose()
        t = self.t_delta.get()
        if t > self.t_total:
            t = self.t_total
            
        pose_fin = self.trajectory.sample(self.t_total).pose
        
        if(
                abs(pose_fin.X() - odometry.X()) < .1
                and abs(pose_fin.Y() - odometry.Y()) < .1
                and abs(pose_fin.rotation().radians() - odometry.rotation().radians()) < .1
        ):
            t = self.t_total
            self.finished = True

        state = self.trajectory.sample(t)
        
        dy = (
            self.y_pid.calculate(odometry.Y(), state.pose.Y())
            * self.subsystem.max_vel
        )
        
        dx = (
            self.x_pid.calculate(odometry.X(), state.pose.X())
            * self.subsystem.max_vel
        )
        
        d_theta = (
            self.w_pid.calculate(odometry.rotation().radians(), state.pose.rotation().radians())
            * self.subsystem.max_angular_vel
        )
        
        self.subsystem.set_driver_centric((-dx, -dy), d_theta)
        
    def isFinished(self):
        return self.finished
        
        
    def end(self, interrupted: bool):
        self.subsystem.set_driver_centric((0, 0), 0)
        
class RouteTarget(SubsystemCommand[SwerveDrivetrain]):
    
    def __init__(self, subsystem: SwerveDrivetrain, target: Pose2d, threshold: tuple = (.2, .2, .1), grid: bool = False):
        
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.target: Pose2d = target
        self.y_pid: PIDController = PIDController(0.4, 0, 0)
        self.x_pid: PIDController = PIDController(0.4, 0, 0)
        self.w_pid: PIDController = PIDController(0.02, 0, 0.008)
        self.threshold: tuple = threshold
        self.grid: bool = grid
        self.pose: Pose2d = None
        self.y_done, self.x_done, self.w_done = False, False, False
        
    def initialize(self):
        
        self.y_pid.reset()
        self.x_pid.reset()
        self.w_pid.reset()
        
        self.y_pid.setTolerance(self.threshold[0])
        self.x_pid.setTolerance(self.threshold[1])
        self.w_pid.setTolerance(self.threshold[2])
    
        config.active_leds = (config.LedType.KStatic(5, 0, 134), 1, 5)
        
        ntcore.NetworkTableInstance.getDefault().getTable('auto').putNumberArray('active_pose',[
            self.target.X(),
            self.target.Y(),
            self.target.rotation().radians()
            ]
            )

    def execute(self):
        
        self.pose = self.subsystem.odometry.getPose()
        
        dy = (
            self.y_pid.calculate(self.pose.Y(), self.target.Y())
            * config.calculated_max_vel
        )
        
        dx = (
            self.x_pid.calculate(self.pose.X(), self.target.X())
            * config.calculated_max_vel
        )
        
        d_theta = (
            self.w_pid.calculate(self.pose.rotation().radians(), self.target.rotation().radians())
            * config.calculated_max_angular_vel
        )
        
        if self.grid:
            if not self.x_done:
                dy = 0
            
            if self.x_done:
                dx = 0
                
        # DOES NOT TAKE VELOCITY INTO CONSIDERATION
                
        if self.y_pid.atSetpoint():
            self.y_done = True
            
        if self.x_pid.atSetpoint():
            self.x_done = True
            
        if self.w_pid.atSetpoint():
            self.w_done = True
        
        self.subsystem.set_driver_centric((-dx, -dy), d_theta)
        
        
        
    def IsFinished(self):
        
        # DOES NOT TAKE FINAL VELOCITY INTO CONSIDERATION
        return self.y_done and self.x_done and self.w_done
    
    def end(self, interrupted: bool):
        
        self.subsystem.set_driver_centric((0, 0), 0)
        config.active_leds = (config.LedType.KBlink(5, 0, 134), 1, 5)
        
class RunRoute(commands2.CommandBase):
    
    def __init__(self, drivetrain: SwerveDrivetrain, odometry: FieldOdometry):
        super().__init__()
        self.drivetrain = drivetrain
        self.odometry = odometry
        self.target: Pose2d = None
        self.april_tags: dict[int, Pose3d] = None
        self.grid_tags: list[Pose3d] = None
        self.station_tag: Pose3d = None
        self.team_station: Pose2d = None
        self.grid: bool = False
        
    def initialize(self):
        
        self.grid = False
        
        grid_pos, station_pos, _ = Sensors.poses.return_poses()
        
        # if the active route type is grid, select grid target
        if config.active_route == config.Route.grid:
            self.grid = True
            # if the active grid is auto, select the closest grid target
            if config.active_grid == 0:
                self.target = min(
                    grid_pos,
                    key=lambda target: target.translation().distance(self.odometry.getPose().translation())
                )
            else:
                # select grid target
                self.target = grid_pos[config.active_grid]
        # if the active route type is station, select station target
        elif config.active_route == config.Route.station:
            # if the active station is auto, select the closest station target
            if config.active_station == config.Station.auto:
                self.target = min(
                    station_pos,
                    key=lambda target: target.translation().distance(self.odometry.getPose().translation())
                )
            else:
                # select station target
                self.target = station_pos[config.active_station]
        # if the active route type is auto, select the closest target
        else:
            targets_total:list[Pose2d] = grid_pos + station_pos
            gs = min(
                grid_pos,
                key=lambda target: target.translation().distance(self.odometry.getPose().translation())
            )
            
            ss = min(
                station_pos,
                key=lambda target: target.translation().distance(self.odometry.getPose().translation())
            )
            
            self.target = min(
                [ss, gs],
                key=lambda target: target.translation().distance(self.odometry.getPose().translation())
            )
            
            if self.target == gs:
                self.grid = True
            
        # run the route
        commands2.CommandScheduler.getInstance().schedule(
            RouteTarget(self.drivetrain, self.target, grid=self.grid)
        )
        
    def isFinished(self):
        return True
    
    def end(self, interrupted: bool) -> None:
        pass