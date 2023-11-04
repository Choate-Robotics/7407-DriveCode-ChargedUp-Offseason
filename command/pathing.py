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
import ntcore

class Path():
    def __init__(self, start: Pose2d, end: Pose2d, waypoints: list[Translation2d], max_velocity:float, max_acceleration:float, reversed=False, grid_speed: tuple = (0, 0)):
        self.start:Pose2d = start
        self.end:Pose2d = end
        self.waypoints:list[Translation2d] = waypoints
        self.max_velocity:float = max_velocity
        self.max_acceleration:float = max_acceleration
        self.reversed: bool = reversed
        self.grid_speed: tuple = grid_speed
        # if self.grid_speed[0] == 0:
        #     self.grid_speed[0] = self.max_velocity
        # if self.grid_speed[1] == 0:
        #     self.grid_speed[1] = self.max_velocity

    def generate(self):
        
        # class Constraint(TrajectoryConstraint):
            
        #     def __init__(self, max_velocity, max_acceleration):
        #         super().__init__()
        #         self.max_velocity = max_velocity
        #         self.max_acceleration = max_acceleration
                
        #     def maxVelocity(self, pose: Pose2d, curvature: float, velocity: float) -> float:
        #         return self.grid_speed[0]
            
        #     def minMaxAcceleration(self, pose: Pose2d, curvature: float, speed: float) -> super().MinMax:
        #         res = super().MinMax()
        #         res.minAcceleration = 0
        #         res.maxAcceleration = self.grid_speed[1]
        #         return res
            
        # rec_points = list[Translation2d]
            
        # if config.active_team == config.Team.blue:
        #     rec_points = [
        #         Pose2d(Translation2d(-constants.border_tag_to_wall, 0),0).relativeTo(constants.ApriltagPositionDictBlue[6].translation()).translation(),
        #         Pose2d(Translation2d(constants.border_tag_to_wall, constants.path_box_height),0).relativeTo(constants.ApriltagPositionDictBlue[8].translation()).translation(),
        #     ]
        # else:
        #     rec_points = [
        #         Pose2d(Translation2d(-constants.border_tag_to_wall, 0),0).relativeTo(constants.ApriltagPositionDictRed[1].translation()).translation(),
        #         Pose2d(Translation2d(constants.border_tag_to_wall, constants.path_box_height),0).relativeTo(constants.ApriltagPositionDictRed[3].translation()).translation(),
        #     ]
            
        # community = RectangularRegionConstraint(
        #     rec_points[0],
        #     rec_points[1],
        #     Constraint(self.grid_speed[0], self.grid_speed[1])
        # )
        config = TrajectoryConfig(self.max_velocity, self.max_acceleration)
        # config.addConstraint(community)
        config.setReversed(self.reversed)
        return TrajectoryGenerator.generateTrajectory(self.start, self.waypoints, self.end, config)
    
class FollowPathCustom(SubsystemCommand[SwerveDrivetrain]):
    
    def __init__(self, subsystem: Drivetrain, path: Path):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.path: Path = path
        self.y_pid: PIDController = PIDController(0.1, 0, 0.001)
        self.x_pid: PIDController = PIDController(0.08, 0, 0.001)
        self.w_pid: ProfiledPIDControllerRadians = PIDController(0.2, 0, 0.0008)
        self.trajectory: Trajectory = None
        self.t_total: float = 0
        self.t_delta: Timer = Timer()
        
    def initialize(self):
        self.trajectory = self.path.generate()
        # traj = Field2d
        # SmartDashboard.putData(traj)
        # traj.getObject('traj').setTrajectory(self.trajectory)
        self.t_total = self.trajectory.totalTime()
        self.t_delta.reset()
        self.t_delta.start()
        self.y_pid.reset()
        
    def execute(self):
        odometry = self.subsystem.odometry.getPose()
        t = self.t_delta.get()
        if t > self.t_total:
            t = self.t_total
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
        
        self.subsystem.set_driver_centric((dy, -dx), d_theta)
        
    def isFinished(self):
        return (
            self.t_delta.get() > self.t_total
            and
            (
                self.y_pid.atSetpoint()
                and
                self.x_pid.atSetpoint()
                and
                self.w_pid.atSetpoint()
            )
        )
        
    def end(self, interrupted: bool):
        self.subsystem.set_driver_centric((0, 0), 0)
        
class RouteTarget(SubsystemCommand[SwerveDrivetrain]):
    
    def __init__(self, subsystem: SwerveDrivetrain, target: Pose2d, threshold: tuple = (2, 2, 1), grid: bool = False):
        
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.target: Pose2d = target
        self.y_pid: PIDController = PIDController(0.1, 0, 0)
        self.x_pid: PIDController = PIDController(0.1, 0, 0)
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
            self.w_pid.calculate(self.pose.rotation(), self.target.rotation())
            * config.calculated_max_angular_vel
        )
        
        if self.grid:
            if not self.y_done:
                dx = 0
            
            if self.y_done:
                dy = 0
                
        # DOES NOT TAKE VELOCITY INTO CONSIDERATION
                
        if self.y_pid.atSetpoint():
            self.y_done = True
            
        if self.x_pid.atSetpoint():
            self.x_done = True
            
        if self.w_pid.atSetpoint():
            self.w_done = True
        
        self.subsystem.set_robot_centric((dy, -dx), d_theta)
        
        
        
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
        self.april_tags: list[Pose3d] = None
        self.grid_tags: list[Pose3d] = None
        self.station_tag: Pose3d = None
        self.team_station: Pose2d = None
        self.grid: bool = False
        
    def initialize(self):
        
        poses = constants.Poses
        
        dis = lambda target: target.translation().distance(self.odometry.getPose().translation())
        
        # Find team and april tags
        if config.active_team == config.Team.blue:
            self.april_tags = [tag for tag in constants.ApriltagPositionDictBlue.values()]
            self.grid_tags = [self.april_tags[0], self.april_tags[1], self.april_tags[2]]
            self.station_tag = self.april_tags[4]
            self.team_station = Pose2d(poses.load_single['blue'], Rotation2d(90))
        else:
            self.april_tags = [tag for tag in constants.ApriltagPositionDictRed.values()]
            self.grid_tags = [self.april_tags[5], self.april_tags[6], self.april_tags[7]]
            self.station_tag = self.april_tags[3]
            self.team_station = Pose2d(poses.load_single['red'], Rotation2d(-90))
            
        # find grid targets
        grid_pos = []
        for tag in self.grid_tags:
            tag2d = tag.toPose2d()
            # the left, front, and right nodes relative to the tag
            grid_pos.append(Pose2d(poses.node_left, Rotation2d(180)).relativeTo(tag2d))
            grid_pos.append(Pose2d(poses.node_front, Rotation2d(180)).relativeTo(tag2d))
            grid_pos.append(Pose2d(poses.node_right, Rotation2d(180)).relativeTo(tag2d))

        # find station targets (single station, double stations)
        station_pos = []
        station_pos.append(self.team_station.relativeTo(self.station_tag.toPose2d()))
        station_pos.append(Pose2d(poses.load_double_left, Rotation2d(180)).relativeTo(self.station_tag.toPose2d()))
        station_pos.append(Pose2d(poses.load_double_right, Rotation2d(180)).relativeTo(self.station_tag.toPose2d()))
        
        for i in range(len(grid_pos)):
            ntcore.NetworkTableInstance.getDefault().getTable('Pose targets').putNumberArray(i, [
                grid_pos[i].translation().x(),
                grid_pos[i].translation().y(),
                grid_pos[i].rotation().radians()
            ])
        
        for i in range(len(station_pos)):
            ntcore.NetworkTableInstance.getDefault().getTable('Pose targets').putNumberArray(i + len(grid_pos), [
                station_pos[i].translation().x(),
                station_pos[i].translation().y(),
                station_pos[i].rotation().radians()
            ])
        
        # if the active route type is grid, select grid target
        if config.active_route == config.Route.grid:
            self.grid = True
            # if the active grid is auto, select the closest grid target
            if config.active_grid == 0:
                self.target = min(
                    grid_pos,
                    key=dis
                )
            else:
                # select grid target
                self.target = grid_pos[config.active_grid - 1]
        # if the active route type is station, select station target
        elif config.active_route == config.Route.station:
            # if the active station is auto, select the closest station target
            if config.active_station == config.Station.auto:
                self.target = min(
                    station_pos,
                    key=dis
                )
            else:
                # select station target
                self.target = station_pos[config.active_station - 1]
        # if the active route type is auto, select the closest target
        else:
            targets_total:list[Pose2d] = grid_pos + station_pos
            self.target = min(
                targets_total,
                key=dis
            )
            
        # run the route
        commands2.CommandScheduler.get_instance().schedule(
            RouteTarget(self.drivetrain, self.target, grid=self.grid)
        )
        
    def isFinished(self):
        return True