import ntcore, config, time

from wpimath.geometry import Pose3d, Translation3d, Rotation3d

from robotpy_toolkit_7407.sensors.limelight.limelight import VisionEstimator


class Limelight():
    '''
    A class for interfacing with the limelight camera.'''
    
    def __init__(self, origin_offset: Pose3d, name: str = "limelight"):
        '''
        
        :param origin_offset: The offset of the limelight from the robot's origin
        
        :param name: The name of the limelight network table. This is used to differentiate between multiple limelights. 
        If you have multiple limelights, you must give them different names in order for their values to be read correctly.
        If you only have one limelight, you can leave this as the default value.'''
        
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.table: ntcore.NetworkTable = self.nt.getTable(name)
        self.tx: float = 0
        self.ty: float = 0
        self.tv: float = 0
        self.ta: float = 0
        self.origin_offset: Pose3d = origin_offset
        self.drive_cam = False
        self.pipeline: config.limelight_pipeline = config.limelight_pipeline['retroreflective']
        self.t_class = None
        self.force_update = False
        self.botpose_blue: Pose3d = Pose3d(Translation3d(0, 0, 0), Rotation3d(0, 0, 0))
        self.botpose_red: Pose3d = Pose3d(Translation3d(0, 0, 0), Rotation3d(0, 0, 0))
        self.botpose: Pose3d = Pose3d(Translation3d(0, 0, 0), Rotation3d(0, 0, 0))
        
    def enable_force_update(self):
        '''
        Forces the limelight to update its values. This is useful if you want to get the limelight's values multiple times in one loop.
        
        This is disabled by default.
        
        Force update can also be enabled on a method-by-method basis by passing force_update=True to the method.
        
        When disabled, the limelight will only update its values once per loop with the update() function.'''
        self.force_update = True
        
    def disable_force_update(self):
        '''
        Disables the force update. When this is disabled, the limelight will only update its values once per loop.'''
        self.force_update = False

    def set_pipeline_mode(self, mode: config.limelight_pipeline) -> None:
        '''
        Sets the pipeline mode of the limelight will be using (Feeducial, Retroreflective, Neural, etc.)
        These modes are defined in config.py and match the numbers used in the limelight web interface.
        
        :param mode: The pipeline int to set the limelight to
        
        '''
        self.table.getEntry("pipeline").setInteger(mode)
        self.pipeline = mode
        
    def get_pipeline_mode(self) -> config.limelight_pipeline:
        '''
        Gets the pipeline mode of the limelight will be using (Feeducial, Retroreflective, Neural, etc.) as an integer.
        
        :return config.limelight_pipeline: The pipeline mode the limelight is currently using
        '''
        pipelime = self.table.getEntry("pipeline").getInteger(0)
        if self.pipeline != pipelime:
            self.pipeline = pipelime
        return self.pipeline
        
    def set_led_mode(self, mode: config.limelight_led_mode) -> None:
        '''
        Changes the LED mode of the limelight.
        
        :param mode: The LED mode to set the limelight to       
        '''
        self.table.getEntry("ledMode").setInteger(mode)
        
    def get_led_mode(self) -> config.limelight_led_mode:
        
        return self.table.getEntry("ledMode").getInteger(0)
    
    def set_cam_vision(self):
        '''
        Sets the limelight to use the camera for vision processing.
        '''
        self.table.getEntry("camMode").setInteger(0)
        self.drive_cam = False
        
    def set_cam_driver(self):
        '''
        Sets the limelight to use the camera for driver vision.
        '''
        self.table.getEntry("camMode").setInteger(1)
        self.drive_cam = True
        
    def get_cam_mode(self):
        '''
        Gets the camera mode of the limelight (Vision or Driver)
        :return bool: True if the limelight is in driver mode, False if it is in vision mode
        '''
        mode = self.table.getEntry("camMode").getInteger(0)
        if self.drive_cam != mode:
            self.drive_cam = mode
        return self.drive_cam
        
    def get_neural_classId(self, force_update: bool = False):
        '''
        Gets the neural classId of the limelight. 
        This id number is defined in config.py and matches the numbers used in the limelight web interface.
        
        :return config.neural_classId: The neural classId the limelight is currently using
        '''
        if force_update or self.force_update:
            self.update()
        if self.pipeline != config.limelight_pipeline['neural']:
            return False
        if self.tv == 0:
            return None
        self.t_class = self.table.getEntry("tclass").getInteger(0)
        return self.t_class
        
    def update(self):
        '''
        Updates the tx, ty, and tv values of the limelight Manually. 
        For proper use, this should be called in the main event loop.
        '''
        self.tx = self.table.getNumber("tx",0)
        self.ty = self.table.getNumber("ty",0)
        self.tv = self.table.getNumber("tv",0)
        self.ta = self.table.getNumber("ta",0)
        self.botpose_red = self.table.getEntry("botpose_wpired").getDoubleArray([0, 0, 0, 0, 0, 0])
        self.botpose_blue = self.table.getEntry("botpose_wpiblue").getDoubleArray([0, 0, 0, 0, 0, 0])
        self.botpose = self.table.getEntry("botpose").getDoubleArray([0, 0, 0, 0, 0, 0])
        
    def target_exists(self, force_update: bool = False):
        '''
        Checks if a target exists within the limelight's field of view.
        
        :param force_update: If True, the limelight variables be updated before checking if a target exists. Defaults to False.
        
        :return bool: True if a target exists, False if not
        '''
        if self.force_update or force_update:
            self.update()
        return self.tv > 0.0

    def get_target(self, force_update: bool = False):
        '''
        Gets the tx, ty values of a target if it exists.
        
        :param force_update: If True, the limelight variables be updated before checking if a target exists. Defaults to False.
        
        :return list: [tx, ty] if a target exists
        
        :return None: if no target exists
        '''
        if self.force_update or force_update:
            self.update()
        if self.tv < 1:
            return None
        return [self.tx, self.ty, self.ta]

    def get_bot_pose(self, team: config.team = None, round_to: int = 4, force_update: bool = False):
        '''
        Gets the pose of the robot relative to the field using the feducial pipeline.
        This uses the botpose values from the limelight configuration, which are relative to the alliance wall.
        To call this properly, you must set the pipeline to feducial.
        :param team: The team color of the robot. This is used to get the botpose of the robot relative to the alliance wall. can be None if you don't want to use it. 0 for red, 1 for blue.
        :param round_to: The number of decimal places to round the botpose to. Defaults to 4.
        :param force_update: If True, the limelight variables be updated before getting the botpose. Defaults to False.
        
        :return list: [x, y, z, pitch, yaw, roll] if a target exists
        
        :return None: if no targets exists
        :return False: if the pipeline is not set to feducial
        '''
        if self.force_update or force_update:
            self.update()
        if self.pipeline != config.limelight_pipeline['feducial']:
            return False
        elif not self.target_exists:
            return None
        else:
            botpose: list = []

            if team == config.team.get('red') or team == 0:
                botpose = self.botpose_red
            elif team == config.team.get('blue') or team == 1:
                botpose = self.botpose_blue
            else:
                botpose = self.botpose
            botpose = [round(i, round_to) for i in botpose]
            pose = Pose3d(
                Translation3d(botpose[0], botpose[1], botpose[2]),
                Rotation3d(botpose[3], botpose[4], botpose[5])
            )
            return pose

class LimelightController(VisionEstimator):
    
    def __init__(self, limelight_list: list[Limelight]):
        super().__init__()
        self.limelights = limelight_list
        
    def get_estimated_robot_pose(self) -> list[Pose3d] | None:
        poses = []
        for limelight in self.limelights:
            if limelight.target_exists() and limelight.get_pipeline_mode() == config.limelight_pipeline['feducial']:
                poses.append(limelight.get_bot_pose())
        if len(poses) > 0:
            return poses
        else:
            return None