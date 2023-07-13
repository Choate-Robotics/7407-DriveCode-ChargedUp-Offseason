import ntcore

import config

from wpimath.geometry import Pose3d, Translation3d, Rotation3d

class Limelight():
    
    def __init__(self, origin_offset: Translation3d, rotation_offset: Rotation3d, name: str = "limelight"):
        
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.table = self.nt.getTable(name)
        self.tx = 0
        self.ty = 0
        self.tv = 0
        self.origin_offset = origin_offset
        self.rotation_offset = rotation_offset
        self.drive_cam = False
        self.pipeline: config.limelight_pipeline = config.limelight_pipeline['retroreflective']
        

    def set_pipeline_mode(self, mode: config.limelight_pipeline):
        self.table.getEntry("pipeline").setInteger(mode)
        self.pipeline = mode
        
    def get_pipeline_mode(self):
        pipelime = self.table.getEntry("pipeline").getInteger(0)
        if self.pipeline != pipelime:
            self.pipeline = pipelime
        return self.pipeline
        
    def set_cam_vision(self):
        self.table.getEntry("camMode").setInteger(0)
        self.drive_cam = False
        
    def set_cam_driver(self):
        self.table.getEntry("camMode").setInteger(1)
        self.drive_cam = True
        
    def get_cam_mode(self):
        mode = self.table.getEntry("camMode").getInteger(0)
        if self.drive_cam != mode:
            self.drive_cam = mode
        return self.drive_cam
        
    def update(self):
        self.tx = self.table.getEntry("tx").getDouble(0)
        self.ty = self.table.getEntry("ty").getDouble(0)
        self.tv = self.table.getEntry("tv").getDouble(0)
        
    def get_target(self):
        if self.tv == 0:
            return None
        self.tx = self.table.getEntry("tx").getDouble(0)
        self.ty = self.table.getEntry("ty").getDouble(0)
        return [self.tx, self.ty]
    
    def get_bot_pose(self, team: config.team = None, round_to: int = 4):
        if self.pipeline != config.limelight_pipeline['feducial']:
            return False
        elif self.tv == 0:
            return None
        else:
            botpose: list = []

            if team == config.team.get('red'):
                botpose = self.table.getEntry("botpose_wpired").getDoubleArray([0, 0, 0, 0, 0, 0])
            elif team == config.team.get('blue'):
                botpose = self.table.getEntry("botpose_wipblue").getDoubleArray([0, 0, 0, 0, 0, 0])
            else:
                botpose = self.table.getEntry("botpose").getDoubleArray([0, 0, 0, 0, 0, 0])
            botpose = [round(i, round_to) for i in botpose]
            return botpose
    