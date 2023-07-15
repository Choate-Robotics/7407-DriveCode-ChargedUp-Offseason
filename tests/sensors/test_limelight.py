import ntcore

import config

import pytest

from unittest.mock import Mock, MagicMock

import random

import ntcore

from wpimath.geometry import Pose3d, Translation3d, Rotation3d

from sensors import Limelight

@pytest.fixture
def limelight():
    a = Limelight(Pose3d(0, 0, 0, Rotation3d(0, 0, 0)))   
    a.enable_force_update()
    yield a

def test_init(limelight):
    assert limelight.origin_offset == Pose3d(0, 0, 0, Rotation3d(0, 0, 0))
    assert limelight.drive_cam == False
    assert limelight.pipeline == config.limelight_pipeline['retroreflective']
    
def test_set_pipeline_mode(limelight):
    limelight.set_pipeline_mode(config.limelight_pipeline['feducial'])
    assert limelight.get_pipeline_mode() == config.limelight_pipeline['feducial']
    limelight.pipeline = config.limelight_pipeline['retroreflective']
    assert limelight.get_pipeline_mode() == config.limelight_pipeline['feducial']

def test_target_exists(limelight):
    assert limelight.target_exists() == False
    limelight.table.getEntry("tv").setBoolean(True)
    assert limelight.target_exists() == True
    
def test_neural_class_id(limelight):
    assert limelight.get_neural_classId() == False
    limelight.set_pipeline_mode(config.limelight_pipeline['neural'])
    assert limelight.get_neural_classId() == None
    limelight.table.getEntry("tv").setBoolean(True)
    limelight.table.getEntry("tclass").setInteger(1)
    assert limelight.get_neural_classId() == 1
    
def test_set_cam_driver(limelight):
    limelight.set_cam_driver()
    assert limelight.drive_cam == True
    
def test_set_cam_vision(limelight):
    limelight.set_cam_vision()
    assert limelight.drive_cam == False
    
def test_update(limelight):
    limelight.update()
    
def test_disable_force_update(limelight):
    limelight.disable_force_update()
    assert limelight.force_update == False
    
def test_get_target_none(limelight):
    assert limelight.get_target() == None
    
def test_get_bot_pose_wrong_pipeline(limelight):
    assert limelight.get_bot_pose() == False
    
def test_get_bot_pose_none(limelight):
    limelight.set_pipeline_mode(config.limelight_pipeline['feducial'])
    assert limelight.get_pipeline_mode() == config.limelight_pipeline['feducial']
    assert limelight.get_bot_pose() == None

@pytest.fixture
def set_limelight_random_values():
    def make(table: ntcore.NetworkTable):
        tx, ty = random.randint(0, 100), random.randint(0, 100)
        table.getEntry('tx').setDouble(tx)
        table.getEntry('ty').setDouble(ty)
        table.getEntry('tv').setBoolean(1)
        if table.getEntry('tv').getBoolean(False) == 1:
            return tx, ty
        return None
    return make

@pytest.fixture
def set_limelight_random_botpose():
    def make(table: ntcore.NetworkTable, team: str = None):
        entry = 'botpose'
        if team == 'red':
            entry = 'botpose_wpired'
        if team == 'blue':
            entry = 'botpose_wpiblue'
        botpose = [random.randint(0, 100) for i in range(6)]
        table.getEntry('tv').setBoolean(1)
        table.getEntry(entry).setDoubleArray(botpose)
        if table.getEntry('tv').getBoolean(False) == 1:
            pose = Pose3d(
                Translation3d(botpose[0], botpose[1], botpose[2]),
                Rotation3d(botpose[3], botpose[4], botpose[5])
            )
            return pose
        return None
    return make
    
def test_get_limelight_target_values(limelight: Limelight, set_limelight_random_values):
    tx, ty = set_limelight_random_values(table=limelight.table)
    assert limelight.get_target() == [tx, ty]
    
def test_get_limelight_botpose_values(limelight: Limelight, set_limelight_random_botpose):
    limelight.set_pipeline_mode(config.limelight_pipeline['feducial'])
    gen = set_limelight_random_botpose(table=limelight.table)
    red = set_limelight_random_botpose(table=limelight.table, team='red')
    blue = set_limelight_random_botpose(table=limelight.table, team='blue')
    assert limelight.get_bot_pose() == gen
    assert limelight.get_bot_pose(config.team['red']) == red
    assert limelight.get_bot_pose(config.team['blue']) == blue
    
def test_multiple_limelights(set_limelight_random_values):
    a = Limelight(Pose3d(0, 0, 0, Rotation3d(0, 0, 0)), "limelighta")
    b = Limelight(Pose3d(1, 1, 1, Rotation3d(0, 0, 0)), "limelightb")
    assert a.origin_offset == Pose3d(0, 0, 0, Rotation3d(0, 0, 0))
    assert b.origin_offset == Pose3d(1, 1, 1, Rotation3d(0, 0, 0))
    atx, aty = set_limelight_random_values(table=a.table)
    btx, bty = set_limelight_random_values(table=b.table)
    assert a.get_target(True) == [atx, aty]
    assert b.get_target(True) == [btx, bty]