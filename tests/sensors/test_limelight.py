import ntcore

import config

import pytest

from unittest.mock import Mock, MagicMock

import random

from wpimath.geometry import Pose3d, Translation3d, Rotation3d

from sensors import Limelight

@pytest.fixture
def limelight():
    print("limelight setup")
    
    yield Limelight(Translation3d(0, 0, 0), Rotation3d(0, 0, 0))
    print("limelight teardown")

def test_init(limelight):
    assert limelight.origin_offset == Translation3d(0, 0, 0)
    assert limelight.rotation_offset == Rotation3d(0, 0, 0)
    assert limelight.drive_cam == False
    assert limelight.pipeline == config.limelight_pipeline['retroreflective']
    
def test_set_pipeline_mode(limelight):
    limelight.set_pipeline_mode(config.limelight_pipeline['feducial'])
    assert limelight.get_pipeline_mode() == config.limelight_pipeline['feducial']
    
def test_set_cam_driver(limelight):
    limelight.set_cam_driver()
    assert limelight.drive_cam == True
    
def test_set_cam_vision(limelight):
    limelight.set_cam_vision()
    assert limelight.drive_cam == False
    
def test_update(limelight):
    limelight.update()
    
def test_get_target_none(limelight):
    assert limelight.get_target() == None
    
@pytest.mark.xfail
def test_get_target(limelight):
    tx, ty = random.randint(0, 100), random.randint(0, 100)
    limelight.table.getEntry("tv").getDouble = MagicMock(return_value=1)
    limelight.table.getEntry("tx").getDouble = MagicMock(return_value=tx)
    limelight.table.getEntry("ty").getDouble = MagicMock(return_value=ty)
    limelight.update()
    limelight.get_target().assert_called_with([tx, ty])