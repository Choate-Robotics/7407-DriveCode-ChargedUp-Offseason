from wpimath.geometry import Pose3d, Translation3d, Rotation3d

# limelight offsets from robot origin (in meters)
limelight_offset: Pose3d = {
    "front": Pose3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0.0, 0.0, 0.0)),
    "back": Pose3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0.0, 0.0, 0.0)),
}
