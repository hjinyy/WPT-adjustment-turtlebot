import math

from wpt_adjustment_turtlebot.map_localization import MapPose, MapPoseEKF


def test_ekf_rejects_far_pose_outlier_and_wraps_yaw():
    ekf = MapPoseEKF(process_variance=0.01, measurement_variance=0.01, outlier_distance_m=0.2)
    pose = ekf.update([MapPose(1.0, 2.0, 3.13), MapPose(1.02, 1.99, -3.14), MapPose(9.0, 9.0, 0.0)])
    assert pose.x_m < 1.1
    assert abs(abs(pose.yaw_rad) - math.pi) < 0.05
