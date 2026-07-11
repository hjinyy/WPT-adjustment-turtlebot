import pytest

from wpt_adjustment_turtlebot.map_localization import MapPose, MapPoseEKF


def test_ekf_predicts_forward_using_filtered_heading():
    ekf = MapPoseEKF(process_variance=0.01, measurement_variance=0.04, outlier_distance_m=0.5)
    ekf.update([MapPose(0.0, 0.0, 0.0)])

    pose = ekf.predict(linear_m_s=0.1, angular_rad_s=0.0, dt_s=1.0)

    assert pose.x_m == pytest.approx(0.1)
    assert pose.y_m == pytest.approx(0.0)


def test_ekf_measurement_update_smooths_position_jump():
    ekf = MapPoseEKF(process_variance=0.01, measurement_variance=0.1, outlier_distance_m=2.0)
    ekf.update([MapPose(0.0, 0.0, 0.0)])

    pose = ekf.update([MapPose(1.0, 0.0, 0.0)])

    assert 0.0 < pose.x_m < 1.0
