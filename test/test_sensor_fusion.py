from wpt_adjustment_turtlebot.sensor_fusion import ErrorKalmanFilter


def test_filter_update_moves_estimate_toward_measurement():
    filt = ErrorKalmanFilter(process_variance=1.0, measurement_variance=4.0)

    estimate = filt.update(x_error=20.0, y_error=0.0, angle_error_deg=10.0)

    assert 0.0 < estimate.x_error <= 20.0
    assert 0.0 < estimate.angle_error_deg <= 10.0
    assert estimate.has_observation


def test_prediction_without_measurement_keeps_last_estimate_available():
    filt = ErrorKalmanFilter(process_variance=1.0, measurement_variance=4.0)
    filt.update(x_error=12.0, y_error=-3.0, angle_error_deg=4.0)

    estimate = filt.predict()

    assert estimate.x_error != 0.0
    assert estimate.y_error != 0.0
    assert estimate.angle_error_deg != 0.0
    assert not estimate.has_observation


def test_low_confidence_measurement_has_smaller_effect_than_high_confidence():
    low = ErrorKalmanFilter(process_variance=1.0, measurement_variance=4.0)
    high = ErrorKalmanFilter(process_variance=1.0, measurement_variance=4.0)

    low_estimate = low.update(x_error=20.0, y_error=0.0, angle_error_deg=0.0, confidence=0.2)
    high_estimate = high.update(x_error=20.0, y_error=0.0, angle_error_deg=0.0, confidence=1.0)

    assert high_estimate.x_error > low_estimate.x_error
