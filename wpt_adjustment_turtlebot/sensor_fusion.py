"""Small dependency-free filters for image-space alignment errors."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ErrorEstimate:
    x_error: float
    y_error: float
    angle_error_deg: float
    has_observation: bool


class _ScalarKalman:
    def __init__(self, process_variance: float, measurement_variance: float) -> None:
        self.value = 0.0
        self.variance = 1.0
        self.process_variance = float(process_variance)
        self.measurement_variance = float(measurement_variance)

    def predict(self) -> float:
        self.variance += self.process_variance
        return self.value

    def update(self, measurement: float, confidence: float = 1.0) -> float:
        confidence = max(0.01, min(1.0, float(confidence)))
        measurement_variance = self.measurement_variance / confidence
        self.predict()
        gain = self.variance / (self.variance + measurement_variance)
        self.value = self.value + gain * (float(measurement) - self.value)
        self.variance = (1.0 - gain) * self.variance
        return self.value


class ErrorKalmanFilter:
    """Independent 1D Kalman filters for x, y, and angle image errors."""

    def __init__(self, process_variance: float = 1.0, measurement_variance: float = 4.0) -> None:
        self._x = _ScalarKalman(process_variance, measurement_variance)
        self._y = _ScalarKalman(process_variance, measurement_variance)
        self._angle = _ScalarKalman(process_variance, measurement_variance)

    def predict(self) -> ErrorEstimate:
        return ErrorEstimate(
            x_error=self._x.predict(),
            y_error=self._y.predict(),
            angle_error_deg=self._angle.predict(),
            has_observation=False,
        )

    def update(
        self,
        *,
        x_error: float,
        y_error: float,
        angle_error_deg: float,
        confidence: float = 1.0,
    ) -> ErrorEstimate:
        return ErrorEstimate(
            x_error=self._x.update(x_error, confidence),
            y_error=self._y.update(y_error, confidence),
            angle_error_deg=self._angle.update(angle_error_deg, confidence),
            has_observation=True,
        )
