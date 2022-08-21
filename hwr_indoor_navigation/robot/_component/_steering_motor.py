from __future__ import annotations

from interface import WithStartup, WithShutdown
from unit import UnitValue
import Adafruit_PCA9685


class SteeringMotor(WithStartup, WithShutdown):
    _heading: UnitValue | None
    _pwm: Adafruit_PCA9685.PCA9685 | None

    def __init__(self):
        self._heading = UnitValue(330, "steering_motor_pwm_frequency")
        self._pwm = None

    def startup(self) -> None:
        self._pwm = Adafruit_PCA9685.PCA9685()
        self._pwm.set_pwm_freq(50)

    def set_heading(self, heading: UnitValue) -> None:
        if self._pwm is None:
            raise RuntimeError("Motor is has not started")

        self._pwm.set_pwm(0, 0, heading.to("steering_motor_pwm_frequency").value)

        self._heading = heading

    def get_heading(self) -> UnitValue:
        return self._heading

    def shutdown(self) -> None:
        pass
