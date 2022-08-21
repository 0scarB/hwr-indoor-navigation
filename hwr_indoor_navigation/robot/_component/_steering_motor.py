from __future__ import annotations

from interface import WithStartup, WithShutdown
from unit import UnitValue, global_converter
import Adafruit_PCA9685


class SteeringMotor(WithStartup, WithShutdown):
    _PWM_FOR_0_DEGREES = 100
    _PWN_FOR_180_DEGREES = 560

    _heading: UnitValue | None
    _pwm: Adafruit_PCA9685.PCA9685 | None

    def __init__(self):
        self._heading = UnitValue(
            (self._PWN_FOR_180_DEGREES + self._PWM_FOR_0_DEGREES) / 2,
            "_steering_motor_pwm_duty_cycle"
        )
        self._pwm = None

    def startup(self) -> None:
        self._pwm = Adafruit_PCA9685.PCA9685()
        self._pwm.set_pwm_freq(50)

        self._add_unit_value_conversion()

    def set_heading(self, heading: UnitValue) -> None:
        if self._pwm is None:
            raise RuntimeError("Motor is has not started")

        self._pwm.set_pwm(0, 0, heading.to("_steering_motor_pwm_duty_cycle").value)

        self._heading = heading

    def get_heading(self) -> UnitValue:
        return self._heading

    def shutdown(self) -> None:
        pass

    def _add_unit_value_conversion(self) -> None:
        global_converter.add_linear_conversion(
            "_steering_motor_pwm_duty_cycle",
            "degrees",
            180 / (self._PWN_FOR_180_DEGREES - self._PWM_FOR_0_DEGREES)
        )
