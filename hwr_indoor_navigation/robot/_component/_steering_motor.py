from __future__ import annotations

from interface import WithStartup, WithShutdown
from unit import UnitValue, global_converter
import Adafruit_PCA9685

from time import sleep


class SteeringMotor(WithStartup, WithShutdown):
    _PWM_FOR_0_DEGREES = 110
    _PWM_FOR_180_DEGREES = 540

    _heading: UnitValue | None
    _pwm: Adafruit_PCA9685.PCA9685 | None

    def __init__(self):
        self._heading = UnitValue(
            (self._PWM_FOR_180_DEGREES + self._PWM_FOR_0_DEGREES) / 2,
            "_steering_motor_pwm_duty_cycle"
        )
        self._pwm = None

    def startup(self) -> None:
        self._pwm = Adafruit_PCA9685.PCA9685()
        self._pwm.set_pwm_freq(50)
        self._add_unit_value_conversion()
        self.set_heading(self._heading)

        # while True:
        #     self.set_heading(UnitValue(45, "degrees"))
        #     sleep(2)
        #     self.set_heading(UnitValue(45 * 3/2, "degrees"))
        #     sleep(2)
        #     self.set_heading(UnitValue(90, "degrees"))
        #     sleep(2)
        #     self.set_heading(UnitValue(45 * 5/2, "degrees"))
        #     sleep(2)
        #     self.set_heading(UnitValue(135, "degrees"))
        #     sleep(2)

    def set_heading(self, heading: UnitValue) -> None:
        if self._pwm is None:
            raise RuntimeError("Motor is has not started")

        my_heading = heading.to("_steering_motor_pwm_duty_cycle").value
        self._pwm.set_pwm(0, 0, int(my_heading))
        print(f"My PWM: {my_heading} degrees={heading.to('degrees').value}")

        self._heading = heading

    def get_heading(self) -> UnitValue:
        return self._heading

    def shutdown(self) -> None:
        pass

    def _add_unit_value_conversion(self) -> None:
        global_converter.add_linear_conversion(
            "_steering_motor_pwm_duty_cycle",
            "degrees",
            180 / (self._PWM_FOR_180_DEGREES - self._PWM_FOR_0_DEGREES),
            target_starting_value=self._PWM_FOR_0_DEGREES,
        )
