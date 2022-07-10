from interface import WithStartup, WithShutdown
from unit import UnitValue


class ForwardBackwardMotor(WithStartup, WithShutdown):
    _speed: UnitValue
    _is_started: bool

    def __init__(self):
        self._add_unit_conversions()

        self._speed = UnitValue(0, "forward_backward_motor_pwm_duty_cycle")
        self._is_started = False

    def startup(self) -> None:
        pass

    def set_speed(self, speed: UnitValue) -> None:
        """
        Set the motor's speed.

        A positive speed will move the motor forward.
        A negative speed will move the motor backward.
        Speed 0 will stop moving the motor.
        """
        self._speed = speed

    def shutdown(self) -> None:
        pass

    def _add_unit_conversions(self) -> None:
        pass
