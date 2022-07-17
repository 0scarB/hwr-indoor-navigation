from interface import WithStartup, WithShutdown
from unit import UnitValue


class SteeringMotor(WithStartup, WithShutdown):
    _heading: UnitValue

    def __init__(self):
        self._heading = UnitValue(0, "steering_motor_pwm_frequency")

    def startup(self) -> None:
        # TODO: Real implementation
        print(f"starting {type(self).__name__}")

    def set_heading(self, heading: UnitValue) -> None:
        """
        Set the motor's speed.

        A positive speed will move the motor forward.
        A negative speed will move the motor backward.
        Speed 0 will stop moving the motor.
        """
        self._heading = heading
        # TODO: Real implementation
        print(f"set robot heading to {self._heading}")

    def get_heading(self) -> UnitValue:
        return self._heading

    def shutdown(self) -> None:
        # TODO: Real implementation
        print(f"shutting down {type(self).__name__}")
