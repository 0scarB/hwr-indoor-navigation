from unit import UnitValue


class SteeringMotor:
    _heading: UnitValue

    def __init__(self):
        self._heading = UnitValue(0, "steering_motor_pwm_frequency")

    def set_heading(self, heading: UnitValue) -> None:
        """
        Set the motor's speed.

        A positive speed will move the motor forward.
        A negative speed will move the motor backward.
        Speed 0 will stop moving the motor.
        """
        self._heading = heading

    def get_heading(self) -> UnitValue:
        return self._heading
