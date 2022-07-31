import math

import RPi.GPIO as GPIO

from interface import WithStartup, WithShutdown
from unit import UnitValue, global_converter


class ForwardBackwardMotor(WithStartup, WithShutdown):
    _WHEEL_DIAMETER_IN_METERS = 0.065
    _WHEEL_ROTATIONS_PER_PWM_DUTY_CYCLE = 0.014

    _MOTOR_A_EN = 17
    _MOTOR_A_PIN1 = 27
    _MOTOR_A_PIN2 = 18

    _speed: UnitValue
    _pwm_a = GPIO.PWM | None

    def __init__(self):
        self._speed = UnitValue(0, "forward_backward_motor_pwm_duty_cycle")
        self._pwm_a = None

    def startup(self) -> None:
        GPIO.cleanup()  # Reset the high and low levels of the GPIO port
        GPIO.setwarnings(False)  # Ignore some insignificant errors
        GPIO.setmode(GPIO.BCM)  # Choose BCM encoding for port

        GPIO.setup(self._MOTOR_A_EN, GPIO.OUT)
        GPIO.setup(self._MOTOR_A_PIN1, GPIO.OUT)
        GPIO.setup(self._MOTOR_A_PIN2, GPIO.OUT)

        self.shutdown()  # Avoid motor starting to rotate automatically after initialization
        self._pwm_a = GPIO.PWM(self._MOTOR_A_EN, 1000)

        self._add_unit_value_conversion()

    def set_speed(self, speed: UnitValue) -> None:
        if self._pwm_a is None:
            raise RuntimeError("Motor is has not started")

        pwm_speed = speed.to("forward_backward_motor_pwm_duty_cycle").value

        if pwm_speed > 0:
            GPIO.output(self._MOTOR_A_PIN1, GPIO.LOW)
            GPIO.output(self._MOTOR_A_PIN2, GPIO.HIGH)
        else:
            GPIO.output(self._MOTOR_A_PIN1, GPIO.HIGH)
            GPIO.output(self._MOTOR_A_PIN2, GPIO.LOW)

        self._pwm_a.start(100)
        self._pwm_a.ChangeDutyCycle(abs(speed.to("forward_backward_motor_pwm_duty_cycle").value))

        self._speed = speed

    def shutdown(self) -> None:
        GPIO.output(self._MOTOR_A_PIN1, GPIO.LOW)
        GPIO.output(self._MOTOR_A_PIN2, GPIO.LOW)
        GPIO.output(self._MOTOR_A_EN, GPIO.LOW)

    def _add_unit_value_conversion(self) -> None:
        global_converter.add_linear_conversion(
            "forward_backward_motor_pwm_duty_cycle",
            "m/s",
            math.pi * self._WHEEL_DIAMETER_IN_METERS * self._WHEEL_ROTATIONS_PER_PWM_DUTY_CYCLE
        )
