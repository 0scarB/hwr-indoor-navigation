from dataclasses import dataclass

from unit import UnitValue


@dataclass(frozen=True)
class Config:
    movement_speed: UnitValue = UnitValue(100, "forward_backward_motor_pwm_duty_cycle")
    steering_increment: UnitValue = UnitValue(1, "steering_motor_pwm_frequency")
    forward_key: str = "w"
    backward_key: str = "s"
    left_key: str = "a"
    right_key: str = "d"
