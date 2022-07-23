from dataclasses import dataclass

from unit import UnitValue


@dataclass(frozen=True)
class Config:
    movement_speed: UnitValue
    steering_increment: UnitValue
    forward_key: str
    backward_key: str
    left_key: str
    right_key: str
