from dataclasses import dataclass
from enum import Enum
from typing import Union

import event
import unit
from .._error import WrongHeading


class _LocalType(str, Enum):
    SET_HEADING = "set_heading"
    SET_SPEED = "set_movement"
    CAPTURED_LIDAR_DATA = "capture_lidar_data"


Type = Union[event.Type, _LocalType]


@dataclass(frozen=True)
class UnitValueFailureValue:
    error: Exception


@dataclass(frozen=True)
class UnitValueCorrection(UnitValueFailureValue):
    correction: unit.UnitValue


@dataclass(frozen=True)
class SetHeadingFailureValue(UnitValueFailureValue):
    error: WrongHeading


@dataclass(frozen=True)
class SetHeadingCorrection(SetHeadingFailureValue, UnitValueCorrection):
    pass


@dataclass(frozen=True)
class SetSpeedFailureValue(UnitValueFailureValue):
    error: WrongHeading


@dataclass(frozen=True)
class SetSpeedCorrection(SetSpeedFailureValue, UnitValueCorrection):
    pass
