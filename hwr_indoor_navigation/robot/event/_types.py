from dataclasses import dataclass
from enum import Enum
from typing import Union

from app.event import Type as _AppType


class _LocalType(str, Enum):
    SET_HEADING = "set_heading"
    SET_SPEED = "set_movement"
    CAPTURED_LIDAR_DATA = "capture_lidar_data"


Type = Union[_AppType, _LocalType]
