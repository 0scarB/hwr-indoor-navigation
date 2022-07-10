from enum import Enum
from typing import Union

from event import Type as _GlobalEventType


class _LocalEventType(str, Enum):
    SET_HEADING = "set_heading"
    SET_MOVEMENT = "set_movement"
    CAPTURED_LIDAR_DATA = "capture_lidar_data"


EventType = Union[_GlobalEventType, _LocalEventType]