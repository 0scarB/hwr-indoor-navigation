from enum import Enum
from typing import Union

from app.event import Type as _AppEventType


class _LocalEventType(str, Enum):
    SET_HEADING = "set_heading"
    SET_MOVEMENT = "set_movement"
    CAPTURED_LIDAR_DATA = "capture_lidar_data"


EventType = Union[_AppEventType, _LocalEventType]
