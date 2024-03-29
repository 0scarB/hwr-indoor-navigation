from __future__ import annotations

from typing import List, Tuple

import event as global_event
import robot
import unit
from ... import _error as error


class SetRobotSpeedPublisher(
    global_event.Publisher[
        robot.event.Type.SET_SPEED,
        unit.UnitValue
    ]
):
    _speed: unit.UnitValue | None

    def __init__(self) -> None:
        self._speed = None

    def set_speed(self, heading: unit.UnitValue) -> None:
        self._speed = heading

    def publish(self) -> List[
        Tuple[robot.event.Type.SET_SPEED, unit.UnitValue]
        | Tuple[robot.event.Type.SET_SPEED, unit.UnitValue, global_event.LogLevel]
    ]:
        if self._speed is None:
            raise error.HeadingNotSet(
                "Cannot publish set robot speed event without heading value being set first"
            )

        return [(
            robot.event.Type.SET_SPEED,
            self._speed
        )]
