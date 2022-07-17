from __future__ import annotations

from typing import List, Tuple

import app.event
import robot.event
import unit
from ... import _error as error


class SetRobotHeadingPublisher(
    app.event.Publisher[
        robot.event.Type.SET_HEADING,
        unit.UnitValue
    ]
):
    _heading: unit.UnitValue | None

    def __init__(self) -> None:
        self._heading = None

    def set_heading(self, heading: unit.UnitValue) -> None:
        self._heading = heading

    def publish(self) -> List[
        Tuple[robot.event.Type.SET_HEADING, unit.UnitValue]
        | Tuple[robot.event.Type.SET_HEADING, unit.UnitValue, app.event.LogLevel]
    ]:
        if self._heading is None:
            raise error.HeadingNotSet(
                "Cannot publish set robot heading event without heading value being set first"
            )

        return [(
            robot.event.Type.SET_HEADING,
            self._heading
        )]
