from __future__ import annotations

from typing import TypeVar, Generic, Union, Callable, List

import event
import robot
import unit


_TType = TypeVar("_TType", bound=robot.event.Type)


class SetRobotUnitValueResponseProcessor(
    Generic[_TType],
    event.Processor[
        _TType,
        Union[event.Success[unit.UnitValue], event.Failure[robot.event.SetHeadingFailureValue]],
        None,
        None
    ]
):
    _success_handlers: List[Callable[[unit.UnitValue], None]]
    _correction_handlers: List[Callable[[unit.UnitValue], None]]

    def __init__(self) -> None:
        self._success_handlers = []
        self._correction_handlers = []

    def add_success_handler(self, handler: Callable[[unit.UnitValue], None]) -> None:
        self._success_handlers.append(handler)

    def add_correction_handler(self, handler: Callable[[unit.UnitValue], None]) -> None:
        self._correction_handlers.append(handler)

    def process(
            self,
            event_: event.Event[event.Success[unit.UnitValue] | event.Failure[robot.event.UnitValueFailureValue]]
    ) -> None:
        if isinstance(event_.value, robot.event.UnitValueCorrection):
            try:
                self._handle_correction(event_.value.correction)
            except Exception as err:
                raise err from event_.value.error
        elif isinstance(event_.value, robot.event.UnitValueFailureValue):
            raise event_.value
        else:
            raise ValueError(event_.value)

    def _handle_success(self, value: unit.UnitValue) -> None:
        for handler in self._success_handlers:
            handler(value)

    def _handle_correction(self, value: unit.UnitValue) -> None:
        for handler in self._correction_handlers:
            handler(value)

