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
        successes = event_.value.successes
        failures = event_.value.failures

        if len(successes) + len(failures) != 1:
            raise ValueError("Expected single response")

        if len(successes) == 1:
            self._handle_success(successes[0].value)
            return

        failure = failures[0]
        if isinstance(failure, robot.event.UnitValueCorrection):
            try:
                self._handle_correction(failure.correction)
            except Exception as err:
                raise err from event_.value.error
        elif isinstance(failure, robot.event.UnitValueFailureValue):
            raise failure.error
        else:
            raise ValueError(failure)

    def _handle_success(self, value: unit.UnitValue) -> None:
        for handler in self._success_handlers:
            handler(value)

    def _handle_correction(self, value: unit.UnitValue) -> None:
        for handler in self._correction_handlers:
            handler(value)

