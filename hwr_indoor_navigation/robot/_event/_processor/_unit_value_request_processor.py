from __future__ import annotations

import typing
from typing import Callable, List, TypeVar, Tuple, ClassVar

import event
import unit
from .._type import *


_TEvent = TypeVar("_TEvent", bound=event.Event)
_TFailureValue = TypeVar("_TFailureValue", bound=UnitValueFailureValue)
_TCorrection = TypeVar("_TCorrection", bound=UnitValueCorrection)
_TExpectedError = TypeVar("_TExpectedError", bound=Exception)


class UnitValueRequestProcessor(
    typing.Generic[_TEvent, _TFailureValue, _TCorrection, _TExpectedError],
    event.Processor[
        _TEvent,
        unit.UnitValue,
        event.Success[unit.UnitValue],
        event.Failure[_TFailureValue]
    ],
):
    _EXPECTED_ERROR: ClassVar[_TExpectedError]
    _FAILURE_VALUE_TYPE: ClassVar[typing.Type[_TFailureValue]]
    _CORRECTION_TYPE: ClassVar[typing.Type[_TCorrection]]

    _handlers: List[Callable[[unit.UnitValue], Tuple[False, unit.UnitValue] | Tuple[True, None]]]

    def __init__(self) -> None:
        self._handlers = []

    def add_handler(self, handler: Callable[[unit.UnitValue], Tuple[False, unit.UnitValue] | Tuple[True, None]]) -> None:
        self._handlers.append(handler)

    def process(
            self,
            event_: event.Event[_TEvent, unit.UnitValue]
    ) -> event.Success[unit.UnitValue] | event.Failure[_TFailureValue]:
        for handler in self._handlers:
            try:
                was_successfully_handled, correction = handler(event_.value)
                if not was_successfully_handled:
                    return event.Failure(
                        self._CORRECTION_TYPE(
                            error=self._EXPECTED_ERROR(
                                f"Could not handle correction {correction}",
                            ),
                            correction=correction,
                        )
                    )
            except self._EXPECTED_ERROR as err:
                return self._FAILURE_VALUE_TYPE(err)

        return event.Success(event_.value)
