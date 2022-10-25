from __future__ import annotations

import typing
from typing import Callable, List, TypeVar, Tuple, ClassVar

import event
from unit import UnitValue
from .._type import *


_TEvent = TypeVar("_TEvent", bound=event.Event)
_TFailureValue = TypeVar("_TFailureValue", bound=UnitValueFailureValue)
_TCorrection = TypeVar("_TCorrection", bound=UnitValueCorrection)
_TExpectedError = TypeVar("_TExpectedError", bound=Exception)


class UnitValueRequestProcessor(
    typing.Generic[_TEvent, _TFailureValue, _TCorrection, _TExpectedError],
    event.Processor[
        _TEvent,
        UnitValue,
        event.Success[UnitValue],
        event.Failure[_TFailureValue]
    ],
):
    _EXPECTED_ERROR: ClassVar[_TExpectedError]
    _FAILURE_VALUE_TYPE: ClassVar[typing.Type[_TFailureValue]]
    _CORRECTION_TYPE: ClassVar[typing.Type[_TCorrection]]

    _handlers: List[Callable[[UnitValue], Tuple[False, UnitValue] | Tuple[True, None]]]

    def __init__(self) -> None:
        super().__init__()
        self._handlers = []

    def add_handler(self, handler: Callable[[UnitValue], Tuple[False, UnitValue] | Tuple[True, None]]) -> None:
        self._handlers.append(handler)

    def _process(
            self,
            event_: event.Event[_TEvent, UnitValue]
    ) -> event.Success[UnitValue] | event.Failure[_TFailureValue]:
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
