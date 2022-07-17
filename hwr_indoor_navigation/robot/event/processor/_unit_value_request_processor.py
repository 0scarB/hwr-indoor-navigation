from __future__ import annotations

from typing import Callable, List, TypeVar, Generic, ClassVar

import app.event
import unit
from .._types import Type


_TExpectedError = TypeVar("_TExpectedError")


class UnitValueRequestProcessor(
    Generic[_TExpectedError],
    app.event.Processor[
        Type.SET_HEADING,
        unit.UnitValue,
        app.event.Success[unit.UnitValue],
        app.event.Failure[_TExpectedError]
    ]
):
    _EXPECTED_ERROR: ClassVar[_TExpectedError]

    _heading_handlers: List[Callable[[unit.UnitValue], None]]

    def __init__(self) -> None:
        self._heading_handlers = []

    def add_handler(self, handler: Callable[[unit.UnitValue], None]) -> None:
        self._heading_handlers.append(handler)

    def process(
            self,
            event: app.event.Event[Type.SET_HEADING, unit.UnitValue]
    ) -> app.event.Success[unit.UnitValue] | app.event.Failure[_TExpectedError]:
        for handler in self._heading_handlers:
            try:
                handler(event.value)
            except self._EXPECTED_ERROR as err:
                return app.event.Failure(err)

        return app.event.Success(event.value)
