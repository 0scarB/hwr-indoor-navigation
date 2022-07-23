from __future__ import annotations

import logging
from collections.abc import Awaitable
from typing import List, Optional

from .._type import (
    Type,
    Success,
    Failure,
    Event,
)
from .._interface import Processor
from interface import WithStartup


class StartupRequestProcessor(
    Processor[
        Type.STARTUP,
        Optional[Exception],
        Optional[Exception],
        List[Exception]
    ]
):
    _objs_with_startup: List[WithStartup]

    def __init__(self) -> None:
        self._objs_with_startup = []

    def startup_obj_on_request(self, obj: WithStartup) -> None:
        self._objs_with_startup.append(obj)

    async def process(
            self,
            event: Event[Type.STARTUP, Optional[Exception]],
    ) -> Success[Optional[Exception]] | Failure[List[Exception]]:
        errs: List[Exception] = []
        for obj in self._objs_with_startup:
            try:
                result = obj.startup()
                if isinstance(result, Awaitable):
                    await result
            except Exception as failure_value:
                # We don't fail fast because as many objects should
                # have the chance to successfully shut down as possible.
                errs.append(failure_value)

        if len(errs) == 0:
            return Success(
                value=event.value,
                log_level=event.log_level
            )

        return Failure(errs, log_level=logging.CRITICAL)
