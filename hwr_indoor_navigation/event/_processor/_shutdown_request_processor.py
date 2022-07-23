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
from interface import WithShutdown


class ShutdownRequestProcessor(
    Processor[
        Type.SHUTDOWN,
        Optional[Exception],
        Optional[Exception],
        List[Exception]
    ]
):
    _objs_with_shutdown: List[WithShutdown]

    def __init__(self) -> None:
        self._objs_with_shutdown = []

    def shutdown_obj_on_request(self, obj: WithShutdown) -> None:
        self._objs_with_shutdown.append(obj)

    async def process(
            self,
            event: Event[Type.SHUTDOWN, Optional[Exception]],
    ) -> Success[Optional[Exception]] | Failure[List[Exception]]:
        errs: List[Exception] = []
        for obj in self._objs_with_shutdown:
            try:
                result = obj.shutdown()
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
