from __future__ import annotations

import logging
from typing import Tuple, List, Optional

from .._types import (
    Type,
    Processor,
    Success,
    Failure,
    Event,
)
from interface import WithShutdown


class ShutdownRequestProcessor(
    Processor[
        Type.SHUTDOWN,
        Optional[Exception],
        Optional[Exception],
        List[Exception]
    ]
):
    _objs_with_shutdown: Tuple[WithShutdown]

    def __init__(self, *objs_with_shutdown: WithShutdown):
        self._objs_with_shutdown = objs_with_shutdown

    def process(
            self,
            event: Event[Type.SHUTDOWN, Optional[Exception]],
    ) -> Success[Optional[Exception]] | Failure[List[Exception]]:
        errs: List[Exception] = []
        for obj in self._objs_with_shutdown:
            try:
                obj.shutdown()
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
