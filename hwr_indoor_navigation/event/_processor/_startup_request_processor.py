from __future__ import annotations

import logging
from typing import List, Optional

from event._type import (
    Type,
    Processor,
    Success,
    Failure,
    Event,
)
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

    def process(
            self,
            event: Event[Type.STARTUP, Optional[Exception]],
    ) -> Success[Optional[Exception]] | Failure[List[Exception]]:
        errs: List[Exception] = []
        for obj in self._objs_with_startup:
            try:
                obj.startup()
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
