from __future__ import annotations

import sys
from typing import List, Optional

import error
from event import Type, Processor, ResponsesValue, Event


class ShutdownResponsesProcessor(
    Processor[
        Type.SHUTDOWN,
        ResponsesValue[Optional[Exception], List[Exception]],
        None,
        None
    ]
):

    def process(
            self,
            event: Event[Type.SHUTDOWN, ResponsesValue[Optional[Exception], List[Exception]]],
    ) -> None:
        if len(event.value.failures) > 0:
            errs_str = ", ".join(f"'{err}'" for err in event.value.failures)

            raise error.ShutdownFailure(
                f"Failed to do cleanup before shutdown with errors: {errs_str}"
            )

        errs = [success.value for success in event.value.successes if success.value is not None]
        if len(errs) > 0:
            errs_str = ", ".join(f"'{err}'" for err in errs)
            sys.exit(
                f"System shut down successfully because of errors: {errs_str}"
            )

        raise SystemExit(0)
