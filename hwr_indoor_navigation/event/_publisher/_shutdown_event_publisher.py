from __future__ import annotations

from typing import Optional, List, Tuple

from .._interface import Publisher
from .._type import Type


class StartupEventPublisher(
    Publisher[
        Type.STARTUP,
        Optional[Exception]
    ]
):
    _error_or_none: Exception | None

    def __init__(self) -> None:
        self._error_or_none = None

    def set_error(self, err: Exception) -> None:
        self._error_or_none = err

    def clear_error(self) -> None:
        self._error_or_none = None

    def publish_on_success(self) -> None:
        self.clear_error()
        self.publish()

    def publish_on_error(self, err: Exception) -> None:
        self.set_error(err)
        self.publish()

    def publish(
            self
    ) -> List[Tuple[Type.SHUTDOWN, Exception | None]]:
        return [(Type.SHUTDOWN, self._error_or_none)]
