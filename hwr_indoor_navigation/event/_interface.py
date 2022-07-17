from __future__ import annotations

from typing import Protocol, Tuple

from ._type import *
from ._broker import Broker


class Publisher(Protocol[T, V]):

    def publish(self) -> List[Tuple[T, V] | Tuple[T, V, LogLevel]]:
        ...


class Processor(Protocol[T, V, VSuccess, VFailure]):

    def process(
            self,
            event: Event[T, V],
    ) -> Success[VSuccess] | Failure[VFailure] | None:
        ...


class Service(Protocol):

    def use_event_broker(self, broker: Broker):
        ...
