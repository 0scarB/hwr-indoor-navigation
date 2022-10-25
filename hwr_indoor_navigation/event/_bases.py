from __future__ import annotations

from typing import overload

from bases import Service as GlobalServiceBase
from ._interface import Publisher, Processor, Service as ServiceInterface
from ._type import *


class ExposedPublisher(Publisher[T, V]):
    """Publisher with the publish method directly exposed."""

    def __init__(self) -> None:
        self._publish = self.publish

    @overload
    def publish(self, value: V) -> list[tuple[T, V]]:
        ...

    @overload
    def publish(self, value: V) -> list[tuple[T, V, LogLevel]]:
        ...

    def publish(self, value: V) -> list[tuple[T, V] | tuple[T, V, LogLevel]]:
        ...


class ExposedProcessor(Processor[T, V, VSuccess, VFailure]):
    """Processor with the process method directly exposed."""

    def __init__(self) -> None:
        self._process = self.process

    @overload
    def process(self, event: Event[T, V]) -> None:  # acts as subscriber when none is returned
        ...

    @overload
    def process(self, event: Event[T, V]) -> Success[VSuccess]:
        ...

    @overload
    def process(self, event: Event[T, V]) -> Failure[VFailure]:
        ...

    @overload
    def process(self, event: Event[T, V]) -> Success[VSuccess] | None:
        ...

    @overload
    def process(self, event: Event[T, V]) -> Failure[VFailure] | None:
        ...

    @overload
    def process(self, event: Event[T, V]) -> Success[VSuccess] | Failure[VFailure]:
        ...

    def process(
            self,
            event: Event[T, V],
    ) -> Success[VSuccess] | Failure[VFailure] | None:
        ...

    def _process(
            self,
            event: Event[T, V],
    ) -> Success[VSuccess] | Failure[VFailure] | None:
        ...


_Config = TypeVar("_Config")


class Service(ServiceInterface, GlobalServiceBase[_Config]):
    pass
