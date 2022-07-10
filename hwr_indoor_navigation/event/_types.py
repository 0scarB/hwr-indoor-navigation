from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum
from typing import Protocol, TypeVar, Tuple, List, Generic


T = TypeVar("T")
V = TypeVar("V")
VSuccess = TypeVar("VSuccess")
VFailure = TypeVar("VFailure")


class Topic(str, Enum):
    REQUEST = "request"
    RESPONSES = "responses"


class Type(str, Enum):
    STARTUP = "startup"
    SHUTDOWN = "shutdown"


LogLevel = logging.CRITICAL | logging.ERROR | logging.WARNING | logging.INFO | logging.DEBUG


@dataclass(frozen=True)
class Event(Generic[T, V]):
    topic: Topic
    type_: T
    id_: int
    timestamp: int
    value: V
    log_level: LogLevel


Request = Event


@dataclass(frozen=True)
class Response(Generic[V]):
    value: V | None = None
    log_level: LogLevel = logging.INFO


@dataclass(frozen=True)
class Success(Response[V]):
    pass


@dataclass(frozen=True)
class Failure(Response[V]):
    pass


@dataclass(frozen=True)
class ResponsesValue(Generic[VSuccess, VFailure]):
    successes: List[Success[VSuccess]]
    failures: List[Failure[VFailure]]


class Responses(
    Generic[T, VSuccess, VFailure],
    Event[T, ResponsesValue[VSuccess, VFailure]]
):
    pass


class Publisher(Protocol[T, V]):

    def publish(self) -> List[Tuple[T, V] | Tuple[T, V, LogLevel]]:
        ...


class Processor(Generic[T, V, VSuccess, VFailure]):

    def process(
            self,
            event: Event[T, V],
    ) -> Success[VSuccess] | Failure[VFailure] | None:
        ...
