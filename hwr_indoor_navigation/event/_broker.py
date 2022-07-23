from __future__ import annotations

import asyncio
import logging
import time
from collections import deque
from collections.abc import Awaitable
from typing import Deque, Any, Dict, List

from ._config import Config
from . import _error as error
from ._type import (
    Event,
    Topic,
    Type,
    Success,
    Failure,
    LogLevel,
    ResponsesValue,
)
from ._interface import Publisher, Processor


class Broker:
    _event_queue: Deque[Event]
    _processors = Dict[Topic, Dict[Type, List[Processor]]]
    _current_event_id: int
    _logger: logging.Logger

    def __init__(self, config: Config):
        self._event_queue = deque()
        self._processors = {}
        self._current_event_id = 0

        self._initialize_logger(config.log_level)

    def add_publisher(self, topic: Topic, publisher: Publisher) -> None:
        if topic != Topic.REQUEST:
            raise error.WrongTopic(
                "Currently publishers are only allowed to publish on the request topic. "
                "Only remove this constraint for good reason."
            )

        original_meth_func = publisher.publish.__func__

        publisher_class = type(publisher)
        publisher_class_str = f"{publisher_class.__module__}.{publisher_class.__name__}"

        def new_publish_method() -> None:
            items = original_meth_func(publisher)

            for item in items:
                if len(item) == 2:
                    type_, value = item
                    log_level = logging.INFO
                else:
                    type_, value, log_level = item

                self._generate_and_publish(
                    topic,
                    type_,
                    value,
                    publisher_class_str,
                    None,
                    log_level
                )

            async_loop = asyncio.get_event_loop()
            async_loop.create_task(self._process_queue())

            return items

        publisher.publish = new_publish_method

    def add_processor(self, topic: Topic, event_type: Type, processor: Processor) -> None:
        if topic not in self._processors:
            self._processors[topic] = {event_type: [processor]}
            return

        processors_for_topic = self._processors[topic]
        if event_type in processors_for_topic:
            processors_for_topic[event_type].append(processor)
            return

        processors_for_topic[event_type] = [processor]

    def _publish(self, event: Event):
        self._event_queue.append(event)

        self._logger.log(event.log_level, str(event))

    async def _process(self, event: Event) -> None:
        try:
            processors = self._processors[event.topic][event.type_]
        except KeyError:
            return

        successes: List[Success[Any]] = []
        failures: List[Failure[Any]] = []
        processor_qualnames: list[str] = []
        for processor in processors:
            result = processor.process(event)

            if isinstance(result, Awaitable):
                result = await result

            if isinstance(result, Success):
                successes.append(result)
            elif isinstance(result, Failure):
                failures.append(result)

            processor_class = type(processor)
            processor_qualnames.append(f"{processor_class.__module__}.{processor_class.__qualname__}")

        # We publish a response event if no processors returned a result.
        if len(successes) == 0 and len(failures) == 0:
            return

        self._generate_and_publish(
            Topic.RESPONSES,
            event.type_,
            ResponsesValue(successes, failures),
            None,
            processor_qualnames,
            logging.INFO
        )

    async def _process_queue(self) -> None:
        while self._event_queue:
            try:
                await self._process(self._event_queue.popleft())
            except SystemExit as exit_:
                if exit_.code == 0:
                    self._handle_successful_system_exit()
                    return

                raise exit_

    def _handle_successful_system_exit(self) -> None:
        # Stop currently running event loop if running in event loop
        try:
            async_loop = asyncio.get_running_loop()
            async_loop.stop()
        except RuntimeError:
            pass

    def _generate_and_publish(
            self,
            topic: Topic,
            type_: Type,
            value: Any,
            published_by: str | None,
            returned_as_processing_response_by: list[str] | None,
            log_level: LogLevel
    ) -> None:
        event = self._generate_event(
            topic,
            type_,
            value,
            published_by,
            returned_as_processing_response_by,
            log_level
        )

        self._publish(event)

    def _generate_event(
            self,
            topic: Topic,
            type_: Type,
            value: Any,
            published_by: str | None,
            returned_as_processing_response_by: list[str] | None,
            log_level: LogLevel
    ) -> Event:
        return Event(
            topic=topic,
            type_=type_,
            id_=self._generate_event_id(),
            timestamp=time.time_ns(),
            value=value,
            published_by=published_by,
            returned_as_processing_response_by=returned_as_processing_response_by,
            log_level=log_level
        )

    def _generate_event_id(self) -> int:
        id_ = self._current_event_id

        self._current_event_id += 1

        return id_

    def _initialize_logger(self, log_level: LogLevel) -> None:
        self._logger = logging.getLogger("EVENT")
        self._logger.setLevel(log_level)

        console_handler = logging.StreamHandler()
        console_handler.setLevel(log_level)

        formatter = logging.Formatter(
            '[%(asctime)s|%(name)s|%(levelname)s] %(message)s'
        )
        console_handler.setFormatter(formatter)

        self._logger.addHandler(console_handler)
