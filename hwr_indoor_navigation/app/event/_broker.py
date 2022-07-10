import logging
import time
from collections import deque
from typing import Deque, Any, Dict, List

from ._config import Config
from . import _error as error
from ._types import (
    Event,
    Publisher,
    Topic,
    Type,
    Processor,
    Success,
    Failure,
    LogLevel,
    ResponsesValue,
)


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

        def new_publish_method() -> None:
            items = original_meth_func(publisher)

            for item in items:
                if len(item) == 2:
                    type_, value = item
                    log_level = logging.INFO
                else:
                    type_, value, log_level = item

                self._generate_and_publish(topic, type_, value, log_level)

            self._process_queue()

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

    def _process(self, event: Event) -> None:
        try:
            processors = self._processors[event.topic][event.type_]
        except KeyError:
            return

        successes: List[Success[Any]] = []
        failures: List[Failure[Any]] = []
        for processor in processors:
            result = processor.process(event)
            if isinstance(result, Success):
                successes.append(result)
            elif isinstance(result, Failure):
                failures.append(result)

        self._generate_and_publish(
            Topic.RESPONSES,
            event.type_,
            ResponsesValue(successes, failures),
            logging.INFO
        )

    def _process_queue(self) -> None:
        while self._event_queue:
            self._process(self._event_queue.popleft())

    def _generate_and_publish(
            self,
            topic: Topic,
            type_: Type,
            value: Any,
            log_level: LogLevel
    ) -> None:
        event = self._generate_event(topic, type_, value, log_level)

        self._publish(event)

    def _generate_event(
            self,
            topic: Topic,
            type_: Type,
            value: Any,
            log_level: LogLevel
    ) -> Event:
        return Event(
            topic=topic,
            type_=type_,
            id_=self._generate_event_id(),
            timestamp=time.time_ns(),
            value=value,
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
