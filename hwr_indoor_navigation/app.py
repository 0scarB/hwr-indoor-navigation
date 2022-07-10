import time

import event
from config import Config


class App:
    _event_broker: event.Broker
    _shutdown_publisher: event.publisher.ShutdownPublisher

    def __init__(self, config: Config) -> None:
        self._event_broker = event.Broker(config.event)
        self._shutdown_publisher = event.publisher.ShutdownPublisher()

    def run(self) -> None:
        self._startup()
        self._shutdown()

    def _startup(self) -> None:
        startup_publisher = event.publisher.StartupPublisher()

        self._event_broker.add_publisher(
            event.Topic.REQUEST,
            startup_publisher
        )
        self._event_broker.add_publisher(
            event.Topic.REQUEST,
            self._shutdown_publisher
        )

        self._event_broker.add_processor(
            event.Topic.REQUEST,
            event.Type.SHUTDOWN,
            event.processor.ShutdownRequestProcessor()
        )
        self._event_broker.add_processor(
            event.Topic.RESPONSES,
            event.Type.SHUTDOWN,
            event.processor.ShutdownResponsesProcessor()
        )

        startup_publisher.publish()

    def _shutdown(self) -> None:
        self._shutdown_publisher.publish()
