import asyncio

import event as global_event
import terminal_controls
from ._config import Config
from . import _event as event
from . import _error as error
import robot


class App:
    _config: Config

    def __init__(self, config: Config) -> None:
        self._config = config

    def start(self) -> None:
        shutdown_global_event_publisher = event.publisher.ShutdownPublisher()

        async def async_start():
            self._run_on_async_loop_start(shutdown_global_event_publisher)

        async def async_shutdown():
            shutdown_global_event_publisher.publish()
        loop = asyncio.get_event_loop()
        try:
            asyncio.ensure_future(asyncio.shield(async_start()))
            loop.run_forever()
        except KeyboardInterrupt:
            asyncio.ensure_future(async_shutdown())
            loop.run_forever()

    def _run_on_async_loop_start(
            self,
            shutdown_global_event_publisher: event.publisher.ShutdownPublisher
    ) -> None:
        global_event_broker = global_event.Broker(self._config.event)

        startup_global_event_publisher = event.publisher.StartupPublisher()

        self._use_services(global_event_broker)

        global_event_broker.add_publisher(
            global_event.Topic.REQUEST,
            startup_global_event_publisher
        )
        global_event_broker.add_publisher(
            global_event.Topic.REQUEST,
            shutdown_global_event_publisher
        )
        global_event_broker.add_processor(
            global_event.Topic.RESPONSES,
            global_event.Type.SHUTDOWN,
            event.processor.ShutdownResponsesProcessor()
        )

        startup_global_event_publisher.publish()

    def _use_services(self, broker: global_event.Broker) -> None:
        for service_name, service_config in self._config.services:
            if service_name == "robot":
                robot.Robot(service_config).use_event_broker(broker)
            elif service_name == "terminal_controls":
                terminal_controls.TerminalControls(
                    service_config
                ).use_event_broker(broker)
            else:
                raise error.UnexpectedServiceError(service_name)
