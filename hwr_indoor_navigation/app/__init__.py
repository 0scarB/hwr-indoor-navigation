import event
import terminal_controls
from ._config import Config
from . import _event as local_event
from . import _error as error
import robot


class App:
    _config: Config

    def __init__(self, config: Config) -> None:
        self._config = config

    def start(self) -> None:
        event_broker = event.Broker(self._config.event)

        startup_publisher = local_event.publisher.StartupPublisher()
        shutdown_publisher = local_event.publisher.ShutdownPublisher()

        self._use_services(event_broker)

        event_broker.add_publisher(
            event.Topic.REQUEST,
            startup_publisher
        )
        event_broker.add_publisher(
            event.Topic.REQUEST,
            shutdown_publisher
        )
        event_broker.add_processor(
            event.Topic.RESPONSES,
            event.Type.SHUTDOWN,
            local_event.processor.ShutdownResponsesProcessor()
        )

        startup_publisher.publish()

    def _use_services(self, broker: event.Broker) -> None:
        for service_name, service_config in self._config.services:
            if service_name == "robot":
                robot.Robot(service_config).use_event_broker(broker)
            elif service_name == "terminal_controls":
                terminal_controls.TerminalControls(
                    service_config
                ).use_event_broker(broker)
            else:
                raise error.UnexpectedServiceError(service_name)
