import event as global_event
from . import _event as event
from ._config import Config
from ._keyboard_listener import KeyboardListener


class TerminalControls(global_event.Service):
    """Service class for terminal controls."""
    _config: Config

    def __init__(self, config: Config) -> None:
        self._config = config

    def use_event_broker(self, broker: event.Broker) -> None:
        keyboard_listener = KeyboardListener()

        startup_request_event_processor = global_event.processor.StartupRequestProcessor()
        set_robot_heading_response_event_processor = event.processor.SetRobotHeadingResponseProcessor()
        set_robot_speed_response_event_processor = event.processor.SetRobotSpeedResponseProcessor()
        shutdown_request_event_processor = global_event.processor.ShutdownRequestProcessor()

        set_robot_heading =