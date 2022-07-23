import event as global_event
import robot
from . import _event as event
from ._config import Config
from ._keyboard_listener import KeyboardListener
from ._forward_backward_keys_handler import ForwardBackwardKeysHandler
from ._left_right_keys_handler import LeftRightKeysHandler


class TerminalControls(global_event.Service):
    """Service class for terminal controls."""
    _config: Config

    def __init__(self, config: Config) -> None:
        self._config = config

    def use_event_broker(self, broker: global_event.Broker) -> None:
        keyboard_listener = KeyboardListener()

        # create processors
        startup_request_event_processor = global_event.processor.StartupRequestProcessor()
        shutdown_request_event_processor = global_event.processor.ShutdownRequestProcessor()
        set_robot_heading_response_event_processor = event.processor.SetRobotHeadingResponseProcessor()
        set_robot_speed_response_event_processor = event.processor.SetRobotSpeedResponseProcessor()

        # handle startup and shutdown
        startup_request_event_processor.startup_obj_on_request(keyboard_listener)
        shutdown_request_event_processor.shutdown_obj_on_request(keyboard_listener)

        # create publishers
        set_robot_heading_publisher = event.publisher.SetRobotHeadingPublisher()
        set_robot_speed_publisher = event.publisher.SetRobotSpeedPublisher()

        # create services
        forward_backward_keys_handler = ForwardBackwardKeysHandler(self._config, keyboard_listener)
        left_right_keys_handler = LeftRightKeysHandler(self._config, keyboard_listener)

        # configure services
        forward_backward_keys_handler.use_set_robot_speed_publisher(set_robot_speed_publisher)
        forward_backward_keys_handler.use_set_robot_speed_response_processor(set_robot_speed_response_event_processor)
        left_right_keys_handler.use_set_robot_heading_publisher(set_robot_heading_publisher)
        left_right_keys_handler.use_set_robot_heading_response_processor(set_robot_heading_response_event_processor)

        # use event broker for publishing/processing
        broker.add_processor(
            global_event.Topic.REQUEST,
            global_event.Type.STARTUP,
            startup_request_event_processor,
        )
        broker.add_processor(
            global_event.Topic.REQUEST,
            global_event.Type.SHUTDOWN,
            shutdown_request_event_processor,
        )
        broker.add_processor(
            global_event.Topic.RESPONSES,
            robot.event.Type.SET_HEADING,
            set_robot_heading_response_event_processor,
        )
        broker.add_processor(
            global_event.Topic.RESPONSES,
            robot.event.Type.SET_SPEED,
            set_robot_speed_response_event_processor,
        )
        broker.add_publisher(
            global_event.Topic.REQUEST,
            set_robot_heading_publisher,
        )
        broker.add_publisher(
            global_event.Topic.REQUEST,
            set_robot_speed_publisher,
        )
