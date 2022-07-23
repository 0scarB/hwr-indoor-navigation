import unit
from ._config import Config
from . import _event as event
from ._keyboard_listener import KeyboardListener


class ForwardBackwardKeysHandler:
    _config: Config
    _keyboard_listener: KeyboardListener

    def __init__(self, config: Config, keyboard_listener: KeyboardListener) -> None:
        self._config = config
        self._keyboard_listener = keyboard_listener

    def use_set_robot_speed_response_processor(
            self,
            processor: event.processor.SetRobotSpeedResponseProcessor,
    ) -> None:
        processor.add_success_handler(self._handle_set_robot_speed_success)
        processor.add_correction_handler(self._handle_set_robot_speed_correction)

    def use_set_robot_speed_publisher(self, publisher: event.publisher.SetRobotSpeedPublisher) -> None:
        def forward_backward_key_handler(key: str) -> None:
            if key == self._config.forward_key:
                publisher.set_speed(self._config.movement_speed)
                publisher.publish()
            elif key == self._config.backward_key:
                publisher.set_speed(-self._config.movement_speed)
                publisher.publish()

        self._keyboard_listener.add_key_change_handler(forward_backward_key_handler)

    def _handle_set_robot_speed_success(self, speed: unit.UnitValue) -> None:
        # TODO: Real implementation
        print(f"robot speed was set to {speed}")

    def _handle_set_robot_speed_correction(self, speed: unit.UnitValue) -> None:
        # TODO: Real implementation
        print(f"robot speed was set corrected to {speed}")
