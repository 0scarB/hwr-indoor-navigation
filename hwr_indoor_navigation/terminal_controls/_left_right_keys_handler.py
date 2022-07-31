import unit
from ._config import Config
from . import _event as event
from ._keyboard_listener import KeyboardListener


class LeftRightKeysHandler:
    _config: Config
    _keyboard_listener: KeyboardListener
    _heading: unit.UnitValue

    def __init__(self, config: Config, keyboard_listener: KeyboardListener) -> None:
        self._config = config
        self._keyboard_listener = keyboard_listener
        self._heading = unit.UnitValue(300, "_steering_motor_pwm")

    def use_set_robot_heading_response_processor(
            self,
            processor: event.processor.SetRobotHeadingResponseProcessor,
    ) -> None:
        processor.add_success_handler(self._handle_set_robot_heading_success_processor)
        processor.add_correction_handler(self._handle_set_robot_heading_success_correction)

    def use_set_robot_heading_publisher(self, publisher: event.publisher.SetRobotHeadingPublisher) -> None:
        def left_right_key_handler(key: str) -> None:
            if key == self._config.left_key:
                # Increment heading
                publisher.set_heading(self._heading + self._config.steering_increment)
                publisher.publish()
            elif key == self._config.right_key:
                # Decrement heading
                publisher.set_heading(self._heading - self._config.steering_increment)
                publisher.publish()

        self._keyboard_listener.add_key_change_handler(left_right_key_handler)

    def _handle_set_robot_heading_success_processor(self, heading: unit.UnitValue) -> None:
        self._heading = heading

    def _handle_set_robot_heading_success_correction(self, heading: unit.UnitValue) -> None:
        self._heading = heading
