from typing import Tuple, Callable, List

import unit
import event as global_event
from . import _component as component
from . import _event as event


class ForwardBackwardMover:
    """Service class for forward and backward movement"""
    _motor: component.ForwardBackwardMotor
    _on_change_speed_callbacks: List[Callable[[unit.UnitValue], None]]

    def __init__(self, motor: component.ForwardBackwardMotor) -> None:
        self._motor = motor
        self._on_change_speed_callbacks = []

    def use_startup_request_event_processor(self, processor: global_event.processor.StartupRequestProcessor) -> None:
        processor.startup_obj_on_request(self._motor)

    def use_shutdown_request_event_processor(self, processor: global_event.processor.ShutdownRequestProcessor) -> None:
        processor.shutdown_obj_on_request(self._motor)

    def use_set_speed_request_event_processor(self, processor: event._processor.SetSpeedRequestProcessor) -> None:
        processor.add_handler(self.handle_request_set_speed)

    def on_change_speed(self, callback: Callable[[unit.UnitValue], None]) -> None:
        self._on_change_speed_callbacks.append(callback)

    def handle_request_set_speed(self, speed: unit.UnitValue) -> Tuple[bool, None]:
        self._motor.set_speed(speed)

        for callback in self._on_change_speed_callbacks:
            callback(speed)

        return True, None
