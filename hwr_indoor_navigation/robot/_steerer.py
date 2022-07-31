from typing import Tuple, Callable, List

import unit
import event as global_event
from . import _component as component
from . import _event as event


class Steerer:
    """Service class for steering"""
    _motor: component.SteeringMotor
    _on_change_heading_callbacks: List[Callable[[unit.UnitValue], None]]

    def __init__(self, motor: component.SteeringMotor):
        self._motor = motor
        self._on_change_heading_callbacks = []

    def use_startup_request_event_processor(self, processor: global_event.processor.StartupRequestProcessor) -> None:
        processor.startup_obj_on_request(self._motor)

    def use_shutdown_request_event_processor(self, processor: global_event.processor.ShutdownRequestProcessor) -> None:
        processor.shutdown_obj_on_request(self._motor)

    def use_set_heading_request_event_processor(self, processor: event._processor.SetHeadingRequestProcessor) -> None:
        processor.add_handler(self.handle_heading_set_speed)

    def on_change_heading(self, callback: Callable[[unit.UnitValue], None]) -> None:
        self._on_change_heading_callbacks.append(callback)

    def handle_heading_set_speed(self, heading: unit.UnitValue) -> Tuple[bool, None]:
        self._motor.set_heading(heading)

        for callback in self._on_change_heading_callbacks:
            callback(heading)

        return True, None
