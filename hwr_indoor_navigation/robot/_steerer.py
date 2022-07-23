from typing import Tuple

import unit
import event as global_event
from . import _component as component
from . import _event as event


class Steerer:
    """Service class for steering"""
    _motor: component.SteeringMotor

    def __init__(self, motor: component.SteeringMotor):
        self._motor = motor

    def use_startup_request_event_processor(self, processor: global_event.processor.StartupRequestProcessor) -> None:
        processor.startup_obj_on_request(self._motor)

    def use_shutdown_request_event_processor(self, processor: global_event.processor.ShutdownRequestProcessor) -> None:
        processor.shutdown_obj_on_request(self._motor)

    def use_set_heading_request_event_processor(self, processor: event._processor.SetHeadingRequestProcessor) -> None:
        processor.add_handler(self.handle_heading_set_speed)

    def handle_heading_set_speed(self, speed: unit.UnitValue) -> Tuple[bool, None]:
        self._motor.set_heading(speed)
        return True, None
