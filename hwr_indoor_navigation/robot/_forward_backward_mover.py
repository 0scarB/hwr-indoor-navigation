from typing import Tuple

import unit
import event as global_event
from . import _component as component
from . import _event as event


class Drive:
    """Service class for forward and backward movement"""
    _motor: component.DriveMotor

    def __init__(self, motor: component.DriveMotor) -> None:
        self._motor = motor

    def use_startup_request_event_processor(self, processor: global_event.processor.StartupRequestProcessor) -> None:
        processor.startup_obj_on_request(self._motor)

    def use_shutdown_request_event_processor(self, processor: global_event.processor.ShutdownRequestProcessor) -> None:
        processor.shutdown_obj_on_request(self._motor)

    def use_set_speed_request_event_processor(self, processor: event._processor.SetSpeedRequestProcessor) -> None:
        processor.add_handler(self.handle_request_set_speed)

    def handle_request_set_speed(self, speed: unit.UnitValue) -> Tuple[bool, None]:
        self._motor.set_speed(speed)

        return True, None
