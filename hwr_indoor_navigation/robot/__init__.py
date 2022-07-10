from typing import Dict, Callable, Any

import command
from command import Command  # Expose event externally with top level import
from robot.forward_backward_motor import ForwardBackwardMotor
from robot.steering_motor import SteeringMotor


class Robot:
    _forward_backward_motor: ForwardBackwardMotor
    _steering_motor: SteeringMotor
    _command_handlers: Dict[command.Type, Callable[[Any], None]]

    def __init__(self) -> None:
        self._forward_backward_motor = ForwardBackwardMotor()
        self._steering_motor = SteeringMotor()

        self._command_handlers = {
            command.Type.SET_SPEED: self._forward_backward_motor.set_speed,
            command.Type.SET_HEADING: self._steering_motor.set_heading,
        }

    def handle_command(self, command_: Command) -> None:
        self._command_handlers[command_.type_](command_.value)
