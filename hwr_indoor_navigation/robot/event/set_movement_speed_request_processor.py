from __future__ import annotations

from typing import Tuple

from event import RequestMetadata
from . import (
    RequestProcessor,
    EventType,
    SetMovementSpeedRequestValue,
    SetMovementSpeedSuccessValue,
    SetMovementSpeedFailureValue,
)
from .. import ForwardBackwardMotor


class SetMovementSpeedRequestProcessor(
    RequestProcessor[
        SetMovementSpeedRequestValue,
        EventType.SET_MOVEMENT_SPEED_SUCCESS,
        SetMovementSpeedSuccessValue,
        EventType.SET_MOVEMENT_SPEED_FAILURE,
        SetMovementSpeedFailureValue,
    ]
):
    _motor: ForwardBackwardMotor

    def __init__(self, motor: ForwardBackwardMotor) -> None:
        self._motor = motor

    def process(
            self,
            value: SetMovementSpeedRequestValue,
            metadata: RequestMetadata
    ) -> Tuple[
        EventType.SET_MOVEMENT_SPEED_SUCCESS,
        SetMovementSpeedSuccessValue
    ] | Tuple[
        EventType.SET_MOVEMENT_SPEED_FAILURE,
        SetMovementSpeedFailureValue
    ]:
        try:
            self._motor.set_speed(value)
        except SetMovementSpeedFailureValue as failure_value:
            return EventType.SET_MOVEMENT_SPEED_FAILURE, failure_value

        return EventType.SET_MOVEMENT_SPEED_SUCCESS, value
