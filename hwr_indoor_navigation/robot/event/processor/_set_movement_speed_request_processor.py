from __future__ import annotations

from typing import Tuple

from event import RequestMetadata
from robot.event import (
    RequestProcessor,
    Type,
    SetMovementSpeedRequestValue,
    SetMovementSpeedSuccessValue,
    SetMovementSpeedFailureValue,
)
from robot import ForwardBackwardMotor


class SetMovementSpeedProcessor(
    RequestProcessor[
        SetMovementSpeedRequestValue,
        Type.SET_MOVEMENT_SPEED_SUCCESS,
        SetMovementSpeedSuccessValue,
        Type.SET_MOVEMENT_SPEED_FAILURE,
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
                            Type.SET_MOVEMENT_SPEED_SUCCESS,
                            SetMovementSpeedSuccessValue
    ] | Tuple[
                            Type.SET_MOVEMENT_SPEED_FAILURE,
                            SetMovementSpeedFailureValue
    ]:
        try:
            self._motor.set_speed(value)
        except SetMovementSpeedFailureValue as failure_value:
            return Type.SET_MOVEMENT_SPEED_FAILURE, failure_value

        return Type.SET_MOVEMENT_SPEED_SUCCESS, value
