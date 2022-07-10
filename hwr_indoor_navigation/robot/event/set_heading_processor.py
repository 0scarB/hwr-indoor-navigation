from __future__ import annotations

from typing import Tuple

from event import RequestMetadata
from . import (
    RequestProcessor,
    EventType,
    SetHeadingRequestValue,
    SetHeadingSuccessValue,
    SetHeadingFailureValue,
)
from .. import SteeringMotor


class SetMovementSpeedRequestProcessor(
    RequestProcessor[
        SetHeadingRequestValue,
        EventType.SET_HEADING_SUCCESS,
        SetHeadingSuccessValue,
        EventType.SET_HEADING_FAILURE,
        SetHeadingFailureValue,
    ]
):
    _motor: SteeringMotor

    def __init__(self, motor: SteeringMotor) -> None:
        self._motor = motor

    def process(
            self,
            value: SetHeadingRequestValue,
            metadata: RequestMetadata
    ) -> Tuple[
        EventType.SET_HEADING_SUCCESS,
        SetHeadingSuccessValue
    ] | Tuple[
        EventType.SET_HEADING_FAILURE,
        SetHeadingFailureValue
    ]:
        try:
            self._motor.set_heading(value)
        except SetHeadingFailureValue as failure_value:
            return EventType.SET_HEADING_SUCCESS, failure_value

        return EventType.SET_HEADING_FAILURE, value
