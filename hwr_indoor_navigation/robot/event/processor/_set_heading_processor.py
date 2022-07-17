from __future__ import annotations

from typing import Tuple

from event import RequestMetadata
from robot.event import (
    RequestProcessor,
    Type,
    SetHeadingRequestValue,
    SetHeadingSuccessValue,
    SetHeadingFailureValue,
)
from robot import SteeringMotor


class SetHeadingProcessor(
    RequestProcessor[
        SetHeadingRequestValue,
        Type.SET_HEADING_SUCCESS,
        SetHeadingSuccessValue,
        Type.SET_HEADING_FAILURE,
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
                            Type.SET_HEADING_SUCCESS,
                            SetHeadingSuccessValue
    ] | Tuple[
                            Type.SET_HEADING_FAILURE,
                            SetHeadingFailureValue
    ]:
        try:
            self._motor.set_heading(value)
        except SetHeadingFailureValue as failure_value:
            return Type.SET_HEADING_SUCCESS, failure_value

        return Type.SET_HEADING_FAILURE, value
