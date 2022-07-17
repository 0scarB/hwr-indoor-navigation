from __future__ import annotations

from ... import _error as error
from .._type import *
from ._unit_value_request_processor import UnitValueRequestProcessor


class SetHeadingRequestProcessor(
    UnitValueRequestProcessor[
        Type.SET_HEADING,
        error.WrongHeading,
        SetHeadingCorrection,
        error.WrongHeading
    ]
):
    _EXPECTED_ERROR = error.WrongHeading
    _FAILURE_VALUE_TYPE: SetHeadingFailureValue
    _CORRECTION_TYPE: SetHeadingCorrection
