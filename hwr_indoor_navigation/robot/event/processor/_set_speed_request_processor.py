from __future__ import annotations

from ... import _error as error
from ._unit_value_request_processor import UnitValueRequestProcessor


class SetHeadingProcessor(UnitValueRequestProcessor[error.SetHeadingFailure]):
    _EXPECTED_ERROR = error.SetSpeedFailure
