class EventFailure(Exception):
    pass


class SetHeadingFailure(EventFailure):
    pass


class SetHeadingExpectedFailure(SetHeadingFailure):
    pass


class SetHeadingUnexpectedFailure(SetHeadingFailure):
    pass


class SetMovementSpeedFailure(EventFailure):
    pass


class SetMovementSpeedExpectedFailure(SetMovementSpeedFailure):
    pass


class SetMovementSpeedUnexpectedFailure(SetMovementSpeedFailure):
    pass
