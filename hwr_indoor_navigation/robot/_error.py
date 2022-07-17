class EventFailure(Exception):
    pass


class WrongHeading(EventFailure):
    pass


class WrongSpeed(EventFailure):
    pass
