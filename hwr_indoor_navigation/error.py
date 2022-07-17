class ValueOutOfBounds(Exception):
    pass


class UnitsConversionError(Exception):
    pass


class UnitConversionAlreadyAdded(UnitsConversionError):
    pass


class UnitConversionNotAdded(UnitsConversionError):
    pass


class UnitConversionInvalidChain(UnitsConversionError):
    pass


class UnitConversionNotAddedInChain(
    UnitConversionNotAdded,
    UnitConversionInvalidChain
):
    pass


class MutabilityDisallowed(Exception):
    pass


class ShutdownFailure(RuntimeError):
    pass


class ValueNotSet(Exception):
    pass
