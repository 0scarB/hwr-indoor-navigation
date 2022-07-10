from __future__ import annotations

from typing import Any, cast, Generic, TypeVar

from hwr_indoor_navigation import error


T = TypeVar("T")
N = TypeVar("N", bound=float)


class NumberBetweenBounds(Generic[N]):
    """
    An attribute that will raise an error if set to value outside its bounds.

    Value must lie within the inclusive interval between lower bound and upper bound.
    """
    _value: N
    _lower_bound: float | int
    _upper_bound: float | int
    _below_lower_bound_err: errors.ValueOutOfBounds
    _above_upper_bound_err: errors.ValueOutOfBounds

    def __init__(
            self,
            initial_value: N,
            *,
            lower_bound: float | int,
            upper_bound: float | int,
            below_lower_bound_error: errors.ValueOutOfBounds | None = None,
            above_upper_bound_error: errors.ValueOutOfBounds | None = None,
    ):
        # Set to lower bound first so initial value can be checked
        self._value = cast(N, lower_bound)
        self._lower_bound = lower_bound
        self._upper_bound = upper_bound
        self._below_lower_bound_err = \
            errors.ValueOutOfBounds(
                f"Cannot set bounded attribute to value below lower bound {lower_bound}"
            ) if below_lower_bound_error is None \
            else below_lower_bound_error
        self._above_upper_bound_err = \
            errors.ValueOutOfBounds(
                f"Cannot set bounded attribute to value above upper bound {upper_bound}"
            ) if above_upper_bound_error is None \
            else above_upper_bound_error

        # Set to initial value with checks
        self.__set__(None, initial_value)

    def __get__(self, instance: Any, cls: Any) -> N:
        return self._value

    def __set__(self, instance: Any, value: N) -> None:
        if self._lower_bound <= value <= self._upper_bound:
            self._value = value
            return

        if value < self._lower_bound:
            raise self._below_lower_bound_err

        if value > self._upper_bound:
            raise self._above_upper_bound_err
