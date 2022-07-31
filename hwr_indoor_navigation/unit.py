from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Dict, Tuple, Set, List

import error


_TConversionCallback = Callable[[float], float]


class Converter:
    _origin_unit_to_target_units = Dict[str, Set[str]]
    _origin_unit_to_target_unit_conversions: Dict[Tuple[str, str], _TConversionCallback]
    _origin_unit_to_target_unit_conversions_chains: Dict[Tuple[str, str], List[str]]

    @staticmethod
    def conversion_chain_to_string(chain: List[str], sep=" -> ") -> str:
        if len(chain) == 0:
            return ""

        return sep.join(chain)

    def __init__(self):
        self._origin_unit_to_target_units = {}
        self._origin_unit_to_target_unit_conversions = {}
        self._origin_unit_to_target_unit_conversions_chains = {}

        self._add_global_conversions()

    def add_conversion(self, origin: str, target: str, callback: _TConversionCallback) -> None:
        already_added_targets = self._origin_unit_to_target_units.get(origin, set())

        if target in already_added_targets:
            raise errors.UnitConversionAlreadyAdded(f"from '{origin}' to '{target}'")

        self._origin_unit_to_target_units[origin] = already_added_targets | {target}
        self._origin_unit_to_target_unit_conversions[(origin, target)] = callback

    def add_linear_conversion(
            self,
            origin: str,
            target: str,
            target_is_x_times_origin: float
    ) -> None:
        def convert_origin_to_target(value: float) -> float:
            return target_is_x_times_origin * value

        def convert_target_to_origin(value: float) -> float:
            return value / target_is_x_times_origin

        self.add_conversion(origin, target, convert_origin_to_target)
        self.add_conversion(target, origin, convert_target_to_origin)

    def convert(self, unit_value: UnitValue, target: str) -> UnitValue:
        value = self.convert_float(unit_value.unit, target, unit_value.value)

        return UnitValue(unit=target, value=value, converter=self)

    def convert_float(self, origin: str, target: str, value: float) -> float:
        if origin == target:
            return value

        conversion_chain = self.find_conversion_chain(origin, target)

        self._validate_conversion_chain(origin, target, conversion_chain)

        for (origin, target) in zip(conversion_chain[:-1], conversion_chain[1:]):
            value = self._convert_float_directly(origin, target, value)

        return value

    def add_conversion_chain(self, origin: str, target: str, chain: List[str]) -> None:
        self._validate_conversion_chain(origin, target, chain)

        self._origin_unit_to_target_unit_conversions_chains[(origin, target)] = chain

    def find_conversion_chain(self, origin: str, target: str) -> List[str]:
        """
        Find the chain of conversion to convert to the target unit.

        Uses depth first search, hence it can be very inefficient for many sparsely connected units.
        """
        try:
            return self._origin_unit_to_target_unit_conversions_chains[(origin, target)]
        except KeyError:
            pass

        def find_conversion_chain_recursive(chain: List[str], current: str, target: str) -> List[str]:
            if len(chain) > 0:
                previous = chain[-1]
                if current not in self._origin_unit_to_target_units[previous]:
                    raise errors.UnitConversionNotAdded(
                        f"No conversion from '{previous}' to '{current}' added"
                    )

                is_loop = current in chain
                if current not in self._origin_unit_to_target_units[previous] or is_loop:
                    raise errors.UnitConversionInvalidChain(
                        f"Found looping chain '{self.conversion_chain_to_string(chain + [current])}'"
                    )

            if current == target:
                return chain + [current]

            for next_ in self._origin_unit_to_target_units[current]:
                try:
                    return find_conversion_chain_recursive(
                        chain + [current],
                        next_,
                        target
                    )
                except (errors.UnitConversionNotAdded, errors.UnitConversionInvalidChain) as err:
                    pass

            raise errors.UnitConversionInvalidChain(
                f"No paths could be found to get from '{current}' to '{target}'"
            )

        try:
            chain = find_conversion_chain_recursive([], origin, target)
        except (errors.UnitConversionNotAdded, errors.UnitConversionInvalidChain) as err:
            raise errors.UnitConversionNotAdded(
                f"No conversion from '{origin}' to '{target}' added"
            ) from err

        self.add_conversion_chain(origin, target, chain)

        return chain

    def _validate_conversion_chain(self, origin: str, target: str, chain: List[str]) -> None:
        if len(chain) == 0:
            raise errors.UnitConversionInvalidChain("Cannot add empty conversion chain")

        if chain[0] != origin:
            raise errors.UnitConversionInvalidChain(
                f"Conversion chain from '{origin}' to '{target}' must start with '{origin}', "
                f"found '{self.conversion_chain_to_string(chain)}'"
            )

        if chain[-1] != target:
            raise errors.UnitConversionInvalidChain(
                f"Conversion chain from '{origin}' to '{target}' must end with '{target}', "
                f"found '{self.conversion_chain_to_string(chain)}'"
            )

        for first_unit, second_unit in zip(chain[:-1], chain[1:]):
            if (first_unit, second_unit) not in self._origin_unit_to_target_unit_conversions:
                raise errors.UnitConversionNotAddedInChain(
                    f"No conversion added for subsequent units in chain '{first_unit}' and '{second_unit}' "
                    f"for chain '{self.conversion_chain_to_string(chain)}'"
                )

    def _convert_float_directly(self, origin: str, target: str, value: float) -> float:
        try:
            convert = self._origin_unit_to_target_unit_conversions[(origin, target)]
        except KeyError as err:
            raise errors.UnitConversionNotAdded(f"No conversion from '{origin}' to '{target}' added") from err

        return convert(value)

    def _add_global_conversions(self) -> None:
        self._add_degrees_to_radians_conversion()

    def _add_degrees_to_radians_conversion(self) -> None:
        self.add_linear_conversion(
            "degrees",
            "radians",
            math.pi / 180
        )


global_converter = Converter()


@dataclass
class UnitValue:
    value: float
    unit: str
    converter: Converter = global_converter

    def to(self, unit: str) -> UnitValue:
        return self.converter.convert(self, unit)

    def __neg__(self) -> UnitValue:
        return UnitValue(
            -self.value,
            unit=self.unit,
            converter=self.converter,
        )

    def __str__(self) -> str:
        return f"{self.value}{self.unit}"

    def __lt__(self, other: Any) -> bool:
        return self.value < self._get_others_value_in_our_unit(other)

    def __le__(self, other: Any) -> bool:
        return self.value <= self._get_others_value_in_our_unit(other)

    def __eq__(self, other: Any) -> bool:
        return self.value == self._get_others_value_in_our_unit(other)

    def __ne__(self, other: Any) -> bool:
        return self.value != self._get_others_value_in_our_unit(other)

    def __gt__(self, other: Any) -> bool:
        return self.value > self._get_others_value_in_our_unit(other)

    def __ge__(self, other: Any) -> bool:
        return self.value >= self._get_others_value_in_our_unit(other)

    def __add__(self, other: Any) -> UnitValue:
        return self._create_arithmetic_operation_result(
            self.value + self._get_others_value_in_our_unit(other)
        )

    __radd__ = __add__

    def __sub__(self, other: Any) -> UnitValue:
        return self._create_arithmetic_operation_result(
            self.value - self._get_others_value_in_our_unit(other)
        )

    def __rsub__(self, other: Any) -> UnitValue:
        return self._create_arithmetic_operation_result(
            self._get_others_value_in_our_unit(other) - self.value
        )

    def __mul__(self, other: Any) -> UnitValue:
        return self._create_arithmetic_operation_result(
            self.value * self._get_others_value_in_our_unit(other)
        )

    __rmul__ = __mul__

    def __truediv__(self, other: Any) -> UnitValue:
        return self._create_arithmetic_operation_result(
            self.value / self._get_others_value_in_our_unit(other)
        )

    def __rtruediv__(self, other: Any) -> UnitValue:
        return self._create_arithmetic_operation_result(
            self._get_others_value_in_our_unit(other) / self.value
        )

    def __pow__(self, other: Any) -> UnitValue:
        return self._create_arithmetic_operation_result(
            self.value ** self._get_others_value_in_our_unit(other)
        )

    def __rpow__(self, other: Any) -> UnitValue:
        return self._create_arithmetic_operation_result(
            self._get_others_value_in_our_unit(other) ** self.value
        )

    def _get_others_value_in_our_unit(self, other: Any) -> float:
        if (type_ := type(other)) is float:
            return other

        if type_ is int:
            return float(other)

        if not isinstance(other, UnitValue):
            raise TypeError(
                f"{type(self).__name__}s can only be used in operations "
                f"with integers, floats or other {type(self).__name__}s"
            )

        try:
            return self.converter.convert(other, self.unit).value
        except error.UnitConversionNotAdded:
            pass

        return other.converter.convert(other, self.unit).value

    def _create_arithmetic_operation_result(self, value: float) -> UnitValue:
        return UnitValue(value, self.unit, self.converter)
