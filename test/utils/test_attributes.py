import pytest

from attribute import NumberBetweenBounds
from hwr_indoor_navigation import error


class TestNumberBetweenBounds:

    def test_cannot_initialize_value_below_lower_bound(self):
        with pytest.raises(errors.ValueOutOfBounds, match=r".*below lower bound -10"):
            class C:
                a = NumberBetweenBounds(-11, lower_bound=-10, upper_bound=10)

    def test_cannot_initialize_value_above_upper_bound(self):
        with pytest.raises(errors.ValueOutOfBounds, match=r".*above upper bound 10"):
            class C:
                a = NumberBetweenBounds(11, lower_bound=-10, upper_bound=10)

    def test_can_initialize_value_at_lower_bound(self):
        class C:
            a = NumberBetweenBounds(-10.5, lower_bound=-10.5, upper_bound=10.5)

    def test_can_initialize_value_at_upper_bound(self):
        class C:
            a = NumberBetweenBounds(10.5, lower_bound=-10.5, upper_bound=10.5)

    def test_can_initialize_value_between_bounds(self):
        class C:
            a = NumberBetweenBounds(0.0, lower_bound=-10.5, upper_bound=10.5)

    def test_cannot_set_value_below_lower_bound(self):
        class C:
            a = NumberBetweenBounds(0.0, lower_bound=-10.5, upper_bound=10.5)

        with pytest.raises(errors.ValueOutOfBounds, match=r".*below lower bound -10.5"):
            C().a = -10.6

        with pytest.raises(errors.ValueOutOfBounds, match=r".*below lower bound -10.5"):
            C().a -= 10.6

    def test_cannot_set_value_above_upper_bound(self):
        class C:
            a = NumberBetweenBounds(0.0, lower_bound=-10.5, upper_bound=10.5)

        with pytest.raises(errors.ValueOutOfBounds, match=r".*above upper bound 10.5"):
            C().a = 10.6

        with pytest.raises(errors.ValueOutOfBounds, match=r".*above upper bound 10.5"):
            C().a += 10.6
