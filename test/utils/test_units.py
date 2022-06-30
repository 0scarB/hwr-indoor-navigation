import math

import pytest

from hwr_indoor_navigation.utils.units import Converter, UnitValue


def test_scenario_degrees_radians_tau():
    angles_converter = Converter()
    angles_converter.add_linear_conversion(
        "degrees",
        "radians",
        math.pi / 180
    )
    angles_converter.add_linear_conversion(
        "tau",
        "radians",
        2 * math.pi,
    )
    angles_converter.add_linear_conversion(
        "m",
        "cm",
        0.01,
    )

    alpha = UnitValue(90, "degrees", angles_converter)
    beta = UnitValue(math.pi, "radians", angles_converter)
    gamma = UnitValue(1, "tau", angles_converter)

    tolerance = 1e-8
    def approx_equals(expected, actual) -> bool:
        return expected - tolerance <= actual <= expected + tolerance

    assert approx_equals(alpha.value, alpha.to("degrees"))
    assert approx_equals(math.pi / 2, alpha.to("radians"))
    assert approx_equals(0.25, alpha.to("tau"))

    assert approx_equals(180, beta.to("degrees"))
    assert approx_equals(beta.value, beta.to("radians"))
    assert approx_equals(0.5, beta.to("tau"))

    assert approx_equals(360, gamma.to("degrees"))
    assert approx_equals(2 * math.pi, gamma.to("radians"))
    assert approx_equals(gamma.value, gamma.to("tau"))

    assert approx_equals(gamma, 2 * alpha + beta)
    alpha += alpha
    assert approx_equals(gamma, alpha + beta)
    assert approx_equals(1, alpha / beta)
