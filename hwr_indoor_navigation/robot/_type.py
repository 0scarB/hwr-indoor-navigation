from dataclasses import dataclass

from unit import UnitValue


@dataclass(frozen=True)
class LidarMeasurement:
    signal: int
    distance: UnitValue
    angle: UnitValue
