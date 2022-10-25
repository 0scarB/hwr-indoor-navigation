from __future__ import annotations

import event as global_event

from ..._type import LidarMeasurement
from .._type import Type as EventType


class LidarMeasurementPublisher(
    global_event.ExposedPublisher[
        EventType.CAPTURED_LIDAR_MEASUREMENT,
        LidarMeasurement,
    ]
):
    def publish(
            self,
            measurement: LidarMeasurement
    ) -> list[tuple[EventType.CAPTURED_LIDAR_MEASUREMENT, LidarMeasurement]]:
        return [(EventType.CAPTURED_LIDAR_MEASUREMENT, measurement)]
