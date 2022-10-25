from collections.abc import Callable

import event
import hardware


class CaptureLidarMeasurementResponseProcessor(
    event.Processor[
        hardware.event.Type.CAPTURED_LIDAR_MEASUREMENT,
        hardware.LidarMeasurement,
        None,
        None
    ]
):
    _handlers: list[Callable[[hardware.LidarMeasurement], None]]

    def __init__(self) -> None:
        self._handlers = []

    def add_handler(self, handler: Callable[[hardware.LidarMeasurement], None]) -> None:
        self._handlers.append(handler)

    def process(
            self,
            event_: event.Event[
                hardware.event.Type.CAPTURED_LIDAR_MEASUREMENT,
                event.ResponsesValue[hardware.LidarMeasurement, Exception]
            ]
    ) -> None:
        for failure in event_.value.failures:
            raise failure.value

        for success in event_.value.successes:
            for handler in self._handlers:
                handler(success.value)
