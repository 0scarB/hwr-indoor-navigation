import event
import hardware


class CaptureLidarMeasurementResponseProcessor(
    event.Processor[
        hardware.event.Type.CAPTURED_LIDAR_MEASUREMENT,
        hardware.
    ]
):
