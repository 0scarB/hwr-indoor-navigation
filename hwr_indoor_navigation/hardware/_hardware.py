from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Type, TypeVar, Any

import event as global_event
from unit import UnitValue
from . import _event as event
from ._event import _processor as event_processor
from ._event import _publisher as event_publisher
from . import _component as component
from ._config import Config
from ._type import LidarMeasurement


class Hardware(global_event.Service[Config]):
    """Service class for interacting with the robot's hardware components."""

    def setup(self, broker: global_event.Broker) -> None:
        request_event_processors = self._create_request_event_processors(broker)
        event_publishers = self._create_event_publishers(broker)

        drive_motor = self._create_component(request_event_processors, component.DriveMotor)
        steering_motor = self._create_component(request_event_processors, component.SteeringMotor)
        lidar = self._create_component(request_event_processors, component.Lidar)

        request_event_processors.set_speed.add_handler(
            self.create_set_speed_handler(drive_motor)
        )
        request_event_processors.set_heading.add_handler(
            self.create_set_heading_handler(steering_motor)
        )
        lidar.on_capture_measurement(
            self.create_lidar_measurement_handler(event_publishers)
        )

    def create_set_speed_handler(
            self,
            drive_motor: component.DriveMotor
    ) -> Callable[[UnitValue], tuple[Any, UnitValue] | tuple[Any, None]]:
        def handler(speed: UnitValue) -> tuple[None, None]:
            drive_motor.set_speed(speed)
            return None, None

        return handler

    def create_set_heading_handler(
            self,
            steering_motor: component.SteeringMotor
    ) -> Callable[[UnitValue], tuple[Any, UnitValue] | tuple[Any, None]]:
        def handler(speed: UnitValue) -> tuple[None, None]:
            steering_motor.set_heading(speed)
            return None, None

        return handler

    def create_lidar_measurement_handler(
            self,
            event_publishers: "_EventPublishers"
    ) -> Callable[[LidarMeasurement], None]:
        def handler(measurement: LidarMeasurement) -> None:
            event_publishers.lidar_measurement.publish(measurement)

        return handler

    @dataclass(frozen=True)
    class _RequestEventProcessors:
        startup: global_event.processor.StartupRequestProcessor
        shutdown: global_event.processor.ShutdownRequestProcessor
        set_speed: event_processor.SetSpeedRequestProcessor
        set_heading: event_processor.SetHeadingRequestProcessor

    def _create_request_event_processors(self, broker: global_event.Broker) -> _RequestEventProcessors:
        processors = self._RequestEventProcessors(
            startup=global_event.processor.StartupRequestProcessor(),
            shutdown=global_event.processor.ShutdownRequestProcessor(),
            set_speed=event_processor.SetSpeedRequestProcessor(),
            set_heading=event_processor.SetHeadingRequestProcessor()
        )

        broker.add_processor(
            global_event.Topic.REQUEST,
            global_event.Type.STARTUP,
            processors.startup,
        )
        broker.add_processor(
            global_event.Topic.REQUEST,
            global_event.Type.SHUTDOWN,
            processors.shutdown,
        )
        broker.add_processor(
            global_event.Topic.REQUEST,
            event.Type.SET_SPEED,
            processors.set_speed,
        )
        broker.add_processor(
            global_event.Topic.REQUEST,
            event.Type.SET_HEADING,
            processors.set_heading,
        )

        return processors

    @dataclass(frozen=True)
    class _EventPublishers:
        lidar_measurement: event_publisher.LidarMeasurementPublisher

    def _create_event_publishers(self, broker: global_event.Broker) -> _EventPublishers:
        publishers = self._EventPublishers(
            lidar_measurement=event_publisher.LidarMeasurementPublisher()
        )

        broker.add_publisher(
            global_event.Topic.RESPONSES,
            publishers.lidar_measurement
        )

        return publishers

    _Component = TypeVar("_Component", bound=component.Component)

    @staticmethod
    def _create_component(
            request_event_processors: _RequestEventProcessors,
            component_class: Type[_Component]
    ) -> _Component:
        component = component_class()
        request_event_processors.startup.startup_obj_on_request(component)
        request_event_processors.shutdown.shutdown_obj_on_request(component)

        return component
