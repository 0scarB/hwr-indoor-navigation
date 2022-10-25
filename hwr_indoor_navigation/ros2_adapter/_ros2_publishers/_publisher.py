from collections.abc import Callable
from typing import Generic, TypeVar, ClassVar

import rclpy

from interface import WithStartup, WithShutdown

_ROSMessage = TypeVar("_ROSMessage")


class Publisher(Generic[_ROSMessage], rclpy.Node, WithStartup, WithShutdown):
    _started_instances_count: ClassVar[int] = 0
    _shutdown_instances_count: ClassVar[int] = 0

    _publish_ros_message: Callable[[_ROSMessage], None]

    def __init__(
            self,
            ros_message_type: _ROSMessage,
            ros_node_name: str,
            ros_topic_name: str
    ):
        super(rclpy.Node, self).__init__(ros_node_name)
        self._publish_ros_message = self.create_publisher(
            ros_message_type,
            ros_topic_name
        ).publish

    def startup(self) -> None:
        if self._started_instances_count == 0:
            rclpy.init(args=None)

        self._started_instances_count += 1

    def shutdown(self) -> None:
        rclpy.spin(self)
        self.destroy_node()

        if self._shutdown_instances_count == self._started_instances_count - 1:
            rclpy.shutdown()
