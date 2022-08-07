from collections.abc import Callable

import rclpy

from geometry_msgs.msg import PoseWithCovarianceStamped


class AMCLPoseSubscriber(rclpy.node.Node):
    _pose_msg_handlers: list[Callable[[PoseWithCovarianceStamped], None]]

    def __init__(self):
        print("init")
        super().__init__("pose_subscriber")
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self._handle_msg,
            10
        )

        self._pose_msg_handlers = []
        rclpy.spin(self)

    def add_handler(self, handler: Callable[[PoseWithCovarianceStamped], None]) -> None:
        self._pose_msg_handlers.append(handler)

    def _handle_msg(self, msg: PoseWithCovarianceStamped) -> None:
        print("1", msg)
        for handler in self._pose_msg_handlers:
            handler(msg)
