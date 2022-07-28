import threading
import time

import rclpy
from rclpy.node import Node
import tf2_ros
import tf_transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

import interface


class OdometryDataPublisher(Node, interface.WithStartup, interface.WithShutdown):

    def __init__(self):
        super().__init__('robot_odom')
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)

    def startup(self) -> None:
        self.publish_thread = threading.Thread(target=self.publish)
        self.publish_thread.start()

    def shutdown(self) -> None:
        self.publish_thread.join()

    def publish(self) -> None:
        r = rclpy.Rate(1.0)
        while True:
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.child_frame_id = "base_link"

            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0

            # For the same reason, turtle can only rotate around one axis
            # and this why we set rotation in x and y to 0 and obtain
            # rotation in z axis from the message
            q = tf_transformations.quaternion_from_euler(0, 0, 0.0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.br.sendTransform(t)

            time.sleep(0.5)

