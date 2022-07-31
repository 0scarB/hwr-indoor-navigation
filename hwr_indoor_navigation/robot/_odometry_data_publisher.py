import math
import threading
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
import tf2_ros
import tf_transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped

import interface
from unit import UnitValue


@dataclass(frozen=True)
class OdometryData:
    heading: UnitValue
    speed: UnitValue


class OdometryDataPublisher(Node, interface.WithStartup, interface.WithShutdown):
    _x: float
    _y: float

    def __init__(self):
        super().__init__('robot_odom')
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)

        self._vx = 0
        self._vy = 0
        self._x = 0
        self._y = 0

    def startup(self) -> None:
        self.publish_thread = threading.Thread(target=self.publish)
        self.publish_thread.start()

    def shutdown(self) -> None:
        self.publish_thread.join()
        rclpy.spin(self)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        self.destroy_node()

    def handle_change_speed(self, new_speed: UnitValue) -> None:
        self.handle_change_odometry(OdometryData(
            heading=self._current_odometry.heading,
            speed=new_speed,
        ))

    def handle_change_heading(self, new_heading: UnitValue) -> None:
        self.handle_change_odometry(OdometryData(
            heading=new_heading,
            speed=self._current_odometry.speed,
        ))

    def handle_change_odometry(self, new_odometry_data: OdometryData) -> None:
        heading_in_radians = new_odometry_data.heading.to("radians").value
        speed_in_meters_per_second = new_odometry_data.speed.to("m/s").value

        self._vx = math.cos(heading_in_radians) * speed_in_meters_per_second
        self._vy = math.sin(heading_in_radians) * speed_in_meters_per_second

        self._x += self._vx / speed_in_meters_per_second
        self._y += self._vy / speed_in_meters_per_second

    def publish(self) -> None:
        while True:
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"

            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.translation.z = 0.0

            # For the same reason, turtle can only rotate around one axis
            # and this why we set rotation in x and y to 0 and obtain
            # rotation in z axis from the message
            q = tf_transformations.quaternion_from_euler(0, 0, 0.0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.odom_broadcaster.sendTransform(t)

            odom_data = Odometry()
            odom_data.header.frame_id = t.header.frame_id
            odom_data.header.stamp = t.header.stamp
            odom_data.child_frame_id = t.child_frame_id

            # set the position
            odom_data.pose.pose.position.x = self._x
            odom_data.pose.pose.position.y = self._y
            odom_data.pose.pose.position.z = 0.0
            odom_data.pose.pose.orientation.x = q[0]
            odom_data.pose.pose.orientation.y = q[1]
            odom_data.pose.pose.orientation.z = q[2]
            odom_data.pose.pose.orientation.w = q[3]

            # set the velocity
            odom_data.twist.twist.linear.x = self._vx
            odom_data.twist.twist.linear.y = self._vy
            odom_data.twist.twist.angular.z = 0.0

            self.odom_pub.publish(odom_data)

            time.sleep(0.001)
