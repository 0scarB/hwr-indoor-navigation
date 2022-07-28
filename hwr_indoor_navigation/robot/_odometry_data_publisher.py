import threading

import rclpy
from rclpy.node import Node
import tf2_ros
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
        pass

    def publish(self) -> None:
        r = rclpy.Rate(1.0)
        while True:
            x = 0.0
            y = 0.0
            th = 0.0

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf2_ros.transformations.quaternion_from_euler(0, 0, th)

            # first, we'll publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                rclpy.Time.now(),
                "base_link",
                "odom"
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = rclpy.Time.now()
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            vx = 0.0
            vy = -0.0
            vth = 0.0

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

            # publish the message
            self.odom_pub.publish(odom)

            r.sleep()

