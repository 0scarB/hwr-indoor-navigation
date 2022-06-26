import sys
import os
import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

sys.path.append(os.path.dirname(__file__))

import lidar

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.lidar = lidar.Lidar()
        self.publisher_ = self.create_publisher(LaserScan, 'topic', 10)

    def publish(self):
        for entry in self.lidar.capture_output():
            scan_data = LaserScan()
            scan_data.header.stamp = self.get_clock().now().to_msg()
            scan_data.header.frame_id = "laser_frame"
            scan_data.angle_min = entry.angle * math.pi / 180
            scan_data.angle_max = entry.angle * math.pi / 180
            scan_data.angle_increment = 0.0
            scan_data.time_increment = 0.0
            scan_data.scan_time = 0.0
            scan_data.range_min = entry.dist
            scan_data.range_max = entry.dist
            scan_data.ranges = [entry.dist]
            scan_data.intensities = []
            self.publisher_.publish(scan_data)
            self.get_logger().info(f"Publishing: {scan_data}")

    def fake_publish(self):
        n_readings = 100
        freq = 40
        count = 0
        while True:
            ranges = [5.0] * n_readings
            intensities = [float(100 + count)] * n_readings
            scan_data = LaserScan()
            scan_data.header.stamp = self.get_clock().now().to_msg()
            scan_data.header.frame_id = "laser_frame"
            scan_data.angle_min = -1.57
            scan_data.angle_max = 1.57
            scan_data.angle_increment = 3.14 / n_readings
            scan_data.time_increment = (1 / freq) / n_readings
            scan_data.scan_time = 1 / freq
            scan_data.range_min = 0.0
            scan_data.range_max = 100.0
            scan_data.ranges = ranges
            scan_data.intensities = intensities
            self.publisher_.publish(scan_data)
            self.get_logger().info(f"Publishing: {scan_data}")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    # minimal_publisher.publish()
    minimal_publisher.fake_publish()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
