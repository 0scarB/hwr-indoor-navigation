import sys
import os
import math
import time
import asyncio
import interface
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ._component import Lidar

sys.path.append(os.path.dirname(__file__))

class LidarDataPublisher(Node, interface.WithStartup, interface.WithShutdown):

    def __init__(self):
        super().__init__('lidar')
        self.lidar = Lidar()
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.is_started: bool = False
        self.is_stopped: bool = True

    def startup(self) -> None:
        self.is_started = True
        self.is_stopped = False
        self.capture_data_thread = threading.Thread(target=self.publish)
        self.capture_data_thread.start()

    def shutdown(self) -> None:
        self.is_started = False
        self.capture_data_thread.join(timeout=5)
        rclpy.spin(self)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        self.destroy_node()
        rclpy.shutdown()

    def publish(self):
        output = self.lidar.capture_output()

        while self.is_started:
            rot_start = time.time()

            ranges = []
            intensities = []
            n_readings = 0
            while self.is_started:
                entry = next(output)

                if entry.angle == 0:
                    rot_end = time.time()
                    rot_duration = rot_end - rot_start

                    scan_data = LaserScan()
                    scan_data.header.stamp = self.get_clock().now().to_msg()
                    scan_data.header.frame_id = "odom"
                    scan_data.angle_min = 0.0
                    scan_data.angle_max = 2 * math.pi
                    scan_data.angle_increment = 2 * math.pi / n_readings
                    scan_data.time_increment = rot_duration / n_readings
                    scan_data.scan_time = rot_duration
                    scan_data.range_min = 0.0
                    scan_data.range_max = 10.0
                    scan_data.ranges = ranges
                    scan_data.intensities = intensities
                    self.publisher_.publish(scan_data)
                    #self.get_logger().info(f"Publishing: {scan_data}")

                    ranges.clear()
                    intensities.clear()
                    n_readings = 0
                    break

                ranges.append(float(entry.dist))
                intensities.append(float(entry.angle))

                n_readings += 1
        self.is_stopped = True