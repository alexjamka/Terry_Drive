#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rplidar import RPLidar
import math
import time

class RPLidarNode(Node):
    def __init__(self):
        super().__init__('rplidar_scan_node')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.port = '/dev/ttyUSB0'
        self.get_logger().info(f'Connecting to RPLidar on {self.port}')
        
        try:
            self.lidar = RPLidar(self.port, baudrate=115200)
            info = self.lidar.get_info()
            self.get_logger().info(f'RPLidar info: {info}')
            health = self.lidar.get_health()
            self.get_logger().info(f'RPLidar health: {health}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            return

        self.timer = self.create_timer(0.001, self.dummy)
        self.scan_thread = __import__('threading').Thread(target=self.scan_loop, daemon=True)
        self.scan_thread.start()

    def dummy(self):
        pass

    def scan_loop(self):
        try:
            for scan in self.lidar.iter_scans(max_buf_meas=500):
                if not rclpy.ok():
                    break
                self.publish_scan(scan)
        except Exception as e:
            self.get_logger().error(f'Scan error: {e}')

    def publish_scan(self, scan):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = 2.0 * math.pi / 360.0
        msg.range_min = 0.15
        msg.range_max = 12.0
        msg.time_increment = 0.0

        ranges = [float('inf')] * 360
        for _, angle, distance in scan:
            if distance > 0:
                idx = int(angle) % 360
                ranges[idx] = distance / 1000.0
        
        msg.ranges = ranges
        self.publisher.publish(msg)

    def destroy_node(self):
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RPLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
