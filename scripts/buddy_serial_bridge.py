#!/usr/bin/env python3
"""
buddy_serial_bridge.py
- Subscribes to /cmd_vel -> sends V:linear,angular to ESP32
- Reads E:left,right from ESP32 -> publishes /odom + odom->base TF
- Resends last command at 10Hz to ensure ESP32 stays in synccd ~/buddy_ws
colcon build --packages-select buddy_description --symlink-install
source install/setup.bash

# Launch
ros2 launch buddy_description buddy_hardware.launch.py
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math
import threading

class BuddySerialBridge(Node):
    def __init__(self):
        super().__init__('buddy_serial_bridge')

        # Robot parameters (must match ESP32 firmware)
        self.WHEEL_SEPARATION = 0.3302
        self.WHEEL_DIAMETER = 0.070
        self.TICKS_PER_REV = 526  # 11 PPR * 131 * 4 quadrature
        self.METERS_PER_TICK = (self.WHEEL_DIAMETER * math.pi) / self.TICKS_PER_REV

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.last_encoder_time = None

        # Serial
        try:
            self.ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.05)
            self.get_logger().info('Connected to ESP32 on /dev/ttyTHS1')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial: {e}')
            self.ser = None

        # Publishers (UNCOMMENTED)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.last_cmd = 'V:0.0000,0.0000\n'
        self.last_cmd_time = self.get_clock().now()
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Resend last command at 10Hz to ensure ESP32 stays in sync
        self.create_timer(0.1, self.resend_cmd)

        # Read serial in a background thread
        self.serial_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.serial_thread.start()

        self.get_logger().info('Buddy serial bridge started. Odom active.')

    def cmd_vel_callback(self, msg):
        if self.ser is None:
            return
        self.last_cmd = f'V:{msg.linear.x:.4f},{msg.angular.z:.4f}\n'
        self.last_cmd_time = self.get_clock().now()
        try:
            self.ser.write(self.last_cmd.encode())
            # self.get_logger().info(f'Sent: {self.last_cmd.strip()}') # Optional: mute this to reduce log spam
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    def resend_cmd(self):
        if self.ser is None:
            return
        try:
            elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            if elapsed > 0.5:
                self.ser.write(b'V:0.0000,0.0000\n')
            else:
                self.ser.write(self.last_cmd.encode())
        except Exception:
            pass

    def serial_read_loop(self):
        buffer = ''
        while rclpy.ok():
            if self.ser is None:
                break
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    buffer += data.decode('utf-8', errors='ignore')
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line.startswith('E:'):
                            self.process_encoder_line(line)
            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')

    def process_encoder_line(self, line):
        try:
            parts = line[2:].split(',')
            left_ticks = int(parts[0])
            right_ticks = int(parts[1])
        except (ValueError, IndexError):
            return

        now = self.get_clock().now()

        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.last_encoder_time = now
            return

        dt = (now - self.last_encoder_time).nanoseconds / 1e9
        self.last_encoder_time = now
        if dt <= 0:
            return

        delta_left = left_ticks - self.prev_left_ticks
        delta_right = right_ticks - self.prev_right_ticks
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        dist_left = delta_left * self.METERS_PER_TICK
        dist_right = delta_right * self.METERS_PER_TICK

        dist_center = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.WHEEL_SEPARATION

        self.theta += delta_theta
        self.x += dist_center * math.cos(self.theta)
        self.y += dist_center * math.sin(self.theta)

        stamp = now.to_msg()
        # Publish both the TF tree update and the physical Odometry topic
        self.publish_tf(stamp)
        self.publish_odom(stamp, dist_center / dt, delta_theta / dt)

    def publish_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'BuddyBaseFootprint_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

    # ADDED: The missing odometry publisher method
    def publish_odom(self, stamp, linear_vel, angular_vel):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'BuddyBaseFootprint_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        
        self.odom_pub.publish(odom)

    def destroy_node(self):
        if self.ser:
            try:
                self.ser.write(b'V:0.0000,0.0000\n')
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BuddySerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()