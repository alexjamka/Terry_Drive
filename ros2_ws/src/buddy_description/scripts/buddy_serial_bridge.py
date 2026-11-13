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
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from buddy_imu import ImuProcessor, parse_imu_line
from tf2_ros import TransformBroadcaster
import serial
import math
import threading
import time

class BuddySerialBridge(Node):
    def __init__(self):
        super().__init__('buddy_serial_bridge')

        # Robot parameters (must match ESP32 firmware)
        # EFFECTIVE track width for odometry, not the geometric one.
        #
        # Owner confirmed 2026-11-13: physical wheel centres are 15 inches
        # (0.381 m) apart, including during the recorded calibration runs.
        # The old 13-inch physical-width assertion was wrong. This effective
        # width is 5.3% above the actual track, not 21%.
        #
        # Keep the empirical yaw calibration: initial odometry used 0.3302,
        # and lidar comparisons measured ~1.215 excess rotation. Multiplying
        # that old estimate by 1.215 gave 0.4012. The correction included BOTH
        # the wrong geometry and tyre scrub; it was not all slip.
        # Firmware command kinematics and URDF use physical 0.381 m instead.
        # Do not replace effective width with physical width without a fresh
        # yaw calibration. See CLAUDE.md for firmware deployment status.
        # Refined 2026-11-13 after the owner corrected the physical track to
        # 15" (0.381 m). The original 0.4012 was derived as 0.3302 x 1.215,
        # using the WRONG geometric baseline (13"), so most of what was called
        # "tyre scrub" was really the bad track figure. Re-measured against two
        # independent sensors with 0.4012 in place:
        #
        #   lidar scan alignment : gain 0.968 (odom under-reports ~3%)
        #   gyro integration     : +92.5 vs +94.3, -92.3 vs -95.3 deg (~2-3%)
        #
        # 0.4012 x 0.968 = 0.3884, which is the physical 0.381 plus ~2% scrub -
        # a far more believable number than the 21% implied before.
        self.WHEEL_SEPARATION = 0.3884
        # Measured 2026-11-11: drive wheels are 2.5" diameter, not 0.070 m.
        self.WHEEL_DIAMETER = 0.0635
        # Derived from a field calibration rather than the datasheet. The old
        # value 526 carried the comment "11 PPR * 131 * 4 quadrature", which
        # actually multiplies to 5764 - so it never matched its own arithmetic.
        #
        # Calibration: commanded a drive until odom read 0.521 m (1246 ticks
        # centre average at the old scale); the tape measure said 21-22".
        # With the corrected 2.5" wheel that gives TICKS_PER_REV 445-466,
        # i.e. 455 +/- 10. The two old errors were nearly cancelling - wheel
        # too big (+10.2% on METERS_PER_TICK) against ticks/rev too big
        # (-13%) - which is why odom was only ~5% out and the fault hid.
        #
        # To refine without slip: lift the robot, mark a wheel, turn it exactly
        # one revolution by hand and count the tick delta. That is exact where
        # a driving measurement is contaminated by wheel slip.
        self.TICKS_PER_REV = 455
        self.METERS_PER_TICK = (self.WHEEL_DIAMETER * math.pi) / self.TICKS_PER_REV

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.last_encoder_time = None

        # The EKF must be the only odom->base TF publisher when enabled.
        # 2026-11-13: +angular physically turned RIGHT (owner observed),
        # while encoder yaw was positive; gyro and lidar both measured negative.
        # The ESP's L/R labels are physically exchanged. Normalize commands and
        # encoder sides together at this boundary; preserve calibrated scales.
        self.declare_parameter('swap_wheel_sides', True)
        self.swap_wheel_sides = self.get_parameter('swap_wheel_sides').value
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_bias_samples', 200)
        self.publish_odom_tf = self.get_parameter('publish_tf').value
        self.imu_processor = ImuProcessor(
            samples=self.get_parameter('imu_bias_samples').value)
        self.last_wheel_motion_ns = 0

        # Serial.
        #
        # The ESP32 is on USB. Both it and the YDLidar are CP2102 bridges
        # reporting serial "0001", so /dev/serial/by-id collides, and ttyUSB*
        # order is unstable. by-path is stable only until a cable moves - which
        # it has, repeatedly. So the default is to AUTODETECT: open each
        # candidate and keep the one that speaks our protocol.
        #
        # Set `port` explicitly to skip detection.
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('detect_timeout', 2.5)
        # An encoder jump larger than this means the ESP32 reset and its tick
        # counters restarted at zero; integrating that delta would teleport the
        # odometry. ~36 ticks is a full-speed frame at 20 Hz.
        self.declare_parameter('max_tick_jump', 500)

        baud = self.get_parameter('baud').value
        self.max_tick_jump = self.get_parameter('max_tick_jump').value
        requested = self.get_parameter('port').value

        if requested and requested != 'auto':
            port = requested
        else:
            port = self._detect_port(
                baud, self.get_parameter('detect_timeout').value)

        self.ser = None
        if port:
            try:
                # Opening a CP2102 toggles DTR/RTS, which resets the ESP32.
                # That's fine - we re-baseline on the first encoder frame - but
                # the base is briefly unresponsive right after this starts.
                self.ser = serial.Serial(port, baud, timeout=0.05, exclusive=True)
                self.get_logger().info(f'Connected to ESP32 on {port}')
            except Exception as e:
                self.get_logger().error(f'Failed to open {port}: {e}')
        if self.ser is None:
            self.get_logger().error(
                'No ESP32 found. The base will not move and no odometry will '
                'be published. Check `ls -l /dev/serial/by-path/`, confirm the '
                'board is powered, and set the `port` parameter if autodetect '
                'picked wrong.')

        # Publishers (UNCOMMENTED)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)
        self.imu_raw_pub = self.create_publisher(
            Imu, '/imu/data_raw', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(
            Imu, '/imu/data', qos_profile_sensor_data)
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

    def _detect_port(self, baud, timeout, attempts=3):
        """
        Find the port that emits our 'E:<l>,<r>' encoder frames.

        Three things this has to get right, each learned the hard way:

        * exclusive=True. pyserial's open does NOT fail just because another
          process holds the port, so probing blind fought the lidar driver for
          its tty - garbling both streams and then raising SerialException mid
          read, which killed this node outright.
        * An explicit reset into RUN mode. Opening a CP2102 asserts DTR/RTS,
          driving the ESP32's auto-reset circuit; merely de-asserting both can
          leave the chip in download mode, silent, and a healthy board then
          reports "no encoder frames".
        * Tolerate a failing candidate. A port that errors is skipped, not
          fatal.
        """
        import glob
        candidates = sorted(glob.glob('/dev/serial/by-path/*')) or \
            sorted(glob.glob('/dev/ttyUSB*')) + sorted(glob.glob('/dev/ttyACM*'))
        self.get_logger().info(
            f'Autodetecting ESP32 across {len(candidates)} serial port(s)...')

        for attempt in range(1, attempts + 1):
            for cand in candidates:
                try:
                    # exclusive=True makes a port the lidar driver already owns
                    # fail to open, which is exactly the skip we want.
                    probe = serial.Serial(cand, baud, timeout=0.2, exclusive=True)
                except Exception as e:
                    self.get_logger().info(
                        f'  {cand}: in use or unavailable ({e.__class__.__name__})')
                    continue
                try:
                    # Pulse EN with GPIO0 high -> clean boot into RUN mode.
                    probe.setDTR(False)
                    probe.setRTS(True)
                    time.sleep(0.2)
                    probe.setRTS(False)
                    time.sleep(1.5)
                    probe.reset_input_buffer()
                    buf = ''
                    deadline = time.time() + timeout
                    while time.time() < deadline:
                        buf += probe.read(256).decode('utf-8', errors='ignore')
                        if 'E:' in buf:
                            for line in buf.split('\n'):
                                line = line.strip()
                                if line.startswith('E:') and ',' in line:
                                    self.get_logger().info(
                                        f'  {cand}: found encoder frames')
                                    return cand
                    self.get_logger().info(f'  {cand}: no encoder frames')
                except Exception as e:
                    self.get_logger().warn(
                        f'  {cand}: probe failed ({e.__class__.__name__}: '
                        f'{str(e)[:60]}), skipping')
                finally:
                    try:
                        probe.close()
                    except Exception:
                        pass
            if attempt < attempts:
                self.get_logger().info(
                    f'  no ESP32 on attempt {attempt}/{attempts}, retrying...')
        return None

    def cmd_vel_callback(self, msg):
        if self.ser is None:
            return
        angular = -msg.angular.z if self.swap_wheel_sides else msg.angular.z
        # Avoid '-0.0000': the stationary calibration check recognizes zero.
        angular = 0.0 if abs(angular) < 0.00005 else angular
        self.last_cmd = f'V:{msg.linear.x:.4f},{angular:.4f}\n'
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
                        elif line.startswith('I:'):
                            self.process_imu_line(line)
                        elif line.startswith('IMU:'):
                            self.get_logger().info(line)
            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')

    def process_encoder_line(self, line):
        try:
            parts = line[2:].split(',')
            left_ticks = int(parts[0])
            right_ticks = int(parts[1])
        except (ValueError, IndexError):
            return

        if self.swap_wheel_sides:
            left_ticks, right_ticks = right_ticks, left_ticks
        now = self.get_clock().now()

        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.last_encoder_time = now
            return

        # An implausible jump means the ESP32 reset and its counters restarted
        # at zero. Re-baseline instead of integrating a bogus delta, which would
        # otherwise teleport the robot in /odom and drag the map frame with it.
        if (abs(left_ticks - self.prev_left_ticks) > self.max_tick_jump or
                abs(right_ticks - self.prev_right_ticks) > self.max_tick_jump):
            self.get_logger().warn(
                f'Encoder jump (L {self.prev_left_ticks}->{left_ticks}, '
                f'R {self.prev_right_ticks}->{right_ticks}); assuming ESP32 '
                f'reset and re-baselining odometry.')
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

        if delta_left or delta_right:
            self.last_wheel_motion_ns = now.nanoseconds

        stamp = now.to_msg()
        # Publish both the TF tree update and the physical Odometry topic
        if self.publish_odom_tf:
            self.publish_tf(stamp)
        self.publish_odom(stamp, dist_center / dt, delta_theta / dt)

    def process_imu_line(self, line):
        try:
            sample = parse_imu_line(line)
        except ValueError:
            return
        now = self.get_clock().now()
        command_age = (now - self.last_cmd_time).nanoseconds / 1e9
        commanded = command_age < 0.5 and self.last_cmd != 'V:0.0000,0.0000\n'
        stationary = (self.last_encoder_time is not None and
                      (now - self.last_encoder_time).nanoseconds < 250_000_000 and
                      now.nanoseconds - self.last_wheel_motion_ns > 500_000_000 and
                      not commanded)
        old_bias = self.imu_processor.bias
        result = self.imu_processor.update(sample, now.nanoseconds, stationary)
        if result is None:
            return
        stamp_ns, corrected = result
        msg = Imu()
        msg.header.stamp.sec = stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
        msg.header.frame_id = 'BuddyImu_link'
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0  # orientation is not measured
        for i, axis in enumerate(('x', 'y', 'z')):
            setattr(msg.angular_velocity, axis, sample.gyro[i])
            setattr(msg.linear_acceleration, axis, sample.acceleration[i])
            # Conservative initial variances, not a calibrated noise model.
            msg.angular_velocity_covariance[i * 4] = 0.005 ** 2
            msg.linear_acceleration_covariance[i * 4] = 0.2 ** 2
        self.imu_raw_pub.publish(msg)
        if corrected is not None:
            for i, axis in enumerate(('x', 'y', 'z')):
                setattr(msg.angular_velocity, axis, corrected[i])
            self.imu_pub.publish(msg)
        if old_bias is None and self.imu_processor.bias is not None:
            self.get_logger().info(
                'IMU gyro bias calibrated (rad/s): ' +
                ', '.join(f'{x:.6f}' for x in self.imu_processor.bias))
        elif old_bias is not None and self.imu_processor.bias is None:
            self.get_logger().warn('IMU clock reset/outage: recalibrating gyro')

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
        
        # Nonzero variances let robot_localization weight wheel/IMU rates.
        # Pose is deliberately not fused; all wheel values share encoders.
        for i, variance in enumerate((0.05**2, 0.05**2, 1.0, 1.0, 1.0, 0.1**2)):
            odom.pose.covariance[i * 7] = variance
        for i, variance in enumerate((0.03**2, 0.02**2, 1.0, 1.0, 1.0, 0.1**2)):
            odom.twist.covariance[i * 7] = variance
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