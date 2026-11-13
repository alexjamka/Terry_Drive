#!/usr/bin/env python3
"""Record an IMU/lidar A/B dataset. --turns physically turns the robot.

Run only on a clear floor with an observer. Requires hardware bringup in
mode:=teleop use_imu:=true. All commands pass through /cmd_vel_raw and guard.
No linear motion. The finally block sends stop even after a failed check.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import rosbag2_py


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('output', type=Path)
    ap.add_argument('--turns', action='store_true')
    ap.add_argument('--short-turns', action='store_true',help='two 0.9 s turns instead of four')
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('imu_lidar_test_recorder')
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(args.output/'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    latest, counts, events = {}, {}, []
    topics = [('/scan', LaserScan), ('/imu/data', Imu), ('/imu/data_raw', Imu),
              ('/odom/wheel', Odometry), ('/odom', Odometry), ('/tf', TFMessage),
              ('/tf_static', TFMessage), ('/scan_guard_status', String),
              ('/cmd_vel', Twist), ('/map', OccupancyGrid)]
    subs = []
    for topic, cls in topics:
        module = cls.__module__.split('.')[0]
        typename = f'{module}/msg/{cls.__name__}'
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=topic, type=typename, serialization_format='cdr'))
        qos = (QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
               if topic in ('/tf_static', '/map') else qos_profile_sensor_data)
        def receive(msg, topic=topic):
            now = node.get_clock().now().nanoseconds
            latest[topic] = (time.monotonic(), msg)
            counts[topic] = counts.get(topic, 0) + 1
            writer.write(topic, serialize_message(msg), now)
        subs.append(node.create_subscription(cls, topic, receive, qos))
    pub = node.create_publisher(Twist, '/cmd_vel_raw', 10)

    def run(seconds, angular=0.0, check=False):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if check:
                for topic, limit in [('/scan', 1.0), ('/imu/data', .3), ('/odom', .3)]:
                    if topic not in latest or time.monotonic()-latest[topic][0] > limit:
                        raise RuntimeError(f'stale required topic {topic}; stopping')
            cmd = Twist(); cmd.angular.z = angular
            pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=.04)

    try:
        run(20)
        for topic in ('/scan', '/imu/data', '/odom/wheel', '/odom', '/tf_static',
                      '/scan_guard_status', '/map'):
            if counts.get(topic, 0) < 1:
                raise RuntimeError(f'missing topic {topic}; no movement allowed')
        print('READY', counts, flush=True)
        if args.turns:
            sequence = ([(1.6, .9), (-1.6, .9)] if args.short_turns else
                        [(1.6, .9), (-1.6, .9), (1.6, 1.4), (-1.6, 1.4)])
            for angular, duration in sequence:
                start = node.get_clock().now().nanoseconds
                print(f'TURN {angular:+.1f} rad/s for {duration:.1f}s', flush=True)
                run(duration, angular, check=True)
                end = node.get_clock().now().nanoseconds
                events.append({'start_ns': start, 'end_ns': end,
                               'command_rad_s': angular, 'duration_s': duration})
                run(4, check=True)
        run(5)
    finally:
        run(.6)
        (args.output/'events.json').write_text(json.dumps(events, indent=2)+'\n')
        (args.output/'counts.json').write_text(json.dumps(counts, indent=2)+'\n')
        del writer
        node.destroy_node()
        rclpy.shutdown()
        print('STOPPED; saved', args.output, counts, flush=True)


if __name__ == '__main__':
    main()
