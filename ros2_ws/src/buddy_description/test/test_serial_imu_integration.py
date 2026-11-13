#!/usr/bin/env python3
"""Exercise the real serial bridge against a fake ESP32 PTY, never motor hardware."""
import math
import os
import pty
import select
import signal
import subprocess
import tempfile
import threading
import time
import unittest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data


class TestSerialImu(unittest.TestCase):
    def test_commands_encoders_calibration_and_tf_mode(self):
        self.assertNotEqual(os.environ.get('ROS_DOMAIN_ID'),'42','use an isolated test domain')
        master,slave=pty.openpty();port=os.ttyname(slave)
        state={'running':True,'left':0,'right':0,'gyro':131,'commands':[]}
        def esp():
            started=time.monotonic();last_enc=0;buf=b''
            while state['running']:
                elapsed=time.monotonic()-started
                if select.select([master],[],[],0)[0]:
                    buf+=os.read(master,4096)
                    while b'\n' in buf:
                        line,buf=buf.split(b'\n',1)
                        if line.startswith(b'V:'):state['commands'].append(line.decode())
                if elapsed-last_enc>=.05:
                    os.write(master,f"E:{state['left']},{state['right']}\n".encode());last_enc=elapsed
                os.write(master,f"I:{int(elapsed*1e6)},0,0,16384,0,0,{state['gyro']}\n".encode())
                time.sleep(.02)
        thread=threading.Thread(target=esp,daemon=True);thread.start()
        log=tempfile.TemporaryFile(mode='w+')
        proc=subprocess.Popen(['ros2','run','buddy_description','buddy_serial_bridge.py','--ros-args',
              '-p',f'port:={port}','-p','publish_tf:=false','-p','odom_topic:=/odom/wheel'],
              stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();node=rclpy.create_node('serial_imu_integration_test');received={}
        subs=[node.create_subscription(typ,topic,lambda m,t=topic:received.update({t:m}),qos_profile_sensor_data)
              for typ,topic in [(Imu,'/imu/data'),(Imu,'/imu/data_raw'),(Odometry,'/odom/wheel')]]
        pub=node.create_publisher(Twist,'/cmd_vel',10)
        def spin(seconds,cmd=None):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                if cmd is not None:pub.publish(cmd)
                rclpy.spin_once(node,timeout_sec=.02)
        try:
            spin(9)
            self.assertIsNone(proc.poll(),'bridge exited unexpectedly')
            self.assertIn('/imu/data',received,'stationary calibration never completed')
            raw=received['/imu/data_raw'];corrected=received['/imu/data']
            self.assertAlmostEqual(raw.angular_velocity.z,math.radians(1),places=5)
            self.assertAlmostEqual(corrected.angular_velocity.z,0,places=5)
            self.assertEqual(corrected.header.frame_id,'BuddyImu_link')
            self.assertEqual(corrected.orientation_covariance[0],-1.0)
            self.assertAlmostEqual(corrected.linear_acceleration.z,9.80665)
            self.assertGreater(received['/odom/wheel'].twist.covariance[35],0)
            cmd=Twist();cmd.angular.z=1.0;spin(.4,cmd)
            self.assertIn('V:0.0000,-1.0000',state['commands'])
            spin(.3,Twist())
            # Raw ESP right label is physical left. Forward travel there is +yaw.
            state['left']=-10;state['right']=10;spin(.07)
            self.assertLess(received['/odom/wheel'].pose.pose.orientation.z,0)
            # The calibrated bias remains fixed through real rotation.
            state['gyro']=13231;spin(.15)
            self.assertAlmostEqual(received['/imu/data'].angular_velocity.z,math.radians(100),places=4)
        finally:
            pub.publish(Twist());spin(.1)
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            state['running']=False;thread.join(timeout=1)
            node.destroy_node();rclpy.shutdown();os.close(master);os.close(slave)
            if self._outcome.result and (self._outcome.result.failures or self._outcome.result.errors):
                log.seek(0);print(log.read())
            log.close()


if __name__=='__main__': unittest.main()
