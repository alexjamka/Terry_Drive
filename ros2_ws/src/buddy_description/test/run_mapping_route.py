#!/usr/bin/env python3
"""Supervised short hardware mapping route; commands go through the scan guard.
Requires an observer, clear floor, teleop bringup and the installed 0.3302 m
command-kinematics firmware. No reversing. Saves a bag, events and the live map.
"""
import argparse,json,math,time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from rclpy.serialization import serialize_message
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu,LaserScan
from nav_msgs.msg import Odometry,OccupancyGrid
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import rosbag2_py

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('output',type=Path)
    ap.add_argument('--leg',type=float,default=.65)
    ap.add_argument('--adapt-clearance',action='store_true',help='Orient toward clearer side and shorten legs to observed space')
    ap.add_argument('--turn-command',type=float,default=1.6)
    args=ap.parse_args()
    if not .2<=args.leg<=.8 or not 1.0<=args.turn_command<=1.8:ap.error('route limits exceeded')
    args.output.mkdir(parents=True,exist_ok=False);rclpy.init();n=rclpy.create_node('supervised_mapping_route')
    writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(args.output/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    latest={};counts={};events=[];state={'yaw':0.,'imu_stamp':None,'path':0.,'xy':None}
    topics=[('/scan',LaserScan),('/imu/data',Imu),('/imu/data_raw',Imu),('/odom',Odometry),('/tf',TFMessage),('/tf_static',TFMessage),('/map',OccupancyGrid),('/scan_guard_status',String),('/cmd_vel',Twist)]
    writer.create_topic(rosbag2_py.TopicMetadata(name='/odom/wheel',type='nav_msgs/msg/Odometry',serialization_format='cdr'))
    subs=[]
    for topic,cls in topics:
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=f'{cls.__module__.split(".")[0]}/msg/{cls.__name__}',serialization_format='cdr'))
        qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic in ('/map','/tf_static') else qos_profile_sensor_data
        def recv(m,topic=topic):
            latest[topic]=(time.monotonic(),m);counts[topic]=counts.get(topic,0)+1
            now=n.get_clock().now().nanoseconds;blob=serialize_message(m);writer.write(topic,blob,now)
            if topic=='/odom':
                # This run is wheel-only. Store an explicit alias for offline A/B.
                writer.write('/odom/wheel',blob,now)
                xy=np.array([m.pose.pose.position.x,m.pose.pose.position.y])
                if state['xy'] is not None:state['path']+=float(np.linalg.norm(xy-state['xy']))
                state['xy']=xy
            if topic=='/imu/data':
                stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
                if state['imu_stamp'] is not None:
                    dt=stamp-state['imu_stamp']
                    if 0<dt<.1:state['yaw']+=m.angular_velocity.z*dt
                state['imu_stamp']=stamp
        subs.append(n.create_subscription(cls,topic,recv,qos))
    pub=n.create_publisher(Twist,'/cmd_vel_raw',10)
    started=time.monotonic()
    def command(v=0.,w=0.):
        m=Twist();m.linear.x=float(v);m.angular.z=float(w);pub.publish(m)
    def check():
        if time.monotonic()-started>120:raise RuntimeError('120 s route limit reached')
        for topic,age in [('/scan',.8),('/odom',.3),('/imu/data',.3),('/scan_guard_status',.3)]:
            if topic not in latest or time.monotonic()-latest[topic][0]>age:raise RuntimeError('stale '+topic)
        status=json.loads(latest['/scan_guard_status'][1].data)
        if status['scan_age'] is None or status['scan_age']>.8:raise RuntimeError('guard has stale scan')
        return status
    def wait(seconds,v=0.,w=0.,validate=True):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            if validate:check()
            command(v,w);rclpy.spin_once(n,timeout_sec=.02)
    def event(name,**kw):
        row={'event':name,'time_ns':n.get_clock().now().nanoseconds,'path_m':state['path'],'yaw_deg':math.degrees(state['yaw']),**kw};events.append(row);print(json.dumps(row),flush=True)
    def turn(angle):
        target=state['yaw']+angle;deadline=time.monotonic()+10
        event('turn_start',target_deg=math.degrees(target))
        for attempt in range(25):
            remaining=target-state['yaw']
            if abs(remaining)<math.radians(3):break
            if time.monotonic()>deadline:raise RuntimeError('turn timeout')
            before=state['yaw'];sign=1 if remaining>0 else -1
            pulse=min(.18,max(.035,abs(remaining)/2.5))
            wait(pulse,w=sign*args.turn_command);wait(.22)
            if (state['yaw']-before)*sign < -.04:raise RuntimeError('turn sign disagrees with gyro')
        else:raise RuntimeError('turn failed to converge')
        wait(2);event('turn_done',error_deg=math.degrees(target-state['yaw']))
    def drive(distance):
        status=check();front=status['sectors'].get('front')
        if front is None or status['points'].get('front',0)<8:raise RuntimeError('front coverage unknown')
        if front<distance+.5:raise RuntimeError(f'not enough clearance for planned leg: {front:.2f} m')
        p0=state['path'];yaw0=state['yaw'];t0=time.monotonic();event('drive_start',requested_m=distance)
        while state['path']-p0<distance:
            status=check();front=status['sectors'].get('front')
            if front is None or status['points'].get('front',0)<8 or front<.5:raise RuntimeError('front clearance/coverage limit')
            if time.monotonic()-t0>6:raise RuntimeError('drive timeout')
            if time.monotonic()-t0>1.5 and state['path']-p0<.02:raise RuntimeError('wheel motion stalled')
            if abs(state['yaw']-yaw0)>.18:raise RuntimeError('straight-leg heading deviation')
            command(.25);rclpy.spin_once(n,timeout_sec=.02)
        wait(2.5);event('drive_done',achieved_m=state['path']-p0)
    outcome='not_started'
    try:
        wait(20,validate=False)
        check()
        if '/map' not in latest:raise RuntimeError('SLAM has no map')
        status=check();event('ready',guard=status)
        sides=status['sectors'];left=sides.get('left');right=sides.get('right')
        direction=1 if left is not None and (right is None or left>=right) else -1
        if args.adapt_clearance and (sides.get('front') is None or sides['front']<args.leg+.5):
            turn(direction*math.pi/2)
        for leg in range(4):
            distance=args.leg
            if args.adapt_clearance:
                front=check()['sectors'].get('front')
                distance=min(distance, max(0.,front-.52)) if front is not None else 0.
                if distance<.18:raise RuntimeError(f'insufficient room for next leg: {front} m front clearance')
            event('leg',number=leg+1);drive(distance);turn(direction*math.pi/2)
        status=check();front=status['sectors'].get('front')
        if front is not None and front>.75:drive(.2)
        outcome='route_completed';wait(8)
    except (RuntimeError,KeyboardInterrupt) as e:
        outcome='stopped: '+str(e);event('route_stopped',reason=str(e))
    finally:
        wait(1,validate=False)
        if '/map' in latest:
            m=latest['/map'][1];grid=np.array(m.data).reshape(m.info.height,m.info.width);pixels=np.full(grid.shape,205,dtype=np.uint8);pixels[(grid>=0)&(grid<65)]=254;pixels[grid>=65]=0
            (args.output/'map.pgm').write_bytes(f'P5\n{m.info.width} {m.info.height}\n255\n'.encode()+np.flipud(pixels).tobytes())
            (args.output/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':m.info.resolution,'origin':[m.info.origin.position.x,m.info.origin.position.y,0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196,'mode':'trinary'}))
            np.savez_compressed(args.output/'map.npz',grid=grid,origin=[m.info.origin.position.x,m.info.origin.position.y],resolution=m.info.resolution)
        summary={'outcome':outcome,'path_m':state['path'],'gyro_yaw_deg':math.degrees(state['yaw']),'counts':counts,'events':events}
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print('STOPPED',json.dumps({k:v for k,v in summary.items() if k!='events'}),flush=True)
        del writer;n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
