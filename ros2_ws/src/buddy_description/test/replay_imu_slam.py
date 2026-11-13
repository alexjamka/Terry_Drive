#!/usr/bin/env python3
"""Replay one recorded run through wheel-only or wheel/IMU lidar SLAM.

Uses an isolated ROS_DOMAIN_ID supplied by the caller; never runs motor nodes.
Both runs use identical scans and mapper parameters. All children are reaped.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from analyze_imu_turns import read_bag


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('run', type=Path);ap.add_argument('pipeline', choices=['wheel','imu'])
    ap.add_argument('--output', required=True,type=Path)
    ap.add_argument('--swap-wheel-sides',action='store_true',
                    help='normalize recordings made before the wheel-side correction')
    ap.add_argument('--allow-pure-rotation',action='store_true',
                    help='disable the translation-only front gate in older slam_toolbox')
    ap.add_argument('--linear-scale',type=float,default=1.0,help='Experimental wheel translation gain; preserve yaw calibration')
    ap.add_argument('--travel-distance', type=float, default=None)
    ap.add_argument('--motion-gate-distance', type=float, default=None,
                    help='Offline wheel-only distance OR heading scan gate')
    ap.add_argument('--motion-gate-heading', type=float, default=0.1)
    ap.add_argument('--debug-matching', action='store_true')
    ap.add_argument('--max-scan-yaw',type=float,default=None,help='Reject scans spanning more than this absolute wheel yaw (radians)')
    args=ap.parse_args()
    if args.motion_gate_distance is not None and args.pipeline != 'wheel':
        ap.error('motion gate currently supports wheel-only replay')
    if any(x is not None and x < 0 for x in (args.travel_distance, args.motion_gate_distance, args.motion_gate_heading)):
        ap.error('thresholds must be nonnegative')
    if not math.isfinite(args.linear_scale) or args.linear_scale <= 0:
        ap.error('linear scale must be finite and positive')
    if args.max_scan_yaw is not None and args.motion_gate_distance is None:
        ap.error('max-scan-yaw requires a motion gate')
    if os.environ.get('ROS_DOMAIN_ID') in (None,'42'):
        ap.error('choose an isolated ROS_DOMAIN_ID (e.g. 93 or 94)')
    args.output.mkdir(parents=True,exist_ok=False)
    data=read_bag(args.run/'bag')
    if args.swap_wheel_sides:
        previous = None
        x = y = 0.0
        for _,msg in data['/odom/wheel']:
            t=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            theta=-2*math.atan2(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
            if previous is not None:
                ds=msg.twist.twist.linear.x*(t-previous)
                x+=ds*math.cos(theta);y+=ds*math.sin(theta)
            previous=t
            msg.pose.pose.position.x=x;msg.pose.pose.position.y=y
            msg.pose.pose.orientation.z=math.sin(theta/2)
            msg.pose.pose.orientation.w=math.cos(theta/2)
            msg.twist.twist.angular.z *= -1
    for _,msg in data['/odom/wheel']:
        msg.pose.pose.position.x *= args.linear_scale
        msg.pose.pose.position.y *= args.linear_scale
        msg.twist.twist.linear.x *= args.linear_scale
        msg.twist.twist.linear.y *= args.linear_scale
    root=Path(__file__).resolve().parents[1]
    params=yaml.safe_load((root/'config/mapper_params_hardware.yaml').read_text())
    params['slam_toolbox']['ros__parameters']['use_sim_time']=True
    if args.allow_pure_rotation:
        params['slam_toolbox']['ros__parameters']['minimum_travel_distance']=0.0
    if args.travel_distance is not None:
        params['slam_toolbox']['ros__parameters']['minimum_travel_distance']=args.travel_distance
    if args.motion_gate_distance is not None:
        params['slam_toolbox']['ros__parameters']['minimum_travel_distance']=0.0
    params['slam_toolbox']['ros__parameters']['debug_logging']=args.debug_matching
    selection=[]
    if args.motion_gate_distance is not None:
        # Interpolate wheel odometry at the scan's acquisition timestamp, not
        # arrival time. Keep the original timestamps/order for accepted scans.
        od=data['/odom/wheel']
        ts=np.array([m.header.stamp.sec+m.header.stamp.nanosec/1e9 for _,m in od])
        xs=np.array([m.pose.pose.position.x for _,m in od])
        ys=np.array([m.pose.pose.position.y for _,m in od])
        angles=np.unwrap([2*math.atan2(m.pose.pose.orientation.z,m.pose.pose.orientation.w) for _,m in od])
        yaw_travel=np.r_[0.0,np.cumsum(abs(np.diff(angles)))]
        accepted=[];last=None
        for t,m in data['/scan']:
            st=m.header.stamp.sec+m.header.stamp.nanosec/1e9
            pose=np.array([np.interp(st,ts,xs),np.interp(st,ts,ys),np.interp(st,ts,angles)])
            dist=float(np.linalg.norm(pose[:2]-last[:2])) if last is not None else 0.0
            yaw=abs(math.atan2(math.sin(pose[2]-last[2]),math.cos(pose[2]-last[2]))) if last is not None else 0.0
            sweep=float(np.interp(st+m.scan_time,ts,yaw_travel)-np.interp(st,ts,yaw_travel))
            keep=(last is None or dist>=args.motion_gate_distance or yaw>=args.motion_gate_heading) and (args.max_scan_yaw is None or sweep<=args.max_scan_yaw)
            selection.append({'stamp':st,'keep':keep,'distance':dist,'heading':yaw,'scan_yaw_sweep':sweep})
            if keep:
                accepted.append((t,m));last=pose
        data['/scan']=accepted
        (args.output/'scan_selection.json').write_text(json.dumps(selection,indent=2))
    mapper=args.output/'mapper.yaml';mapper.write_text(yaml.safe_dump(params))
    rclpy.init();node=rclpy.create_node('imu_slam_replay')
    qos_static=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    static_pub=node.create_publisher(TFMessage,'/tf_static',qos_static)
    clock_pub=node.create_publisher(Clock,'/clock',10)
    topics=['/scan','/odom/wheel']+(['/imu/data'] if args.pipeline=='imu' else [])
    pubs={topic:node.create_publisher(type(data[topic][0][1]),topic,qos_profile_sensor_data)
          for topic in topics}
    tf_broadcaster=TransformBroadcaster(node)
    maps=[]; poses=[]; graphs=[]
    sub=node.create_subscription(OccupancyGrid,'/map',lambda m:maps.append(m),qos_static)
    pose_sub=node.create_subscription(PoseWithCovarianceStamped,'/pose',lambda m:poses.append(m),10)
    graph_sub=node.create_subscription(MarkerArray,'/slam_toolbox/graph_visualization',
                                      lambda m:graphs.append(m),10)
    children=[];logs=[]
    def launch(command, filename):
        log=open(args.output/filename,'w');logs.append(log)
        children.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    last_clock_ns = [0]
    def tick(t):
        msg=Clock();ns=max(last_clock_ns[0], round(t*1e9))
        last_clock_ns[0] = ns
        msg.clock.sec=ns//10**9;msg.clock.nanosec=ns%10**9;clock_pub.publish(msg)
    def spin(seconds,t):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            tick(t);rclpy.spin_once(node,timeout_sec=.01)
    try:
        launch(['ros2','launch','slam_toolbox','online_async_launch.py','use_sim_time:=true',
                f'slam_params_file:={mapper}'],'slam.log')
        if args.pipeline=='imu':
            launch(['ros2','run','robot_localization','ekf_node','--ros-args','-r','__node:=ekf_filter_node',
                    '--params-file',str(root/'config/ekf_imu.yaml'),'-p','use_sim_time:=true',
                    '-r','odometry/filtered:=/odom'],'ekf.log')
        records=sorted([(t,topic,msg) for topic in topics for t,msg in data[topic]],key=lambda x:x[0])
        t0=records[0][0]
        spin(5,t0)
        transforms={tr.child_frame_id:tr for _,msg in data['/tf_static'] for tr in msg.transforms}
        static=TFMessage();static.transforms=list(transforms.values());static_pub.publish(static)
        spin(2,t0)
        start=time.monotonic()
        for t,topic,msg in records:
            while time.monotonic()-start<t-t0:
                tick(min(t, t0+time.monotonic()-start))
                rclpy.spin_once(node,timeout_sec=.002)
            tick(t)
            if topic=='/odom/wheel' and args.pipeline=='wheel':
                tf=TransformStamped();tf.header=msg.header;tf.child_frame_id=msg.child_frame_id
                tf.transform.translation.x=msg.pose.pose.position.x
                tf.transform.translation.y=msg.pose.pose.position.y
                tf.transform.rotation=msg.pose.pose.orientation
                tf_broadcaster.sendTransform(tf)
            pubs[topic].publish(msg)
            rclpy.spin_once(node,timeout_sec=0)
        # Advance simulated time for the final map update, with last zero twist.
        end=records[-1][0]
        for step in range(1,151):
            tick(end+step*.02);rclpy.spin_once(node,timeout_sec=.02)
        if not maps:
            raise RuntimeError('SLAM produced no map; inspect slam.log')
        m=maps[-1];grid=np.array(m.data).reshape(m.info.height,m.info.width)
        occupied=int(np.count_nonzero(grid>=65));free=int(np.count_nonzero((grid>=0)&(grid<65)))
        metrics={'pipeline':args.pipeline,'linear_scale':args.linear_scale,'map_updates':len(maps),'processed_pose_count':len(poses),
                 'graph_node_markers':sum(m.ns=='slam_toolbox' for m in graphs[-1].markers) if graphs else None,
                 'graph_edge_segments':sum(len(m.points)//2 for m in graphs[-1].markers if m.ns=='slam_toolbox_edges') if graphs else None,
                 'forwarded_scans':len(data['/scan']),
                 'travel_distance':params['slam_toolbox']['ros__parameters']['minimum_travel_distance'],
                 'motion_gate_distance':args.motion_gate_distance,'motion_gate_heading':args.motion_gate_heading,'max_scan_yaw':args.max_scan_yaw,
                 'swap_wheel_sides':args.swap_wheel_sides,'allow_pure_rotation':args.allow_pure_rotation,'occupied_cells':occupied,
                 'free_cells':free,'explored_m2':(occupied+free)*m.info.resolution**2,
                 'occupied_fraction':occupied/(occupied+free),
                 'resolution':m.info.resolution,'width':m.info.width,'height':m.info.height}
        pixels=np.full(grid.shape,205,dtype=np.uint8)
        pixels[(grid>=0)&(grid<65)]=254;pixels[grid>=65]=0
        (args.output/'map.pgm').write_bytes(f'P5\n{m.info.width} {m.info.height}\n255\n'.encode()+np.flipud(pixels).tobytes())
        (args.output/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm',
            'resolution':m.info.resolution,'origin':[m.info.origin.position.x,m.info.origin.position.y,0.0],
            'negate':0,'occupied_thresh':.65,'free_thresh':.196,'mode':'trinary'}))
        np.savez_compressed(args.output/'map.npz',grid=grid,
                            origin=[m.info.origin.position.x,m.info.origin.position.y],resolution=m.info.resolution)
        trajectory=[{'t':m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                     'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,
                     'yaw':2*math.atan2(m.pose.pose.orientation.z,m.pose.pose.orientation.w)} for m in poses]
        (args.output/'trajectory.json').write_text(json.dumps(trajectory,indent=2))
        (args.output/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
        print(json.dumps(metrics),flush=True)
    finally:
        for proc in children:
            if proc.poll() is None: os.killpg(proc.pid,signal.SIGINT)
        for proc in children:
            try: proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for log in logs: log.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
