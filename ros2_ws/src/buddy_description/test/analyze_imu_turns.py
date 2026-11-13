#!/usr/bin/env python3
"""Compare wheel/IMU yaw with independent stationary-endpoint lidar alignment."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import minimize
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    data = {}
    while reader.has_next():
        topic, serialized, stamp = reader.read_next()
        data.setdefault(topic, []).append((stamp/1e9, deserialize_message(serialized, types[topic])))
    return data


def points(scans, start, end):
    clouds = []
    for stamp, scan in scans:
        if start <= stamp <= end:
            r = np.asarray(scan.ranges)
            a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
            valid = np.isfinite(r) & (r > .2) & (r < scan.range_max)
            clouds.append(np.c_[r[valid]*np.cos(a[valid]), r[valid]*np.sin(a[valid])])
    if not clouds:
        raise ValueError('no stationary scans in endpoint window')
    cloud = np.concatenate(clouds)
    _, indices = np.unique(np.round(cloud/.02), axis=0, return_index=True)
    return cloud[indices]


def align(a, b):
    # Align AFTER points into BEFORE coordinates; +theta is robot left turn.
    ta, tb = cKDTree(a), cKDTree(b)
    def score(p):
        x, y, theta = p
        c, s = math.cos(theta), math.sin(theta)
        rot = np.array([[c, -s], [s, c]])
        d1 = ta.query(b@rot.T + [x,y])[0]
        d2 = tb.query((a-[x,y])@rot)[0]
        # Trimming rejects disocclusion and moving objects symmetrically.
        return sum(np.mean(np.sort(d)[:int(.8*len(d))]**2) for d in (d1,d2))
    initial = []
    for theta in np.linspace(-math.pi, math.pi, 361):
        x, y = .0254*(math.cos(theta)-1), .0254*math.sin(theta)
        initial.append((score([x,y,theta]), [x,y,theta]))
    best = sorted(initial, key=lambda x:x[0])[:6]
    results = [minimize(score, p, method='Nelder-Mead',
                        options={'maxiter': 300, 'xatol': 1e-6}) for _,p in best]
    result = min(results, key=lambda r:r.fun)
    return {'yaw_deg': math.degrees(result.x[2]),
            'translation_m': result.x[:2].tolist(),
            'trimmed_rms_m': math.sqrt(result.fun/2),
            'points_before': len(a), 'points_after': len(b)}


def odom_yaw_at(data, t):
    stamps = np.array([s for s,_ in data])
    yaw = np.unwrap([2*math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)
                     for _,m in data])
    return float(np.interp(t, stamps, yaw))


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('run',type=Path)
    ap.add_argument('--swap-wheel-sides',action='store_true',help='normalize the pre-correction wheel baseline')
    args=ap.parse_args();data=read_bag(args.run/'bag')
    events=json.loads((args.run/'events.json').read_text())
    result={'turns': []}
    for event in events:
        start,end=event['start_ns']/1e9,event['end_ns']/1e9
        a=points(data['/scan'],start-1.4,start-.3)
        b=points(data['/scan'],end+1.2,end+2.5)
        aligned=align(a,b)
        t0,t1=start-.2,end+1.0
        wheel=math.degrees(odom_yaw_at(data['/odom/wheel'],t1)-odom_yaw_at(data['/odom/wheel'],t0))
        if args.swap_wheel_sides: wheel = -wheel
        fused=math.degrees(odom_yaw_at(data['/odom'],t1)-odom_yaw_at(data['/odom'],t0))
        gyro=[(t,m.angular_velocity.z) for t,m in data['/imu/data'] if t0<=t<=t1]
        gyro_yaw=math.degrees(np.trapz([v for _,v in gyro],[t for t,_ in gyro]))
        row={**event,'lidar':aligned,'wheel_deg':wheel,'fused_deg':fused,'gyro_integral_deg':gyro_yaw,
             'alignment_usable':aligned['trimmed_rms_m'] < .05,
             'wheel_error_deg':wheel-aligned['yaw_deg'],'fused_error_deg':fused-aligned['yaw_deg']}
        result['turns'].append(row)
        print(json.dumps(row),flush=True)
    start=events[0]['start_ns']/1e9 if events else float('inf')
    for topic in ['/imu/data_raw','/imu/data']:
        vals=np.array([[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]
                       for t,m in data[topic] if t<start-1])
        result[topic]={'stationary_mean_rad_s': vals.mean(axis=0).tolist(),
                       'stationary_std_rad_s':vals.std(axis=0).tolist()}
    usable=[x for x in result['turns'] if x['alignment_usable']]
    result['usable_alignment_count']=len(usable)
    result['mean_abs_wheel_yaw_error_deg']=(float(np.mean([abs(x['wheel_error_deg']) for x in usable])) if usable else None)
    result['mean_abs_fused_yaw_error_deg']=(float(np.mean([abs(x['fused_error_deg']) for x in usable])) if usable else None)
    (args.run/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='turns'},indent=2))


if __name__=='__main__': main()
