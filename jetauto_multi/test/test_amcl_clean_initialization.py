#!/usr/bin/env python
"""Native AMCL regression: poisoned saved params, actual launch, isolated master."""
from __future__ import print_function
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11423', ROS_IP='127.0.0.1',
                  MACHINE_TYPE='JetAuto', LIDAR_TYPE='A1', DEPTH_CAMERA_TYPE='AstraProPlus')
os.environ.pop('ROS_HOSTNAME', None)
os.environ.pop('FLEET_RUNTIME_CONFIG', None)
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'scripts/fleet_vendor_omni'))
import rosgraph
import roslaunch
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from fleet_profile import load_profile, build_launch


def finite(values):
    return all(not math.isnan(v) and not math.isinf(v) for v in values)


def tf(parent, child):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.header.stamp = rospy.Time.now()
    t.transform.rotation.w = 1.0
    return t


assert not rosgraph.is_master_online(), 'isolated test port already occupied'
processes = []
with open(os.devnull, 'w') as sink:
    try:
        processes.append(subprocess.Popen(['roscore', '-p', '11423'], stdout=sink, stderr=sink))
        end = time.time()+8
        while not rosgraph.is_master_online() and time.time()<end:
            time.sleep(.1)
        assert rosgraph.is_master_online()
        rospy.init_node('isolated_amcl_initialization_probe')
        p,m,r = load_profile(ROOT,'config/fleet_vendor_omni/profiles/fleet10_same_map/profile.yaml')
        fd,path = tempfile.mkstemp(suffix='.launch')
        with os.fdopen(fd,'wb') as f:
            f.write(build_launch(ROOT,p,m,r,False,False,False))
        try:
            cfg=roslaunch.config.load_config_default([path],11423,verbose=False)
        finally:
            os.unlink(path)
        for name in m['robot_order']:
            prefix='/'+name+'/amcl/'
            for key in ('initial_pose_x','initial_pose_y','initial_pose_a','initial_cov_xx','initial_cov_yy','initial_cov_aa'):
                rospy.set_param(prefix+key, float('nan') if 'pose' in key else -1e-14)
            for key,param in cfg.params.items():
                if key.startswith(prefix):
                    rospy.set_param(key,param.value)
            for key in ('initial_pose_x','initial_pose_y','initial_pose_a'):
                assert finite([rospy.get_param(prefix+key)])
            for key in ('initial_cov_xx','initial_cov_yy','initial_cov_aa'):
                value=rospy.get_param(prefix+key)
                assert finite([value]) and value>0

        poses, transforms = [], []
        ps=rospy.Subscriber('/robot_9/amcl_pose',PoseWithCovarianceStamped,poses.append)
        ts=rospy.Subscriber('/fleet/tf',TFMessage,transforms.append)
        mp=rospy.Publisher('/robot_1/map',OccupancyGrid,queue_size=1,latch=True)
        dp=rospy.Publisher('/fleet/tf',TFMessage,queue_size=10)
        sp=rospy.Publisher('/fleet/tf_static',TFMessage,queue_size=1,latch=True)
        lp=rospy.Publisher('/robot_9/amcl_scan',LaserScan,queue_size=1)
        grid=OccupancyGrid();grid.header.frame_id='robot_1/map'
        grid.info.resolution=.1;grid.info.width=grid.info.height=100
        grid.info.origin.position.x=grid.info.origin.position.y=-5
        grid.info.origin.orientation.w=1;grid.data=[0]*10000
        mp.publish(grid)
        sp.publish(TFMessage([tf('robot_9/base_footprint','robot_9/lidar_frame')]))
        node=next(n for n in cfg.nodes if n.namespace=='/robot_9/' and n.type=='amcl')
        args=['/opt/ros/melodic/lib/amcl/amcl','__name:=amcl','__ns:=/robot_9']
        args.extend(a+':='+b for a,b in node.remap_args)
        processes.append(subprocess.Popen(args,stdout=sink,stderr=sink))
        scan=LaserScan();scan.header.frame_id='robot_9/lidar_frame'
        scan.angle_min=-math.pi;scan.angle_max=math.pi;scan.angle_increment=math.pi/90
        scan.range_min=.1;scan.range_max=6.;scan.ranges=[float('inf')]*181
        began=time.time()
        end=began+12
        while time.time()<end:
            dp.publish(TFMessage([tf('robot_9/odom','robot_9/base_footprint')]))
            scan.header.stamp=rospy.Time.now()-rospy.Duration(.1)
            lp.publish(scan)
            if time.time()-began>=4 and poses and any(t.child_frame_id=='robot_9/odom' for msg in transforms for t in msg.transforms):
                break
            time.sleep(.1)
        assert poses, 'native AMCL did not publish a pose'
        for pose in poses:
            v=pose.pose.pose
            assert finite([v.position.x,v.position.y,v.orientation.z,v.orientation.w]+list(pose.pose.covariance))
        map_tf=[t for msg in transforms for t in msg.transforms if t.child_frame_id=='robot_9/odom']
        assert map_tf, 'native AMCL did not publish map->odom'
        for t in map_tf:
            a=t.transform.translation;q=t.transform.rotation
            assert finite([a.x,a.y,a.z,q.x,q.y,q.z,q.w])
        assert rospy.get_param('/robot_9/amcl/save_pose_rate') < 0
        for key in ('initial_pose_x','initial_pose_y','initial_pose_a','initial_cov_xx','initial_cov_yy','initial_cov_aa'):
            path='/robot_9/amcl/'+key
            assert rospy.get_param(path)==cfg.params[path].value, 'periodic pose cache changed '+key
        print('PASS: ten initial states reset; native AMCL/TF finite; periodic pose caching disabled')
    finally:
        rospy.signal_shutdown('test complete')
        for proc in reversed(processes):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                end=time.time()+5
                while proc.poll() is None and time.time()<end: time.sleep(.05)
                if proc.poll() is None: proc.kill()
                proc.wait()
