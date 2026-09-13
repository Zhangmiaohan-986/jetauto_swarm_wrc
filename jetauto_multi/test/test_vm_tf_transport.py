#!/usr/bin/env python
"""One bounded ROS integration check on an isolated localhost master; no robots."""
from __future__ import print_function
import os
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11422'
os.environ['ROS_IP'] = '127.0.0.1'
os.environ['MACHINE_TYPE'] = 'JetAuto'
os.environ['LIDAR_TYPE'] = 'A1'
os.environ['DEPTH_CAMERA_TYPE'] = 'AstraProPlus'
os.environ.pop('ROS_HOSTNAME', None)
os.environ.pop('FLEET_RUNTIME_CONFIG', None)
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'scripts/fleet_vendor_omni'))
import rospy
import roslaunch
import rosgraph
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from fleet_profile import load_profile, build_launch
from launch_fleet import acquire_launch_lock


def wait_for(predicate, timeout=8):
    end = time.time()+timeout
    while time.time() < end:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError('bounded condition timed out')


def transform(parent, child, stamp):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.header.stamp = rospy.Time.from_sec(stamp)
    t.transform.translation.x = 1.25
    t.transform.rotation.w = 1.0
    return t


processes = []
with open(os.devnull, 'w') as sink:
    try:
        processes.append(subprocess.Popen(['roscore', '-p', '11422'], stdout=sink, stderr=sink))
        wait_for(rosgraph.is_master_online)
        held = acquire_launch_lock()
        try:
            acquire_launch_lock()
            raise AssertionError('duplicate launch accepted')
        except RuntimeError:
            pass
        held.close()
        acquire_launch_lock().close()

        for relative in ('six_robot_profile.yaml', 'profiles/fleet10_same_map/profile.yaml'):
            p, m, r = load_profile(ROOT, 'config/fleet_vendor_omni/'+relative)
            for sim in (False, True):
                content = build_launch(ROOT, p, m, r, sim)
                handle, path = tempfile.mkstemp(suffix='.launch')
                with os.fdopen(handle, 'wb') as f:
                    f.write(content)
                try:
                    cfg = roslaunch.config.load_config_default([path], 11422, verbose=False)
                finally:
                    os.unlink(path)
                relayed = bool(p.get('vm_tf_relay'))
                sensors = bool(p.get('vm_sensor_relay'))
                sensor_nodes = [n for n in cfg.nodes if n.type == 'fleet_sensor_ingress']
                assert len(sensor_nodes) == int(sensors and not sim)
                assert sum(n.type == 'tf_ingress.py' for n in cfg.nodes) == int(relayed and not sim)
                native = [n for n in cfg.nodes if n.package == 'topic_tools' and n.type == 'relay']
                assert len(native) == int(relayed and not sim and not p.get('per_robot_tf'))
                if native:
                    assert native[0].args == '/tf /fleet/tf'
                    assert native[0].required
                for n in cfg.nodes:
                    remaps = dict(n.remap_args)
                    for name in m['robot_order']:
                        for suffix in ('scan', 'odom', 'imu'):
                            topic = '/%s/%s' % (name, suffix)
                            if sensors:
                                assert remaps[topic] == (topic if n in sensor_nodes else '/fleet'+topic)
                            else:
                                assert topic not in remaps
                        assert '/%s/jetauto_controller/cmd_vel' % name not in remaps
                    if n.type == 'tf_ingress.py' or n in native:
                        assert remaps['/tf'] == '/tf' and remaps['/tf_static'] == '/tf_static'
                    elif relayed and n not in sensor_nodes:
                        assert remaps['/tf'] == '/fleet/tf'
                        assert remaps['/tf_static'] == '/fleet/tf_static'
                    else:
                        assert '/tf' not in remaps
                recorder = next(n for n in cfg.nodes if n.type == 'record.sh')
                assert dict(recorder.env_args)['FLEET_RECORD_BOTTOM_RAW'] == ('false' if sensors else 'true')

        rospy.init_node('isolated_tf_probe')
        dynamic, fixed = [], []
        dpub = rospy.Publisher('/tf', TFMessage, queue_size=10)
        spub = rospy.Publisher('/tf_static', TFMessage, queue_size=10)
        sub = rospy.Subscriber('/fleet/tf', TFMessage, dynamic.append)
        processes.append(subprocess.Popen([
            '/opt/ros/melodic/lib/topic_tools/relay', '/tf', '/fleet/tf',
            '__name:=fleet_tf_dynamic_relay'], stdout=sink, stderr=sink))
        processes.append(subprocess.Popen([sys.executable,
            os.path.join(ROOT, 'scripts/fleet_vendor_omni/tf_ingress.py')], stdout=sink, stderr=sink))
        wait_for(lambda: dpub.get_num_connections() == 1 and spub.get_num_connections() == 1)
        # topic_tools learns the message type from the first input before
        # advertising its output; this warmup is not the delivery assertion.
        dpub.publish(TFMessage([transform('probe/odom', 'probe/warmup', 1.0)]))
        wait_for(lambda: sub.get_num_connections() == 1)
        sent = TFMessage([transform('r1/odom', 'r1/base', 123.5)])
        dpub.publish(sent)
        wait_for(lambda: sent in dynamic)
        a, b = transform('r1/base', 'r1/lidar', 100), transform('r2/base', 'r2/lidar', 101)
        spub.publish(TFMessage([a]))
        spub.publish(TFMessage([b]))
        time.sleep(0.3)
        late = rospy.Subscriber('/fleet/tf_static', TFMessage, fixed.append)
        wait_for(lambda: fixed and len(fixed[-1].transforms) == 2)
        assert fixed[-1].transforms == [a, b]
        time.sleep(0.3)
        assert dynamic.count(sent) == 1, 'dynamic relay loop'
        print('PASS: 6/10 real/sim expanded remaps; native dynamic relay isolated; original TF preserved; '
              'late static subscriber gets both robots; no loop; duplicate launch rejected')
    finally:
        rospy.signal_shutdown('test complete')
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                end = time.time()+5
                while process.poll() is None and time.time() < end:
                    time.sleep(0.05)
                if process.poll() is None:
                    process.kill()
                process.wait()
