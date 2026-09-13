#!/usr/bin/env python
"""Bounded localhost ROS check; never uses the fleet master or real sensors."""
from __future__ import print_function
import os
import signal
import socket
import subprocess
import time

PORT = 11429
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % PORT
os.environ['ROS_IP'] = '127.0.0.1'
os.environ.pop('ROS_HOSTNAME', None)
import rosgraph
import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage


def wait_for(predicate, description, timeout=8.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return
        time.sleep(0.025)
    raise AssertionError(description)


def received(message, target):
    target.append(bytes(message._buff))


def transform(robot, child, stamp):
    result = TransformStamped()
    result.header.frame_id = robot + '/base_footprint'
    result.header.stamp = rospy.Time.from_sec(stamp)
    result.header.seq = 73
    result.child_frame_id = robot + '/' + child
    result.transform.translation.x = 0.125
    result.transform.rotation.w = 1.0
    return result


processes, publishers, subscribers = [], [], []
probe = socket.socket()
try:
    probe.settimeout(0.2)
    assert probe.connect_ex(('127.0.0.1', PORT)) != 0, 'isolated master port is occupied'
finally:
    probe.close()

with open(os.devnull, 'w') as sink:
    try:
        processes.append(subprocess.Popen(['roscore', '-p', str(PORT)], stdout=sink, stderr=sink))
        wait_for(rosgraph.is_master_online, 'isolated ROS master did not start')
        rospy.init_node('sensor_ingress_check', disable_signals=True)
        rospy.set_param('/fleet_sensor_ingress/robots', ['robot_8', 'robot_10'])
        rospy.set_param('/fleet_sensor_ingress/per_robot_tf', True)
        processes.append(subprocess.Popen(
            ['rosrun', 'jetauto_multi', 'fleet_sensor_ingress'], stdout=sink, stderr=sink))

        channels = []
        for robot in ('robot_8', 'robot_10'):
            for suffix, kind in (('scan', LaserScan), ('odom', Odometry), ('imu', Imu)):
                source = '/' + robot + '/' + suffix
                output = '/fleet' + source
                raw, first, second = [], [], []
                subscribers.append(rospy.Subscriber(source, rospy.AnyMsg, received,
                                                    callback_args=raw, queue_size=1))
                subscribers.append(rospy.Subscriber(output, rospy.AnyMsg, received,
                                                    callback_args=first, queue_size=1))
                subscribers.append(rospy.Subscriber(output, rospy.AnyMsg, received,
                                                    callback_args=second, queue_size=1))
                pub = rospy.Publisher(source, kind, queue_size=1)
                publishers.append(pub)
                channels.append((source, robot, suffix, pub, kind, raw, first, second))

        # Two source connections: one ingress, plus this wire-observation probe.
        wait_for(lambda: all(pub.get_num_connections() == 2 for pub in publishers),
                 'ingress must subscribe to each source exactly once')
        wait_for(lambda: all(sub.get_num_connections() == 1 for sub in subscribers),
                 'source and both VM-side callbacks did not connect')
        for source, robot, suffix, pub, kind, raw, first, second in channels:
            message = kind()
            message.header.stamp = rospy.Time(123, 456789)
            message.header.frame_id = robot + '/original_sensor_frame'
            if suffix == 'scan':
                message.angle_min = -1.1
                message.angle_increment = 0.01
                message.range_min, message.range_max = 0.04, 12.0
                message.ranges = [0.125, float('inf'), float('nan')]
                message.intensities = [1.0, 2.0, 3.0]
            elif suffix == 'odom':
                message.child_frame_id = robot + '/base_footprint'
                message.pose.pose.position.x = float('nan')
                message.pose.pose.position.y = -1.25
                message.pose.pose.orientation.w = 1.0
                message.pose.covariance[0] = 0.123
                message.twist.twist.angular.z = -0.07
            else:
                message.orientation.x = float('nan')
                message.orientation_covariance[0] = -1.0
                message.angular_velocity.z = 0.0123
                message.linear_acceleration.z = 9.81
            # Offset the test publisher's sequence so accidental roscpp seq
            # rewriting cannot pass merely because both publishers start at 0.
            pub.impl.seq = 40
            pub.publish(message)
            wait_for(lambda: len(raw) == len(first) == len(second) == 1,
                     'single message not delivered once to both consumers: ' + source)
            assert raw[0] == first[0] == second[0], 'wire values changed: ' + source

        counts = [(len(raw), len(first), len(second))
                  for _source, _robot, _suffix, _pub, _kind, raw, first, second in channels]
        time.sleep(0.25)
        assert counts == [(len(raw), len(first), len(second))
                          for _source, _robot, _suffix, _pub, _kind, raw, first, second in channels], \
            'extra output without new input (possible loop or heartbeat)'

        dynamic = []
        dynamic_sub = rospy.Subscriber('/fleet/tf', rospy.AnyMsg, received,
                                       callback_args=dynamic, queue_size=10)
        subscribers.append(dynamic_sub)
        for robot in ('robot_8', 'robot_10'):
            raw = []
            source = '/' + robot + '/tf'
            pub = rospy.Publisher(source, TFMessage, queue_size=1)
            publishers.append(pub)
            subscribers.append(rospy.Subscriber(source, rospy.AnyMsg, received,
                                                callback_args=raw, queue_size=1))
            wait_for(lambda: pub.get_num_connections() == 2 and
                     dynamic_sub.get_num_connections() == 1, 'per-robot TF not connected')
            t = transform(robot, 'dynamic_child', 123.456)
            if robot == 'robot_10':
                t.transform.translation.y = float('nan')
            count = len(dynamic)
            pub.publish(TFMessage([t, transform(robot, 'wheel_joint', 123.457)]))
            wait_for(lambda: len(raw) == 1 and len(dynamic) == count + 1,
                     'per-robot dynamic TF not forwarded')
            assert raw[0] == dynamic[-1], 'dynamic TF changed stamp/frame/values'
        time.sleep(0.25)
        assert len(dynamic) == 2, 'dynamic TF feedback loop'

        # Latched sources predate the merger; a late VM subscriber must receive
        # every fixed child, not merely the last vehicle's message.
        a = transform('robot_8', 'lidar_frame', 100)
        b = transform('robot_10', 'lidar_frame', 101)
        c = transform('robot_8', 'camera_frame', 102)
        static_pubs = []
        for robot, transforms in (('robot_8', [a, c]), ('robot_10', [b])):
            pub = rospy.Publisher('/' + robot + '/tf_static', TFMessage,
                                  queue_size=1, latch=True)
            static_pubs.append(pub)
            publishers.append(pub)
            pub.publish(TFMessage(transforms))
        rospy.set_param('/fleet_tf_ingress/robots', ['robot_8', 'robot_10'])
        static_process = subprocess.Popen(['rosrun', 'jetauto_multi', 'tf_ingress.py'],
                                          stdout=sink, stderr=sink)
        processes.append(static_process)
        wait_for(lambda: all(pub.get_num_connections() == 1 for pub in static_pubs),
                 'per-robot static TF merger not subscribed')
        fixed = []
        subscribers.append(rospy.Subscriber('/fleet/tf_static', TFMessage, fixed.append))
        wait_for(lambda: fixed and len(fixed[-1].transforms) == 3,
                 'late static subscriber missed fixed links')
        assert fixed[-1].transforms == sorted([a, b, c], key=lambda t: t.child_frame_id)

        # The omitted/empty roster retains the existing global-TF contract.
        static_process.send_signal(signal.SIGINT)
        wait_for(lambda: static_process.poll() is not None, 'static merger did not stop')
        global_pub = rospy.Publisher('/tf_static', TFMessage, queue_size=1, latch=True)
        publishers.append(global_pub)
        global_pub.publish(TFMessage([a]))
        processes.append(subprocess.Popen(['rosrun', 'jetauto_multi', 'tf_ingress.py',
                                          '__name:=global_static_ingress'],
                                          stdout=sink, stderr=sink))
        wait_for(lambda: global_pub.get_num_connections() == 1,
                 'default global static input not connected')
        fallback = []
        subscribers.append(rospy.Subscriber('/fleet/tf_static', TFMessage, fallback.append))
        wait_for(lambda: fallback and fallback[-1].transforms == [a],
                 'default global static TF not latched for late subscriber')

        for name, robots, remaps in (
                ('invalid_names', ['robot_8', 'robot_8'], []),
                ('invalid_remap', ['robot_8'], ['/robot_8/scan:=/fleet/robot_8/scan'])):
            rospy.set_param('/' + name + '/robots', robots)
            process = subprocess.Popen(['rosrun', 'jetauto_multi', 'fleet_sensor_ingress',
                                        '__name:=' + name] + remaps,
                                       stdout=sink, stderr=sink)
            processes.append(process)
            wait_for(lambda: process.poll() is not None, 'invalid ingress did not exit')
            assert process.returncode != 0, 'invalid configuration accepted'
        print('PASS: 2 robots x 3 sensor types; one source subscription, two consumers, '
              'byte-exact header/seq/stamp/frame/NaN preservation, merged per-robot dynamic TF, '
              'late static subscriber gets all links, default global static fallback, '
              'no loop, bad config rejected')
    finally:
        for sub in subscribers:
            sub.unregister()
        for pub in publishers:
            pub.unregister()
        if rospy.core.is_initialized():
            rospy.signal_shutdown('isolated check complete')
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                deadline = time.time() + 4.0
                while process.poll() is None and time.time() < deadline:
                    time.sleep(0.05)
                if process.poll() is None:
                    process.kill()
            process.wait()
