#!/usr/bin/env python
"""Bounded, ROS-free tests of latest-only scan processing (Python 2/3)."""
from __future__ import print_function
import json
import os
import sys
import types
import unittest


class Stamp(object):
    now_value = 100.0

    def __init__(self, value=0):
        self.value = value

    def to_sec(self):
        return self.value

    @classmethod
    def now(cls):
        return cls(cls.now_value)


class Publisher(object):
    def __init__(self, topic, msg_type, queue_size, tcp_nodelay=False):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class String(object):
    def __init__(self, data):
        self.data = data


class Buffer(object):
    def __init__(self, cache_time):
        self.calls = []

    def lookup_transform(self, target, source, stamp, timeout):
        self.calls.append(timeout)
        assert timeout.to_sec() == 0, 'TF must never wait'
        raise RuntimeError('No cross-robot TF yet')


def load_filter():
    names = ['rospy', 'tf2_ros', 'sensor_msgs', 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg']
    old = dict((name, sys.modules.get(name)) for name in names)
    for name in names:
        sys.modules[name] = types.ModuleType(name)
    ros = sys.modules['rospy']
    ros.init_node = lambda *args: None
    ros.get_param = lambda name, default=None: {'~robot_id': 1, '~robot_count': 10}.get(name, default)
    ros.Time = ros.Duration = Stamp
    ros.Publisher = Publisher
    ros.Subscriber = lambda topic, typ, cb, queue_size, buff_size, tcp_nodelay: None
    ros.Timer = lambda period, cb, reset: None
    ros.loginfo = ros.loginfo_throttle = lambda *args: None
    sys.modules['tf2_ros'].Buffer = Buffer
    sys.modules['tf2_ros'].TransformListener = lambda buffer: None
    sys.modules['sensor_msgs.msg'].LaserScan = object
    sys.modules['std_msgs.msg'].String = String
    path = os.path.join(os.path.dirname(__file__), '../src/amcl_teammate_scan_filter.py')
    module = types.ModuleType('scan_filter_under_test')
    try:
        with open(path) as stream:
            code = compile(stream.read(), path, 'exec')
        exec(code, module.__dict__)
    finally:
        for name, previous in old.items():
            if previous is None:
                del sys.modules[name]
            else:
                sys.modules[name] = previous
    return module


module = load_filter()


def scan(stamp=99.9):
    msg = type('Scan', (object,), {})()
    msg.header = type('Header', (object,), {})()
    msg.header.stamp = Stamp(stamp)
    msg.header.frame_id = 'robot_1/lidar_frame'
    msg.ranges = [1.0, 2.0]
    msg.angle_min = 0.0
    msg.angle_increment = 1.0
    return msg


class LatestScanTest(unittest.TestCase):
    def setUp(self):
        Stamp.now_value = 100.0
        self.node = module.AmclTeammateScanFilter()

    def health(self):
        return json.loads(self.node.health_pub.messages[-1].data)

    def test_latest_only_and_no_duplicate(self):
        for i in range(100):
            latest = scan(99.0 + i * .01)
            self.node.scan_cb(latest)
        self.assertEqual(self.node.tf_buffer.calls, [])
        self.node.process_latest(None)
        self.assertIs(self.node.pub.messages[0], latest)
        self.assertEqual(len(self.node.tf_buffer.calls), 9)
        self.assertEqual(self.node.overwritten, 99)
        self.assertEqual(self.health()['status'], 'PASSTHROUGH')
        self.node.process_latest(None)
        self.assertEqual(len(self.node.pub.messages), 1)

    def test_old_future_and_zero_stamp(self):
        for stamp, status in [(1.0, 'STALE_SCAN'), (101.0, 'FUTURE_STAMP'), (0.0, 'INVALID_STAMP')]:
            self.node.scan_cb(scan(stamp))
            self.node.process_latest(None)
            self.assertEqual(self.health()['status'], status)
        self.assertEqual(self.node.pub.messages, [])
        self.assertEqual(self.node.tf_buffer.calls, [])

    def test_filter_preserves_input_and_header(self):
        self.node.teammate_regions = lambda *args: ([(0., .1, .8, 1.2)], [])
        original = scan()
        self.node.scan_cb(original)
        self.node.process_latest(None)
        out = self.node.pub.messages[0]
        self.assertEqual(out.ranges, [float('inf'), 2.0])
        self.assertEqual(original.ranges, [1.0, 2.0])
        self.assertEqual(out.header.stamp.to_sec(), 99.9)
        self.assertEqual(self.health()['status'], 'FILTERED')

    def test_expired_during_processing(self):
        def delayed(*args):
            Stamp.now_value = 102.0
            return [], []
        self.node.teammate_regions = delayed
        self.node.scan_cb(scan())
        self.node.process_latest(None)
        self.assertEqual(self.node.pub.messages, [])
        self.assertEqual(self.health()['status'], 'STALE_SCAN')

    def test_new_arrival_during_processing_is_retained(self):
        newer = scan(100.0)
        def arrival(*args):
            self.node.scan_cb(newer)
            return [], []
        self.node.teammate_regions = arrival
        self.node.scan_cb(scan())
        self.node.process_latest(None)
        self.assertIs(self.node.latest_scan, newer)

    def test_invalid_rate(self):
        original = module.rospy.get_param
        module.rospy.get_param = lambda name, default=None: 0 if name == '~max_output_rate' else original(name, default)
        try:
            with self.assertRaises(ValueError):
                module.AmclTeammateScanFilter()
        finally:
            module.rospy.get_param = original


if __name__ == '__main__':
    unittest.main()
