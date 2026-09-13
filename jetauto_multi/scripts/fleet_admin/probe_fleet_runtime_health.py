#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""One-shot, read-only roster-driven ROS health sampler.

All scan/odom topics share one sampling window and all TF lookups share one
warm tf2 buffer.  This avoids false negatives caused by repeatedly starting
short-lived ``rostopic hz`` and ``tf_echo`` processes.
"""

from __future__ import print_function

import time
import sys
import re
import os

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


ROBOTS = tuple(sys.argv[1:])
if not ROBOTS or len(set(ROBOTS)) != len(ROBOTS) or any(not re.match(r'^robot_[1-9][0-9]*$',r) for r in ROBOTS):
    raise SystemExit('Pass the unique robot_N names from the runtime roster')
SAMPLE_SECONDS = 8.0


def main():
    samples = dict((name, {"scan": [], "odom": []}) for name in ROBOTS)

    def receive(_msg, callback_args):
        robot, stream = callback_args
        samples[robot][stream].append(time.time())

    rospy.init_node(
        "fleet_runtime_health_probe", anonymous=True, disable_signals=True
    )
    tf_buffer = tf2_ros.Buffer(rospy.Duration(20.0))
    subscribers = []
    local_tf = os.environ.get('FLEET_LOCAL_TF', 'false')
    if local_tf not in ('true', 'false'):
        raise ValueError('FLEET_LOCAL_TF must be true or false')
    if local_tf == 'true':
        def receive_tf(msg, is_static):
            authority = msg._connection_header.get('callerid', 'fleet_health_probe')
            setter = tf_buffer.set_transform_static if is_static else tf_buffer.set_transform
            for transform in msg.transforms:
                setter(transform, authority)
        for robot in ROBOTS:
            for suffix, fixed in (('tf', False), ('tf_static', True)):
                subscribers.append(rospy.Subscriber('/%s/%s' % (robot, suffix),
                    TFMessage, receive_tf, fixed, queue_size=100,
                    buff_size=1048576, tcp_nodelay=True))
    else:
        tf_listener = tf2_ros.TransformListener(tf_buffer)  # noqa: F841
    for robot in ROBOTS:
        subscribers.append(
            rospy.Subscriber(
                "/%s/scan_raw" % robot,
                LaserScan,
                receive,
                (robot, "scan"),
                queue_size=50,
            )
        )
        subscribers.append(
            rospy.Subscriber(
                "/%s/odom" % robot,
                Odometry,
                receive,
                (robot, "odom"),
                queue_size=100,
            )
        )

    deadline = time.time() + SAMPLE_SECONDS
    while time.time() < deadline and not rospy.is_shutdown():
        time.sleep(0.05)

    for robot in ROBOTS:
        fields = []
        for stream in ("scan", "odom"):
            stamps = samples[robot][stream]
            if len(stamps) > 1 and stamps[-1] > stamps[0]:
                fields.append("%.3f" % ((len(stamps) - 1) / (stamps[-1] - stamps[0])))
            elif stamps:
                fields.append("0.000")
            else:
                fields.append("NA")

        try:
            tf_buffer.lookup_transform(
                "%s/odom" % robot,
                "%s/base_footprint" % robot,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            tf_state = "PASS"
        except Exception:
            tf_state = "FAIL"

        print("%s|%s|%s|%s" % (robot, fields[0], fields[1], tf_state))


if __name__ == "__main__":
    main()
