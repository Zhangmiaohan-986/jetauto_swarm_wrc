#!/usr/bin/env python
"""Merge global or per-robot static TF for late VM subscribers."""
import re
import threading

import rospy
from tf2_msgs.msg import TFMessage


class TfIngress(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.static = {}
        robots = rospy.get_param('~robots', [])
        if (not isinstance(robots, list) or
                any(not isinstance(name, (str, type(u''))) or
                    not re.match(r'^robot_[1-9][0-9]*$', name) for name in robots) or
                len(set(robots)) != len(robots)):
            raise ValueError('~robots must be a list of unique robot_N names')
        topics = ['/%s/tf_static' % name for name in robots] if robots else ['/tf_static']
        if any(rospy.resolve_name(topic) != topic for topic in topics + ['/fleet/tf_static']):
            raise ValueError('TF ingress endpoints must not be remapped')
        # Retain each source's fixed links; a plain relay would latch only one.
        self.static_pub = rospy.Publisher('/fleet/tf_static', TFMessage,
                                          queue_size=1, latch=True)
        self.static_subs = [rospy.Subscriber(topic, TFMessage, self.static_cb,
                                            queue_size=100, buff_size=4194304,
                                            tcp_nodelay=True) for topic in topics]

    def static_cb(self, msg):
        # A latched publisher must retain ALL source publishers' fixed links,
        # not merely the last robot's message. Serialize concurrent callbacks.
        with self.lock:
            changed = False
            for transform in msg.transforms:
                key = transform.child_frame_id
                if self.static.get(key) != transform:
                    self.static[key] = transform
                    changed = True
            if changed:
                self.static_pub.publish(TFMessage(
                    transforms=[self.static[key] for key in sorted(self.static)]))


if __name__ == '__main__':
    rospy.init_node('fleet_tf_ingress')
    TfIngress()
    rospy.spin()
