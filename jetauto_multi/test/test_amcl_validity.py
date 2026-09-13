#!/usr/bin/env python
"""ROS-free checks of actual supervisor methods; Python 2.7/3, no robot access."""
from __future__ import print_function
import ast
import json
import math
import os
import threading
import time


class Value(object):
    def __init__(self, **values):
        self.__dict__.update(values)


def stamp(value):
    return Value(to_sec=lambda: value)


path = os.path.join(os.path.dirname(__file__), '..', 'scripts',
                    'fleet_vendor_omni', 'supervisor.py')
with open(path) as stream:
    tree = ast.parse(stream.read(), filename=path)
supervisor = next(node for node in tree.body if isinstance(node, ast.ClassDef))
methods = ('amcl_message_error', 'amcl_callback', 'yaw_from_quaternion',
           'wrap_angle', 'preflight', 'cached_preflight_errors', 'state_value',
           'amcl_timeout_reason', 'collect_amcl_samples', 'verify_samples', 'robot_pose')
supervisor.body = [node for node in supervisor.body
                   if isinstance(node, ast.FunctionDef) and node.name in methods]
tree.body = [supervisor]
namespace = dict(math=math, time=time, threading=threading,
                 rospy=Value(Duration=float, Time=Value(now=lambda: stamp(time.time()))))
exec(compile(tree, path, 'exec'), namespace)
Controller = namespace['FleetVendorOmniSupervisor']


def message():
    return Value(header=Value(stamp=stamp(time.time())), pose=Value(
        pose=Value(position=Value(x=0.0, y=0.0, z=0.0),
                   orientation=Value(x=0.0, y=0.0, z=0.0, w=1.0)),
        covariance=[0.0] * 36))


c = Controller()
c.lock = threading.RLock()
c.condition = threading.Condition(c.lock)
c.latest = {'robot_1': {}}
c.robot_order = ['robot_1']
c.map_frame = 'robot_1/map'
c.clients = {'robot_1': Value(wait_for_server=lambda _timeout: True)}
c.graph_state = lambda: ({'/robot_1/map': ['/map_server'],
                         '/robot_1/jetauto_controller/cmd_vel': ['/robot_1/move_base']},
                        {'/robot_1/request_nomotion_update': ['/robot_1/amcl']})
c.lookup_ready = lambda _target, _source: True
c.sensor_max_age = 1.0
c.stage_index = 0
c.stage_motion_complete = False
c.state = 'READY'
c.active = c.motion_enabled = c.allow_full_run = False
c.stages = []
c.last_error = None
c.checkpoint_state = {}
c.expected = {'robot_1': dict(x=0.0, y=0.0, yaw=0.0)}
c.initial_position_error = c.initial_yaw_error = 0.1
c.last_preflight = []
c.preflight_checked_wall = time.time()
c.preflight_cache_max_age = 2.5
for key in ('odom', 'imu', 'scan'):
    c.latest['robot_1'][key] = dict(receive_wall=time.time(), frame='robot_1/lidar_frame')

good = message()
good.pose.covariance[0] = -1.3487475025719675e-16
c.amcl_callback(good, 'robot_1')
assert c.preflight() == [], c.preflight()
assert c.latest['robot_1']['amcl']['cov_x'] == good.pose.covariance[0]

bad_messages = []
bad = message()
bad.pose.pose.position.x = float('nan')
bad_messages.append(bad)
bad = message()
bad.pose.covariance[1] = float('inf')
bad_messages.append(bad)
bad = message()
bad.pose.pose.orientation.w = 0.0
bad_messages.append(bad)
bad = message()
bad.pose.pose.orientation.w = 2.0
bad_messages.append(bad)
bad = message()
bad.pose.covariance[35] = -0.001
bad_messages.append(bad)
for bad in bad_messages:
    c.amcl_callback(message(), 'robot_1')
    c.amcl_callback(bad, 'robot_1')
    assert 'amcl' not in c.latest['robot_1'], 'invalid frame retained old good pose'
    assert any('invalid AMCL' in error for error in c.preflight())
    assert any('invalid AMCL' in error for error in c.cached_preflight_errors())
    assert c.amcl_timeout_reason('robot_1', 'OK') == 'AMCL_INVALID'
    assert c.state_value()['robots']['robot_1']['amcl_invalid']
    json.dumps(c.state_value(), allow_nan=False)
    try:
        c.robot_pose('robot_1')
        raise AssertionError('old cached TF accepted after invalid AMCL')
    except ValueError:
        pass

c.amcl_callback(message(), 'robot_1')
assert 'amcl_invalid' not in c.latest['robot_1']
assert c.preflight() == [] and c.cached_preflight_errors() == []

# One good checkpoint sample followed by an invalid update must not pass the
# minimum-one-sample gate using the previously collected good sample.
c.stop_requested = threading.Event()
c.checkpoint_timeout = 1.0
c.nomotion_max_requests = c.samples_preferred = 2
c.samples_required = 1
c.nomotion_service_timeout = 0.1
c.amcl_callback_timeout = 0.01
c.nomotion_request_interval = 0.0
c.publish_event = lambda *args, **kwargs: None
requests = []


def update(_name):
    requests.append(1)
    sample = message()
    if len(requests) == 2:
        sample.pose.pose.position.y = float('nan')
    c.amcl_callback(sample, 'robot_1')
    return 'OK'


c.call_nomotion = update
ok, reason, samples, detail = c.collect_amcl_samples(['robot_1'])
assert len(samples['robot_1']) == 1
assert not ok and reason == 'AMCL_INVALID' and detail['invalid']['robot_1']
c.sample_position_spread = c.sample_yaw_spread = 0.1
c.expected_position_error = c.expected_yaw_error = 0.1
c.tf_settle_time = 0.0
ok, reason, metrics = c.verify_samples(samples, c.expected, ['robot_1'])
assert not ok and 'invalid AMCL' in metrics['robot_1']['tf_error']
print('PASS: AMCL finite/norm/covariance rejection, roundoff tolerance, preflight, '
      'strict JSON, recovery and checkpoint invalidation')
