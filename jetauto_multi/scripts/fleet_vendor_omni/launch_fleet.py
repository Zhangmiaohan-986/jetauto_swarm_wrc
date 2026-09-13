#!/usr/bin/env python
"""ROS entry point: assemble one profile without duplicated per-robot launches."""
from __future__ import print_function
import os
import sys
import tempfile
import fcntl
import hashlib
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import rospkg
import rospy
import roslaunch
from fleet_profile import (load_profile, build_launch, check_map_evidence,
                           filter_active_fleet)

def acquire_launch_lock():
    key = hashlib.sha256(os.environ.get('ROS_MASTER_URI',
        'http://localhost:11311').encode('utf-8')).hexdigest()[:16]
    handle = open(os.path.join(tempfile.gettempdir(),
        'jetauto_fleet_%s_%s.lock' % (os.getuid(), key)), 'a')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except IOError:
        handle.close()
        raise RuntimeError('Fleet upper launch already running for this ROS master; '
                           'stop the old launch with Ctrl+C before restarting.')
    return handle

def main():
    # Before init_node: duplicate registration would otherwise kill the old node.
    launch_lock = acquire_launch_lock()
    rospy.init_node('fleet_profile_launcher')
    root = rospkg.RosPack().get_path('jetauto_multi')
    profile, mission, roster = load_profile(root, rospy.get_param('~profile'))
    excluded = rospy.get_param('~excluded_robots', '')
    mission, roster = filter_active_fleet(mission, roster, excluded)
    for key in ('motion_enabled','allow_full_run'):
        mission[key] = bool(rospy.get_param('~'+key,False))
    override_map = rospy.get_param('~map_yaml','')
    if override_map:
        profile['map_yaml'] = override_map
        check_map_evidence(override_map,mission)
    simulation = bool(rospy.get_param('~simulation',False))
    if simulation and any(name.startswith('/robot_') for name, _kind in rospy.get_published_topics()):
        raise RuntimeError('Use an independent simulation ROS master')
    content = build_launch(root,profile,mission,roster,simulation,
                           bool(rospy.get_param('~gui',True)),bool(rospy.get_param('~record_data',True)))
    handle, path = tempfile.mkstemp(prefix='jetauto_fleet_',suffix='.launch')
    with os.fdopen(handle,'wb') as stream:
        stream.write(content)
    parent = roslaunch.parent.ROSLaunchParent(rospy.get_param('/run_id'), [path])
    try:
        parent.start()
        parent.spin()
    finally:
        parent.shutdown()
        os.unlink(path)
        launch_lock.close()

if __name__ == '__main__':
    main()
