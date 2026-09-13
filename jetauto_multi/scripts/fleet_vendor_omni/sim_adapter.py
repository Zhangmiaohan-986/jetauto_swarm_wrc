#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Minimal simulated bottom adapter for the vendor-omni six-robot stack.

The real launch supplies the vendor driver/EKF/AMCL outputs.  This adapter
supplies the same topic and TF contract in simulation while the real
move_base+TEB and fleet supervisor remain unchanged.
"""

from __future__ import print_function

import math
import json
import os
import sys
import threading
import numpy as np
sys.path.insert(0,os.path.dirname(os.path.realpath(__file__)))
from fleet_geometry import MapGrid, distance

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse


def quaternion(yaw):
    return tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)


class VendorOmniSimAdapter(object):

    def __init__(self):
        self.lock = threading.RLock()
        self.map_frame = str(rospy.get_param("~map_frame", "robot_1/map"))
        self.robot_order = list(rospy.get_param(
            "~robot_order", ["robot_%d" % i for i in range(1, 7)]))
        self.time_scale = max(0.01, float(
            rospy.get_param("~time_scale", 1.0)))
        self.scan_rate = max(1.0, float(
            rospy.get_param("~scan_rate", 10.0)))
        self.scan_beams = max(9, int(
            rospy.get_param("~scan_beams", 181)))
        self.scan_range = max(1.0, float(
            rospy.get_param("~scan_range", 8.0)))
        configured = rospy.get_param("~robots", {})

        self.poses = {}
        self.commands = {}
        self.command_stamps = {}
        self.collisions = []
        self.collision_keys = set()
        self.collision_grid = None
        self.truth_pub = rospy.Publisher('~truth_json', String, queue_size=1)
        self.odom_pubs = {}
        self.raw_odom_pubs = {}
        self.imu_pubs = {}
        self.scan_pubs = {}
        self.raw_scan_pubs = {}
        self.amcl_pubs = {}
        self.last_scan = rospy.Time(0)
        self.map_grid = None
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber(rospy.get_param('~map_topic','/robot_1/map'), OccupancyGrid,
                         self.map_callback, queue_size=1)

        for name in self.robot_order:
            item = configured.get(name, {})
            self.poses[name] = [
                float(item.get("initial_x", 0.0)),
                float(item.get("initial_y", 0.0)),
                math.radians(float(item.get("initial_yaw_deg", 0.0))),
            ]
            self.commands[name] = [0.0, 0.0, 0.0]
            self.command_stamps[name] = rospy.Time(0)
            self.odom_pubs[name] = rospy.Publisher(
                "/%s/odom" % name, Odometry, queue_size=10)
            self.raw_odom_pubs[name] = rospy.Publisher(
                "/%s/odom_raw" % name, Odometry, queue_size=10)
            self.imu_pubs[name] = rospy.Publisher(
                "/%s/imu" % name, Imu, queue_size=10)
            self.scan_pubs[name] = rospy.Publisher(
                "/%s/scan" % name, LaserScan, queue_size=3)
            self.raw_scan_pubs[name] = rospy.Publisher(
                "/%s/scan_raw" % name, LaserScan, queue_size=3)
            self.amcl_pubs[name] = rospy.Publisher(
                "/%s/amcl_pose" % name,
                PoseWithCovarianceStamped, queue_size=3)
            rospy.Subscriber(
                "/%s/jetauto_controller/cmd_vel" % name,
                Twist, self.command_callback, callback_args=name,
                queue_size=10)
            rospy.Service(
                "/%s/request_nomotion_update" % name,
                Empty, self.nomotion_callback)

    def command_callback(self, msg, name):
        with self.lock:
            self.command_stamps[name] = rospy.Time.now()
            self.commands[name] = [
                float(msg.linear.x), float(msg.linear.y),
                float(msg.angular.z)]

    @staticmethod
    def nomotion_callback(_request):
        return EmptyResponse()

    def map_callback(self, msg):
        with self.lock:
            self.map_grid = msg
            self.grid_values = np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
            self.collision_grid = MapGrid(self.grid_values,msg.info.resolution,
                                           (msg.info.origin.position.x,msg.info.origin.position.y),.20)

    def raycast(self, x, y, yaw):
        """Return a map-backed scan; unknown cells are treated as clear."""
        grid = self.map_grid
        if grid is None or not grid.info.resolution:
            return [self.scan_range] * self.scan_beams
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        width = int(grid.info.width)
        height = int(grid.info.height)
        resolution = float(grid.info.resolution)
        angles=np.linspace(-math.pi,math.pi,self.scan_beams)+yaw
        ranges=np.arange(.10,self.scan_range+1e-6,max(.05,resolution))
        cx=np.floor((x+np.cos(angles)[:,None]*ranges[None,:]-origin_x)/resolution).astype(int)
        cy=np.floor((y+np.sin(angles)[:,None]*ranges[None,:]-origin_y)/resolution).astype(int)
        valid=(cx>=0)&(cy>=0)&(cx<width)&(cy<height)
        hits=~valid | (self.grid_values[np.clip(cy,0,height-1),np.clip(cx,0,width-1)]>=50)
        hits[:,-1]=True
        return ranges[np.argmax(hits,axis=1)].tolist()

    def step(self, dt):
        with self.lock:
            proposed={}
            now=rospy.Time.now()
            for name in self.robot_order:
                x, y, yaw = self.poses[name]
                if (now-self.command_stamps[name]).to_sec()>.5:
                    self.commands[name]=[0.,0.,0.]
                vx, vy, wz = self.commands[name]
                scale = self.time_scale
                x += (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt * scale
                y += (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt * scale
                yaw = math.atan2(math.sin(yaw + wz * dt * scale),
                                 math.cos(yaw + wz * dt * scale))
                proposed[name] = [x, y, yaw]
            blocked=set()
            for index,name in enumerate(self.robot_order):
                if self.collision_grid is not None and not self.collision_grid.free(proposed[name]):
                    blocked.add(name)
                    self.note_collision(name,'map')
                for other in self.robot_order[index+1:]:
                    if distance(proposed[name],proposed[other])<.40:
                        blocked.update((name,other)); self.note_collision(name,other)
            for name in self.robot_order:
                if name not in blocked: self.poses[name]=proposed[name]
                else: self.commands[name]=[0.,0.,0.]

    def note_collision(self,first,second):
        key=(first,second)
        if key not in self.collision_keys:
            self.collision_keys.add(key)
            self.collisions.append({'robots':[first,second],'time':rospy.Time.now().to_sec()})

    def publish(self, stamp):
        scan_due = (self.last_scan == rospy.Time(0)
                    or (stamp - self.last_scan).to_sec() >= 1.0 / self.scan_rate)
        if scan_due:
            self.last_scan = stamp
            self.truth_pub.publish(String(data=json.dumps({'time':stamp.to_sec(),
                'poses':self.poses,'collisions':self.collisions,'contact_model':'0.20m enclosing discs'})))
        with self.lock:
            for name in self.robot_order:
                x, y, yaw = self.poses[name]
                vx, vy, wz = self.commands[name]
                q = quaternion(yaw)
                odom_frame = "%s/odom" % name
                base_frame = "%s/base_footprint" % name

                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = odom_frame
                odom.child_frame_id = base_frame
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.orientation.x = q[0]
                odom.pose.pose.orientation.y = q[1]
                odom.pose.pose.orientation.z = q[2]
                odom.pose.pose.orientation.w = q[3]
                odom.twist.twist.linear.x = vx
                odom.twist.twist.linear.y = vy
                odom.twist.twist.angular.z = wz
                self.odom_pubs[name].publish(odom)
                self.raw_odom_pubs[name].publish(odom)

                imu = Imu()
                imu.header.stamp = stamp
                imu.header.frame_id = base_frame
                imu.orientation.x = q[0]
                imu.orientation.y = q[1]
                imu.orientation.z = q[2]
                imu.orientation.w = q[3]
                imu.angular_velocity.z = wz
                imu.linear_acceleration_covariance[0] = -1.0
                self.imu_pubs[name].publish(imu)

                amcl = PoseWithCovarianceStamped()
                amcl.header.stamp = stamp
                amcl.header.frame_id = self.map_frame
                amcl.pose.pose = odom.pose.pose
                amcl.pose.covariance[0] = 0.0001
                amcl.pose.covariance[7] = 0.0001
                amcl.pose.covariance[35] = 0.0001
                self.amcl_pubs[name].publish(amcl)

                if scan_due:
                    scan = LaserScan()
                    scan.header.stamp = stamp
                    scan.header.frame_id = base_frame
                    scan.angle_min = -math.pi
                    scan.angle_max = math.pi
                    scan.angle_increment = (2.0 * math.pi /
                                            float(self.scan_beams - 1))
                    scan.range_min = 0.10
                    scan.range_max = self.scan_range
                    scan.ranges = self.raycast(x, y, yaw)
                    self.scan_pubs[name].publish(scan)
                    self.raw_scan_pubs[name].publish(scan)

                # The simulated odom is expressed in map coordinates, so the
                # map->odom transform is identity and remains single-owner.
                self.tf_broadcaster.sendTransform(
                    (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), stamp,
                    odom_frame, self.map_frame)
                self.tf_broadcaster.sendTransform(
                    (x, y, 0.0), q, stamp, base_frame, odom_frame)

    def run(self):
        rate = rospy.Rate(30.0)
        previous = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = min(0.10, max(0.0, (now - previous).to_sec()))
            previous = now
            self.step(dt)
            self.publish(now)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("fleet_vendor_omni_sim_adapter")
    VendorOmniSimAdapter().run()
