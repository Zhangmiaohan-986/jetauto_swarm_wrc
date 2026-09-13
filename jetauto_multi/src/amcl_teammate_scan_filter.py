#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Remove known teammate body returns from localization/navigation scans."""

import copy
import json
import math
import time
import threading

import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class AmclTeammateScanFilter(object):

    def __init__(self):
        rospy.init_node("amcl_teammate_scan_filter")

        self.robot_id = int(rospy.get_param("~robot_id"))
        self.robot_count = int(rospy.get_param("~robot_count", 3))
        self.robot_prefix = str(rospy.get_param("~robot_prefix", "robot_"))
        self.base_suffix = str(rospy.get_param("~base_frame_suffix", "base_footprint"))
        self.input_topic = str(rospy.get_param("~input_topic", "scan_raw"))
        self.output_topic = str(rospy.get_param("~output_topic", "amcl_scan"))
        self.body_radius = float(rospy.get_param("~body_radius", 0.24))
        self.range_margin = float(rospy.get_param("~range_margin", 0.08))
        self.angle_margin = math.radians(
            float(rospy.get_param("~angle_margin_deg", 2.0))
        )
        self.use_scan_stamp = bool(rospy.get_param("~use_scan_stamp", False))
        self.max_output_rate = float(rospy.get_param("~max_output_rate", 5.0))
        self.max_scan_age = float(rospy.get_param("~max_scan_age", 0.8))
        if not (0.0 < self.max_output_rate <= 100.0 and
                0.0 < self.max_scan_age <= 10.0):
            raise ValueError("max_output_rate must be in (0,100], max_scan_age in (0,10]")
        self.scan_lock = threading.Lock()
        self.latest_scan = None
        self.received = 0
        self.overwritten = 0
        self.dropped = 0

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher(
            self.output_topic,
            LaserScan,
            queue_size=1,
            tcp_nodelay=True
        )
        self.health_pub = rospy.Publisher(
            "~health_json", String, queue_size=1
        )
        self.sub = rospy.Subscriber(
            self.input_topic,
            LaserScan,
            self.scan_cb,
            queue_size=1,
            buff_size=1048576,
            tcp_nodelay=True
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.max_output_rate),
                                 self.process_latest, reset=True)

        rospy.loginfo(
            "AMCL teammate scan filter robot_%d: %s -> %s",
            self.robot_id,
            self.input_topic,
            self.output_topic
        )

    def base_frame(self, robot_id):
        return "%s%d/%s" % (
            self.robot_prefix,
            robot_id,
            self.base_suffix
        )

    @staticmethod
    def normalize_angle(value):
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    def teammate_regions(self, scan_frame, lookup_time):
        regions = []
        missing = []

        for other_id in range(1, self.robot_count + 1):
            if other_id == self.robot_id:
                continue

            try:
                transform = self.tf_buffer.lookup_transform(
                    scan_frame,
                    self.base_frame(other_id),
                    lookup_time,
                    rospy.Duration(0.0)
                )
            except Exception:
                missing.append(other_id)
                continue

            x_pos = transform.transform.translation.x
            y_pos = transform.transform.translation.y
            distance = math.hypot(x_pos, y_pos)

            if distance <= self.body_radius:
                continue

            center_angle = math.atan2(y_pos, x_pos)
            ratio = min(1.0, (self.body_radius + self.range_margin) / distance)
            half_angle = math.asin(ratio) + self.angle_margin
            min_range = max(0.0, distance - self.body_radius - self.range_margin)
            max_range = distance + self.body_radius + self.range_margin
            regions.append((center_angle, half_angle, min_range, max_range))

        return regions, missing

    def scan_cb(self, msg):
        # Never perform TF or filtering on the TCP receive thread.
        with self.scan_lock:
            self.received += 1
            if self.latest_scan is not None:
                self.overwritten += 1
            self.latest_scan = msg

    def stale_reason(self, msg):
        stamp = msg.header.stamp.to_sec()
        age = rospy.Time.now().to_sec() - stamp
        if stamp <= 0.0 or math.isnan(age) or math.isinf(age):
            return "INVALID_STAMP"
        if age < -0.1:
            return "FUTURE_STAMP"
        if age > self.max_scan_age:
            return "STALE_SCAN"
        return None

    def process_latest(self, _event):
        with self.scan_lock:
            msg = self.latest_scan
            self.latest_scan = None
        if msg is None:
            return
        started = time.time()
        reason = self.stale_reason(msg)
        if reason:
            self.dropped += 1
            self.publish_health(msg, 0, 0, [], reason, started)
            return

        scan_frame = msg.header.frame_id
        lookup_time = msg.header.stamp if (
            self.use_scan_stamp and msg.header.stamp != rospy.Time(0)
        ) else rospy.Time(0)
        regions, missing = self.teammate_regions(scan_frame, lookup_time)

        filtered = copy.deepcopy(msg) if regions else msg
        ranges = list(filtered.ranges)
        masked = 0

        for index, measured_range in enumerate(ranges if regions else ()):
            if math.isinf(measured_range) or math.isnan(measured_range):
                continue

            beam_angle = msg.angle_min + index * msg.angle_increment

            for center_angle, half_angle, min_range, max_range in regions:
                angle_error = abs(
                    self.normalize_angle(beam_angle - center_angle)
                )

                if (
                    angle_error <= half_angle
                    and
                    min_range <= measured_range <= max_range
                ):
                    ranges[index] = float("inf")
                    masked += 1
                    break

        if regions:
            filtered.ranges = ranges
        # Processing/scheduling must not turn an initially fresh scan into old output.
        reason = self.stale_reason(msg)
        if reason:
            self.dropped += 1
            self.publish_health(msg, len(regions), masked, missing, reason, started)
            return
        self.pub.publish(filtered)
        self.publish_health(msg, len(regions), masked, missing,
                            "FILTERED" if regions else "PASSTHROUGH", started)

        rospy.loginfo_throttle(
            5.0,
            "robot_%d AMCL scan masked %d teammate beams",
            self.robot_id,
            masked
        )

    def publish_health(self, msg, region_count, masked, missing, status, started):
        value = {
            "robot_id": self.robot_id,
            "scan_stamp": msg.header.stamp.to_sec(),
            "scan_frame": msg.header.frame_id,
            "lookup_mode": "scan_stamp" if self.use_scan_stamp else "latest",
            "region_count": region_count,
            "masked_beams": masked,
            "missing_teammates": missing,
            "ros_time": rospy.Time.now().to_sec(),
            "status": status,
            "scan_age": rospy.Time.now().to_sec() - msg.header.stamp.to_sec(),
            "processing_ms": (time.time() - started) * 1000.0,
            "received": self.received,
            "overwritten": self.overwritten,
            "dropped": self.dropped,
        }
        self.health_pub.publish(String(data=json.dumps(value, sort_keys=True)))


if __name__ == "__main__":
    try:
        AmclTeammateScanFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
