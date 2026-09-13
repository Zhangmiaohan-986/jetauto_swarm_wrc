#!/usr/bin/env python3

from __future__ import print_function

import ast
import math
import os
import re
import unittest
import xml.etree.ElementTree as ET

import yaml
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts', 'fleet_vendor_omni'))
from fleet_profile import load_profile, build_launch


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read(relative_path):
    with open(os.path.join(ROOT, relative_path), "r") as stream:
        return stream.read()


class SixRobotVendorOmniContractTest(unittest.TestCase):

    def setUp(self):
        self.config = yaml.safe_load(read(
            "config/fleet_vendor_omni/six_robot_demo.yaml"))

    def test_launch_reuses_one_navigation_include_for_six_robots(self):
        profile, mission, roster = load_profile(ROOT, "config/fleet_vendor_omni/six_robot_profile.yaml")
        root = ET.fromstring(build_launch(ROOT, profile, mission, roster))
        includes = root.findall("include")
        self.assertEqual(6, len(includes))
        for robot_id, include in enumerate(includes, 1):
            args = {item.attrib["name"]: item.attrib["value"] for item in include.findall("arg")}
            self.assertEqual("robot_%d" % robot_id, args["robot_name"])
            self.assertEqual("6", args["robot_count"])
        public = ET.parse(os.path.join(ROOT, "launch/real.launch")).getroot()
        args = {item.attrib["name"]: item.attrib.get("default") for item in public.findall("arg")}
        self.assertEqual("false", args["motion_enabled"])
        self.assertEqual("false", args["allow_full_run"])

    def test_one_runtime_table_drives_bottom_and_upper_contracts(self):
        table = read("config/fleet_vendor_omni/six_robot_runtime.bash")
        rows = re.findall(r'^\s*"(robot_[1-6]\|[^\"]+)"\s*$', table,
                          flags=re.MULTILINE)
        self.assertEqual(6, len(rows))
        expected_ips = ["192.168.1.111", "192.168.1.112",
                        "192.168.1.113", "192.168.1.117",
                        "192.168.1.115", "192.168.1.116"]
        expected_factory = ["robot_2", "robot_3", "robot_4",
                            "robot_8", "robot_6", "robot_7"]
        for index, row in enumerate(rows):
            values = row.split("|")
            self.assertEqual(16, len(values))
            self.assertEqual("robot_%d" % (index + 1), values[0])
            self.assertEqual(expected_ips[index], values[1])
            self.assertEqual(expected_factory[index], values[2])
            self.assertEqual(["false", "false"], values[3:5])
            self.assertEqual(["0.10", "0.10", "0.10", "0.08"],
                             values[5:9])
            self.assertEqual(["0.0", "0.0"], values[14:16])

        self.assertEqual(0.08, self.config["formation_line"]["speed"])
        self.assertIn("FLEET_FORMATION_SPEED=0.08", table)

        bottom = read("scripts/fleet_admin/start_fleet_bottom.sh")
        checker = read("scripts/fleet_admin/check_fleet_bottom.sh")
        self.assertIn("FLEET_RESTART_EXISTING", bottom)
        self.assertIn("dynamic IMU bias=$actual_bias", checker)

    def test_stage_order_avoids_known_crossing_routes(self):
        self.assertEqual([
            "ROW_FORWARD", "COLUMN_R4_CLEAR", "COLUMN_R1_CLEAR",
            "COLUMN_FRONT_PAIR", "COLUMN_R1_ALIGN", "COLUMN_R4_ALIGN",
            "COLUMN_REAR_PAIR", "COLUMN_FORWARD",
            "TRIANGLE_LEADERS_OUT",
            "TRIANGLE_OUTER_FOLLOWERS", "TRIANGLE_INNER_FOLLOWERS",
            "TRIANGLE_FORWARD",
        ], [stage["name"] for stage in self.config["stages"]])

        stages = {stage["name"]: stage for stage in self.config["stages"]}
        self.assertEqual(set(self.config["robot_order"]), set(
            stages["ROW_FORWARD"]["goals"].keys()))
        self.assertEqual(["robot_4"], list(
            stages["COLUMN_R4_CLEAR"]["goals"].keys()))
        self.assertEqual({"robot_1", "robot_4"}, set(
            stages["TRIANGLE_OUTER_FOLLOWERS"]["goals"].keys()))
        self.assertEqual({"robot_5", "robot_6"}, set(
            stages["TRIANGLE_INNER_FOLLOWERS"]["goals"].keys()))
        for name in ("ROW_FORWARD", "COLUMN_FORWARD", "TRIANGLE_FORWARD"):
            self.assertEqual("formation_line", stages[name]["mode"])
            self.assertEqual(set(self.config["robot_order"]), set(
                stages[name]["goals"].keys()))

    def test_stage_goals_are_not_occupied_by_teammates_at_release(self):
        minimum = self.config["checkpoint"]["goal_teammate_min_distance"]
        self.assertEqual(0.50, minimum)
        poses = dict((name, (robot["initial_x"], robot["initial_y"]))
                     for name, robot in self.config["robots"].items())
        for stage in self.config["stages"]:
            for name, goal in stage["goals"].items():
                for other_name, other_pose in poses.items():
                    if name == other_name:
                        continue
                    if other_name in stage["goals"]:
                        other_goal = stage["goals"][other_name]
                        other_pose = (other_goal["x"], other_goal["y"])
                    distance = math.hypot(
                        goal["x"] - other_pose[0], goal["y"] - other_pose[1])
                    self.assertGreaterEqual(
                        distance, minimum,
                        "%s goal for %s is occupied by %s (%.3f m)" % (
                            stage["name"], name, other_name, distance))
            for name, goal in stage["goals"].items():
                poses[name] = (goal["x"], goal["y"])

    def test_every_stage_endpoint_keeps_rectangular_bodies_disjoint(self):
        poses = dict((name, (robot["initial_x"], robot["initial_y"]))
                     for name, robot in self.config["robots"].items())
        for stage in self.config["stages"]:
            for name, goal in stage["goals"].items():
                poses[name] = (goal["x"], goal["y"])
            names = sorted(poses)
            for index, first in enumerate(names):
                for second in names[index + 1:]:
                    dx = abs(poses[first][0] - poses[second][0])
                    dy = abs(poses[first][1] - poses[second][1])
                    self.assertTrue(dx >= 0.30 or dy >= 0.255,
                                    "%s overlaps %s/%s" % (
                                        stage["name"], first, second))

    def test_straight_line_stage_sweeps_keep_bodies_disjoint(self):
        poses = dict((name, (robot["initial_x"], robot["initial_y"]))
                     for name, robot in self.config["robots"].items())
        for stage in self.config["stages"]:
            starts = dict(poses)
            for sample_index in range(101):
                ratio = sample_index / 100.0
                sample = dict(poses)
                for name, goal in stage["goals"].items():
                    start_x, start_y = starts[name]
                    sample[name] = (
                        start_x + ratio * (goal["x"] - start_x),
                        start_y + ratio * (goal["y"] - start_y),
                    )
                names = sorted(sample)
                for index, first in enumerate(names):
                    for second in names[index + 1:]:
                        dx = abs(sample[first][0] - sample[second][0])
                        dy = abs(sample[first][1] - sample[second][1])
                        self.assertTrue(dx >= 0.30 or dy >= 0.255,
                                        "%s sweep overlaps %s/%s" % (
                                            stage["name"], first, second))
            for name, goal in stage["goals"].items():
                poses[name] = (goal["x"], goal["y"])

    def test_six_robot_checkpoint_scopes_amcl_to_moving_robots(self):
        source = read("scripts/fleet_vendor_omni/supervisor.py")
        self.assertIn("def collect_amcl_samples(self, robot_names):", source)
        self.assertIn("def verify_samples(self, samples, expected, robot_names):",
                      source)
        self.assertIn("def run_checkpoint(self, expected, moving):", source)
        self.assertIn("self.collect_amcl_samples(moving)", source)
        self.assertIn("self.verify_samples(samples, expected, moving)", source)
        self.assertIn('moving = list(stage["goals"].keys())', source)
        self.assertIn('scan TF unavailable', source)

    def test_formation_line_synchronizes_to_slowest_robot(self):
        source = read("scripts/fleet_vendor_omni/supervisor.py")
        tree = ast.parse(source)
        helper = next(node for node in tree.body
                      if isinstance(node, ast.FunctionDef)
                      and node.name == "synchronized_forward_speeds")
        module = ast.Module(body=[helper])
        if hasattr(module, "type_ignores"):
            module.type_ignores = []
        namespace = {}
        exec(compile(module, "fleet_vendor_omni_supervisor.py", "exec"),
             namespace)
        synchronize = namespace["synchronized_forward_speeds"]
        progress, speeds = synchronize(
            {"slow": 1.0, "ahead": 1.0},
            {"slow": 0.80, "ahead": 0.70},
            0.05, 0.025, 1.0, 0.18, 0.05)
        self.assertAlmostEqual(0.20, progress)
        self.assertEqual(0.05, speeds["slow"])
        self.assertEqual(0.0, speeds["ahead"])
        self.assertIn("def run_formation_line(self, stage, moving):", source)
        self.assertIn('stage_mode == "formation_line"', source)

    def test_runtime_rejects_a_goal_occupied_by_a_teammate(self):
        source = read("scripts/fleet_vendor_omni/supervisor.py")
        tree = ast.parse(source)
        helper = next(node for node in tree.body
                      if isinstance(node, ast.FunctionDef)
                      and node.name == "goal_occupancy_violations")
        module = ast.Module(body=[helper])
        if hasattr(module, "type_ignores"):
            module.type_ignores = []
        namespace = {"math": math}
        exec(compile(module, "fleet_vendor_omni_supervisor.py", "exec"),
             namespace)
        check = namespace["goal_occupancy_violations"]

        current = {
            "robot_2": {"x": -0.043, "y": 0.887},
            "robot_5": {"x": -0.967, "y": 0.940},
        }
        blocked = check(
            current,
            {"robot_5": {"x": 0.185, "y": 0.940}}, 0.50)
        self.assertEqual("robot_2", blocked[0]["blocking_robot"])
        self.assertEqual("robot_5", blocked[0]["robot"])
        self.assertEqual("current", blocked[0]["blocking_pose"])
        self.assertEqual([], check(
            current,
            {
                "robot_2": {"x": 1.085, "y": 0.887},
                "robot_5": {"x": 0.185, "y": 0.940},
            }, 0.50))
        self.assertIn("GOAL_OCCUPIED_BY_TEAMMATE", source)
        self.assertIn("STAGE_GOAL_REJECTED", source)

    def test_cached_preflight_and_direct_nomotion_service(self):
        source = read("scripts/fleet_vendor_omni/supervisor.py")
        begin = source.split("    def begin(self, run_all):", 1)[1]
        begin = begin.split("    def start_next", 1)[0]
        execute = source.split("    def execute_current_stage(self):", 1)[1]
        execute = execute.split("    def run_worker", 1)[0]
        self.assertIn("errors = self.cached_preflight_errors()", begin)
        self.assertNotIn("self.preflight()", begin)
        self.assertNotIn("self.preflight()", execute)
        self.assertIn("rospy.ServiceProxy(", source)
        self.assertIn("request_nomotion_update", source)
        self.assertNotIn("subprocess", source)
        self.assertIn("retrying checkpoint for %s", begin)
        self.assertIn("def check_static_parameters(self):", source)
        self.assertIn("def prime_startup_amcl(self):", source)
        self.assertEqual(5.0, self.config["checkpoint"][
            "preflight_cache_max_age"])

    def test_yaw_excursion_guard_is_enabled_for_six_robot_run(self):
        checkpoint = self.config["checkpoint"]
        self.assertEqual(15.0, checkpoint["yaw_excursion_guard_deg"])
        self.assertEqual(0.5, checkpoint["yaw_excursion_guard_hold"])
        source = read("scripts/fleet_vendor_omni/supervisor.py")
        self.assertIn('return False, "YAW_EXCURSION"', source)
        self.assertIn('sample["receive_wall"] < guard_started', source)

    def test_filtered_local_costmap_has_runtime_clearance_guard(self):
        navigation = read(
            "launch/fleet_vendor_omni/include/robot_navigation.launch")
        self.assertIn(
            '<arg name="local_costmap_sensor_topic" '
            'value="/$(arg robot_name)/amcl_scan"/>', navigation)
        scan_filter = yaml.safe_load(read(
            "config/fleet_vendor_omni/amcl_scan_filter.yaml"))
        self.assertFalse(scan_filter["use_scan_stamp"])
        self.assertEqual(0.42, self.config["checkpoint"][
            "runtime_teammate_min_distance"])
        source = read("scripts/fleet_vendor_omni/supervisor.py")
        self.assertIn("def teammate_clearance_violations(self):", source)
        self.assertIn('return False, "TEAMMATE_CLEARANCE"', source)

    def test_rviz_and_recorder_cover_all_six_robots(self):
        rviz = read("rviz/fleet_vendor_omni_six.rviz")
        recorder = read("scripts/fleet_vendor_omni/record.sh")
        for robot_id in range(1, 7):
            self.assertIn("Topic: /robot_%d/scan" % robot_id, rviz)
            self.assertIn(
                "Topic: /robot_%d/move_base/GlobalPlanner/plan" % robot_id,
                rviz)
        self.assertEqual(6, rviz.count("Style: Flat Squares"))
        self.assertEqual(6, rviz.count("Size (Pixels): 5"))
        self.assertIn("FLEET_ROBOT_COUNT:-6", recorder)
        self.assertIn('"/${robot}/amcl_scan"', recorder)
        self.assertIn('"/${robot}/ros_robot_controller/set_motor"', recorder)
        self.assertIn('"/${robot}/move_base/local_costmap/costmap_updates"', recorder)
        self.assertIn('"/${robot}/move_base/TebLocalPlannerROS/local_plan"', recorder)


if __name__ == "__main__":
    unittest.main()
