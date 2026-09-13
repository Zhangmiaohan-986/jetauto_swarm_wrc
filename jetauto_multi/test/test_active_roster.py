#!/usr/bin/env python3
import os
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(ROOT, 'scripts', 'fleet_vendor_omni'))
from fleet_profile import load_profile, build_launch, filter_active_fleet


class ActiveRosterTest(unittest.TestCase):
    def test_excluding_robot10_builds_valid_nine_robot_launch(self):
        profile, mission, roster = load_profile(
            ROOT, 'config/fleet_vendor_omni/profiles/fleet10_same_map/profile.yaml')
        reduced, reduced_roster = filter_active_fleet(
            mission, roster, ['robot_10'])

        self.assertEqual(9, len(reduced['robot_order']))
        self.assertNotIn('robot_10', reduced['robot_order'])
        self.assertEqual(set(reduced['robot_order']), set(reduced['robots']))
        self.assertEqual(set(reduced['robot_order']),
                         set(reduced['runtime_profiles']))
        self.assertEqual(9, len(reduced_roster))
        for stage in reduced['stages']:
            self.assertNotIn('robot_10', stage.get('goals', {}))
            for task in stage.get('tasks', []):
                self.assertIn(task['robot'], reduced['robot_order'])
                self.assertNotIn('robot_10', task.get('after', []))
                for gate in task.get('gates', []):
                    self.assertNotIn('robot_10', gate)

        root = ET.fromstring(build_launch(ROOT, profile, reduced,
                                          reduced_roster, False, False, False))
        includes = root.findall('include')
        self.assertEqual(9, len(includes))
        for include in includes:
            args = {item.attrib['name']: item.attrib['value']
                    for item in include.findall('arg')}
            self.assertEqual('9', args['robot_count'])
        self.assertNotIn('robot_10', ET.tostring(root).decode('utf-8'))


if __name__ == '__main__':
    unittest.main()
