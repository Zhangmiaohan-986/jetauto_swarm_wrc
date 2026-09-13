#!/usr/bin/env python3
"""Offline templates only: these checks are NOT full ROS simulation approval."""
import copy
import os
import sys
import unittest
from unittest import mock
import numpy as np
import yaml
ROOT=os.path.abspath(os.path.join(os.path.dirname(__file__),'..'))
sys.path.insert(0,os.path.join(ROOT,'scripts','fleet_vendor_omni'))
from plan_profile import generate, formation_catalog, template_slots, template_assignment
from fleet_geometry import MapGrid, distance

class FormationCatalogTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.catalog=formation_catalog(ROOT)
        base=os.path.join(ROOT,'config/fleet_vendor_omni/profiles/fleet10_same_map')
        with open(os.path.join(base,'planning.yaml')) as stream: cls.spec=yaml.safe_load(stream)
        with open(os.path.join(base,'mission.yaml')) as stream: cls.current=yaml.safe_load(stream)

    def test_generated_mission_matches_planning_source(self):
        self.assertEqual(self.current,generate(ROOT,self.spec))

    def test_all_eight_shapes_have_ten_separated_slots(self):
        self.assertEqual(8,len(self.catalog['formations']))
        for key in self.catalog['formations']:
            slots=template_slots(self.catalog,{'formation':key,'anchor':[0,0]},10)
            for i,a in enumerate(slots):
                for b in slots[i+1:]: self.assertGreaterEqual(distance(a,b),.55,key)
        for order in self.catalog['schemes'].values():
            self.assertTrue(set(order).issubset(self.catalog['formations']))

    def test_transform_pinning_and_invalid_inputs(self):
        phase={'formation':'double_column','anchor':[5,3],'rotation_deg':90}
        slots=template_slots(self.catalog,phase,10)
        self.assertEqual([4.7,3.0],slots[0])
        names=self.spec['robot_order']; poses=dict(zip(names,slots))
        fixed=dict(phase,slot_robots=list(reversed(names)))
        result=template_assignment(names,poses,slots,fixed)
        self.assertEqual(slots[0],result[names[-1]])
        for change in ({'anchor':None},{'scale':-1},{'anchor':[float('nan'),0]},{'formation':'missing'}):
            bad=dict(phase,**change)
            with self.assertRaises(ValueError): template_slots(self.catalog,bad,10)
        with self.assertRaises(ValueError):
            template_assignment(names,poses,slots,{'slot_robots':[names[0]]*10})

    def test_template_flows_through_existing_route_planner(self):
        spec=copy.deepcopy(self.spec)
        slots=template_slots(self.catalog,{'formation':'double_column','anchor':[-3,0]},10)
        spec['robots']={n:{'initial_x':p[0],'initial_y':p[1],'initial_yaw_deg':0.0}
                        for n,p in zip(spec['robot_order'],slots)}
        spec['phases']=[{'name':'FORM_DOUBLE','layout':'template','mode':'transition',
                         'formation':'double_column','anchor':[2,0],'timeout':300}]
        grid=MapGrid(np.zeros((160,200)),.1,(-10,-8),.34)
        with mock.patch('plan_profile.MapGrid.load',return_value=grid): result=generate(ROOT,spec)
        stage=result['stages'][0]
        self.assertEqual(10,len(stage['tasks']))
        self.assertEqual('transition',stage['mode'])
        self.assertTrue(all(g['yaw_deg']==0 for g in stage['goals'].values()))

    def test_reject_unsupported_direct_control_direction(self):
        spec=copy.deepcopy(self.spec)
        spec['phases']=[{'name':'SIDEWAYS','mode':'formation_line','layout':'translate',
                         'delta':[0,1],'timeout':50}]
        with self.assertRaisesRegex(ValueError,'map \\+x'): generate(ROOT,spec)
        spec['phases']=[{'name':'UNSAFE_DIRECT_CHANGE','mode':'formation_line',
                         'layout':'template','formation':'single_file','anchor':[2,0],'timeout':50}]
        with self.assertRaisesRegex(ValueError,'transition mode'): generate(ROOT,spec)

if __name__=='__main__': unittest.main()
