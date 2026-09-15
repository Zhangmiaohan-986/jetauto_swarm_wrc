#!/usr/bin/env python3
"""Shared profile and geometric scheduling contracts, without ROS."""
import os
import sys
import unittest
import xml.etree.ElementTree as ET
import xmlrpc.client
import numpy as np
import yaml
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(ROOT,'scripts','fleet_vendor_omni'))
from fleet_profile import load_profile, build_launch
from fleet_geometry import paths_conflict, MapGrid
from transition_gates import validate_gates

class ProfileContract(unittest.TestCase):
    def test_real_sim_share_control_and_runtime(self):
        for relative,count in [('six_robot_profile.yaml',6),('profiles/fleet10_same_map/profile.yaml',10)]:
            profile,mission,roster=load_profile(ROOT,'config/fleet_vendor_omni/'+relative)
            gated=any(s.get('scheduler')=='progress_gates' for s in mission['stages'])
            # rosparam is XML-RPC: YAML null cannot be sent to the ROS master.
            xmlrpc.client.dumps((mission,),allow_none=False)
            navigation=[]
            for sim in (False,True):
                root=ET.fromstring(build_launch(ROOT,profile,mission,roster,sim))
                includes=root.findall('include')
                self.assertEqual(count,len(includes))
                args=[{a.get('name'):a.get('value') for a in i.findall('arg')} for i in includes]
                for index,a in enumerate(args,1):
                    self.assertEqual(str(count),a['robot_count'])
                    self.assertEqual('robot_%d'%index,a['robot_name'])
                    self.assertEqual('false' if sim else 'true',a.pop('enable_amcl'))
                    self.assertEqual('0.1',a['max_vel_x'])
                    self.assertEqual('false' if relative=='six_robot_profile.yaml' else 'true',a['imu_bias_estimation'])
                    self.assertEqual('true' if gated else 'false',a['use_transition_via_points'])
                navigation.append(args)
                types=[node.get('type') for node in root.findall('node')]
                self.assertIn('supervisor.py',types)
                self.assertEqual(sim,'sim_adapter.py' in types)
                models=root.findall('group')
                self.assertEqual(count if sim else 0,len(models))
                for index,model in enumerate(models,1):
                    name='robot_%d'%index
                    self.assertEqual(name,model.get('ns'))
                    self.assertIn('jetauto_description',model.find('param').get('command'))
                    state=model.find("node[@name='robot_state_publisher']")
                    self.assertEqual(name,state.find("param[@name='tf_prefix']").get('value'))
                    self.assertIsNotNone(model.find("node[@name='joint_state_publisher']"))
                self.assertFalse(any('guard' in t or 'safety' in t for t in types))
            self.assertEqual(navigation[0],navigation[1])
            required=mission['required_parameter_suffixes']
            if gated:
                self.assertEqual(-0.1,required['move_base/TebLocalPlannerROS/global_plan_viapoint_sep'])
                self.assertFalse(required['move_base/TebLocalPlannerROS/via_points_ordered'])
                for key in ('weight_max_vel_x','weight_max_vel_y','weight_acc_lim_y'):
                    self.assertEqual(20,required['move_base/TebLocalPlannerROS/'+key])

    def test_ten_mapping_and_transition_reservations(self):
        _,mission,roster=load_profile(ROOT,'config/fleet_vendor_omni/profiles/fleet10_same_map/profile.yaml')
        self.assertEqual(['192.168.1.%d'%i for i in range(111,121)],[r['ip'] for r in roster])
        with open(os.path.join(ROOT,'config/fleet_vendor_omni/six_robot_demo.yaml')) as stream:
            six=yaml.safe_load(stream)
        for new,old in [(2,2),(3,1),(4,3),(7,5),(8,4),(9,6)]:
            self.assertEqual(six['robots']['robot_%d'%old],mission['robots']['robot_%d'%new])
        for stage in mission['stages']:
            if stage['mode']!='transition': continue
            if stage.get('scheduler')=='progress_gates':
                validate_gates(stage['tasks'],stage['goals'])
                self.assertEqual(10,len(stage['tasks']))
                continue
            seen={}
            for task in stage['tasks']:
                self.assertTrue(set(task['after']).issubset(seen))
                for tid,previous in seen.items():
                    if paths_conflict(task['path'],previous['path'],stage['route_reservation']):
                        self.assertIn(tid,task['after'])
                seen[task['id']]=task
            self.assertEqual(10,len(seen))

    def test_geometry_blocks_crossings_and_finds_detour(self):
        self.assertTrue(paths_conflict([(0,0),(2,0)],[(1,-1),(1,1)],.5))
        self.assertFalse(paths_conflict([(0,0),(2,0)],[(0,1),(2,1)],.5))
        grid=MapGrid(np.zeros((30,30)),.1,(0,0),0)
        self.assertTrue(grid.free((1,1)))
        path=grid.plan((.3,1.5),(2.7,1.5),[(1.5,1.5)],.5)
        self.assertGreater(len(path),2)

    def test_public_entries_and_rviz(self):
        for name in ('real','sim','ten_real','ten_sim'):
            root=ET.parse(os.path.join(ROOT,'launch',name+'.launch')).getroot()
            args={a.get('name'):a.get('default') for a in root.findall('arg')}
            self.assertEqual('false',args['motion_enabled'])
        with open(os.path.join(ROOT,'rviz/fleet_vendor_omni_ten.rviz')) as stream:
            rviz=stream.read()
        for i in range(1,11): self.assertIn('/robot_%d/scan'%i,rviz)

if __name__=='__main__': unittest.main()
