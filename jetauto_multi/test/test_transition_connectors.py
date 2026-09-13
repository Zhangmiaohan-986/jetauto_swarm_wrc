#!/usr/bin/env python3
"""Exercise the actual connector certificate without ROS or robot commands."""
import ast
import math
import os
import sys
import threading
import time
import unittest
from types import SimpleNamespace as NS
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'scripts/fleet_vendor_omni'))
from fleet_geometry import point_path,paths_conflict

class ConnectorTest(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'scripts/fleet_vendor_omni/supervisor.py')) as f:
            tree=ast.parse(f.read())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='connector_clear')
        env=dict(math=math,time=time,point_path=point_path,paths_conflict=paths_conflict,
            rospy=NS(Time=NS(now=lambda:NS(to_sec=time.time)),Duration=lambda x:x))
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'<actual-connector>','exec'),env)
        self.check=env['connector_clear']
        vector=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='vector_command')
        tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='tick_connector')
        env['Twist']=lambda:NS(linear=NS(x=0.,y=0.),angular=NS(z=0.))
        exec(compile(ast.Module(body=[vector,tick],type_ignores=[]),'<actual-vector-tick>','exec'),env)
        self.vector=env['vector_command'];self.tick=env['tick_connector']
        self.scan=NS(header=NS(frame_id='lidar',stamp=1),range_min=.05,range_max=8.,
                     angle_min=0.,angle_increment=.1,ranges=[float('inf')]*20)
        tr=NS(transform=NS(translation=NS(x=0.,y=0.),rotation=None))
        self.owner=NS(transition_grid=NS(line_free=lambda *a:True),lock=threading.Lock(),
            runtime_teammate_min_distance=.42,sensor_max_age=.8,map_frame='map',
            yaw_from_quaternion=lambda q:0.,tf_buffer=NS(lookup_transform=lambda *a:tr),
            latest={'a':{'scan':dict(message=self.scan,receive_wall=time.time(),stamp=time.time())}})
        self.goals={'a':{'x':.1,'y':0.}}
        self.poses={'a':[0.,0.,0.],'b':[0.,1.,0.]}

    def test_free_and_blocked_connectors(self):
        self.assertIsNone(self.check(self.owner,self.goals,self.poses))
        self.scan.ranges=[.25]
        self.assertIn('obstacle',self.check(self.owner,self.goals,self.poses))
        self.scan.ranges=[float('nan')]
        self.assertIn('blind',self.check(self.owner,self.goals,self.poses))
        self.scan.ranges=[float('inf')]
        self.poses['b']=[.3,0.,0.]
        self.assertIn('conflicts',self.check(self.owner,self.goals,self.poses))
        self.poses['b']=[0.,1.,0.]
        self.owner.transition_grid.line_free=lambda *a:False
        self.assertIn('map',self.check(self.owner,self.goals,self.poses))

    def test_stale_is_not_bypassed(self):
        self.owner.latest['a']['scan']['stamp']-=2
        self.assertIn('stale',self.check(self.owner,self.goals,self.poses))

    def test_active_route_not_just_current_point_and_no_double_margin(self):
        # Currently distant teammate will cross the short path: must not admit it.
        self.assertIn('conflicts',self.check(self.owner,self.goals,self.poses,
            {'b':[[0.,1.],[0.,-.5]]},.08,.65))
        # Measured parked center: .42+.08=.50, not nominal .55 plus .08 again.
        self.poses['b']=[0.,.54,0.]
        self.assertIsNone(self.check(self.owner,self.goals,self.poses,{},.08,.65))
        self.assertIn('conflicts',self.check(self.owner,self.goals,self.poses,
            {'b':[[0.,.54],[1.,.54]]},.08,.65))

    def test_vector_acceleration_and_stop(self):
        goal={'x':.1,'y':.2,'yaw_deg':0.}
        cmd,done=self.vector([0.,0.,0.],goal,{},(0.,0.,0.),.05)
        self.assertFalse(done);self.assertGreater(cmd[0],0);self.assertGreater(cmd[1],0)
        self.assertLessEqual(max(abs(v) for v in cmd),.005001)
        self.assertEqual(((0.,0.,0.),True),self.vector([.1,.2,0.],goal,{},cmd,.05))

    def test_short_tick_does_not_cancel_other_robot_and_keeps_reservation_on_wait(self):
        sent=[];events=[]
        self.owner.ACTIVE_GOAL_STATES=(0,1,6,7)
        self.owner.clients={'a':NS(get_state=lambda:3),'b':NS(get_state=lambda:1)}
        self.owner.robot_stopped=lambda n:True
        self.owner.publish_event=lambda *a,**kw:events.append((a,kw))
        checked=[]
        self.owner.connector_clear=lambda *a:checked.append(a) or None
        state={'running':True,'connector':dict(goal={'x':.1,'y':0.,'yaw_deg':0.},
            route=[[0,0],[.1,0]],started=time.time()-1,last_tick=time.time()-.05,
            cfg={},command=(0.,0.,0.),stable=None)}
        pub=NS(publish=sent.append,get_num_connections=lambda:1,unregister=lambda:None)
        stage={'name':'test','tracking_error':.08,'tracking_error_hard':.10,
               'tracking_error_hold':2.,'route_reservation':.65,
               'connector_recheck_period':0.,'connector_clear_stable':.5,
               'connector_block_timeout':20.,'connector_motion_timeout':20.}
        self.assertIsNone(self.tick(self.owner,'a',state,stage,self.poses,{},pub))
        self.assertGreater(sent[-1].linear.x,0.)
        self.assertAlmostEqual(.58,checked[-1][-1])
        self.owner.connector_clear=lambda *a:'a: connector conflicts with b'
        self.assertIsNone(self.tick(self.owner,'a',state,stage,self.poses,{},pub))
        self.assertEqual(0.,sent[-1].linear.x);self.assertTrue(state['running'])
        self.assertTrue(state['connector'])  # Paused path remains reserved.
        self.owner.connector_clear=lambda *a:None
        state['connector']['clear_since']=time.time()-.6
        self.assertIsNone(self.tick(self.owner,'a',state,stage,self.poses,{},pub))
        self.assertIsNone(state['connector']['blocked'])
        self.assertGreater(sent[-1].linear.x,0.)
        self.owner.connector_clear=lambda *a:'a: obstacle in short connector'
        state['connector']['blocked']='a: obstacle in short connector'
        state['connector']['blocked_since']=time.time()-21.
        self.assertEqual('TRANSITION_CONNECTOR_BLOCKED',
            self.tick(self.owner,'a',state,stage,self.poses,{},pub))
        self.owner.clients['a'].get_state=lambda:1
        self.assertEqual('TRANSITION_COMMAND_OWNER_CONFLICT',
            self.tick(self.owner,'a',state,stage,self.poses,{},pub))

if __name__=='__main__':unittest.main()
