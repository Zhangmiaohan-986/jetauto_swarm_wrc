#!/usr/bin/env python3
"""Bounded offline geometry/execution checks; not physical robot acceptance."""
import copy
import ast
import math
import os
import sys
import unittest
from types import SimpleNamespace as NS
import yaml
ROOT=os.path.abspath(os.path.join(os.path.dirname(__file__),'..'))
sys.path.insert(0,os.path.join(ROOT,'scripts','fleet_vendor_omni'))
from fleet_geometry import distance, segment_distance
from transition_gates import (compile_gates, validate_gates, permitted_target,
                              project_progress, remaining_route)


def load_route_tracking_action():
    with open(os.path.join(ROOT,'scripts/fleet_vendor_omni/supervisor.py')) as stream:
        tree=ast.parse(stream.read())
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef)
                  and n.name=='route_tracking_action')
    namespace={}
    exec(compile(ast.Module(body=[function],type_ignores=[]),
                 '<tracking-contract>','exec'),namespace)
    return namespace['route_tracking_action']


def load_supervisor_function(name):
    with open(os.path.join(ROOT,'scripts/fleet_vendor_omni/supervisor.py')) as stream:
        tree=ast.parse(stream.read())
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef)
                  and n.name==name)
    namespace={'math':math}
    exec(compile(ast.Module(body=[function],type_ignores=[]),
                 '<supervisor-contract>','exec'),namespace)
    return namespace[name]


def advance(task, progress, amount):
    progress=min(task['arc'][-1],progress+amount)
    for i in range(1,len(task['arc'])):
        if task['arc'][i]+1e-9>=progress:
            fraction=(progress-task['arc'][i-1])/(task['arc'][i]-task['arc'][i-1])
            a,b=task['path'][i-1:i+1]
            return progress,[a[k]+fraction*(b[k]-a[k]) for k in (0,1)]
    return progress,task['path'][-1]


def replay(stage, delay=False):
    tasks=stage['tasks']; progress={t['robot']:0. for t in tasks}; arrived=set()
    poses={t['robot']:t['path'][0] for t in tasks}; peak=0; closest=float('inf')
    for tick in range(12000):  # At most ten minutes of simulated time.
        next_progress={}; moving=0
        for task in tasks:
            name=task['robot']; p=progress[name]
            target=permitted_target(task,progress,arrived,p)
            amount=min(.005,max(0.,task['arc'][target]-p))
            if delay and name==tasks[0]['robot'] and 80<=tick<220:
                amount=0.  # Unexpected 7 second pause: must not release on time.
            value,pose=advance(task,p,amount)
            next_progress[name]=value; poses[name]=pose
            moving+=amount>1e-6
            if value>=task['arc'][-1]-1e-8:
                arrived.add(name)
        progress=next_progress; peak=max(peak,moving)
        names=list(poses)
        for i,name in enumerate(names):
            for other in names[i+1:]:
                closest=min(closest,distance(poses[name],poses[other]))
        if len(arrived)==len(tasks):
            return {'duration':(tick+1)*.05,'peak':peak,'minimum_distance':closest}
    raise AssertionError('Offline gate execution deadlock')


class GateTest(unittest.TestCase):
    def test_yaw_excursion_becomes_confirmed_local_recovery(self):
        step=load_supervisor_function('yaw_excursion_action')
        self.assertEqual(('normal',None),step(.1,.2,0.,.5,None))
        action,since=step(.3,.2,10.,.5,None)
        self.assertEqual(('confirm',10.),(action,since))
        self.assertEqual(('recover',10.),step(.3,.2,10.51,.5,since))

    def test_short_connector_preserves_clear_original_route(self):
        classify=load_supervisor_function('short_connector_action')
        class Grid:
            def line_free(self,a,b):
                return (tuple(a),tuple(b)) in {
                    ((0.,0.),(.1,0.)),((.1,0.),(.1,.1)),
                    ((.1,.1),(.2,.1))}
        grid=Grid(); route=[[0.,0.],[.1,0.],[.1,.1],[.2,.1]]
        self.assertEqual('teb',classify(grid,[route[0],route[-1]],route))
        self.assertEqual('connector',classify(grid,[route[0],route[1]],route[:2]))
        self.assertEqual('wait',classify(grid,[route[0],[.3,.3]],
                                         [route[0],[.3,.3]]))

    def test_teb_success_waits_for_stop_then_recovers_endpoint(self):
        step=load_supervisor_function('transition_goal_action')
        self.assertEqual(('wait_stop',None),step(.03,False,0.,.08,.10,2.,None))
        self.assertEqual(('arrive',None),step(.08,True,0.,.08,.10,2.,None))
        self.assertEqual(('recover',None),step(.0915,True,0.,.08,.10,2.,None))
        action,since=step(.11,True,10.,.08,.10,2.,None)
        self.assertEqual('confirm',action)
        self.assertEqual(('confirm',since),step(.11,True,11.9,.08,.10,2.,since))
        self.assertEqual(('recover',since),step(.11,True,12.01,.08,.10,2.,since))

    def test_slot_amcl_refresh_is_bounded_and_accepts_minimum_sample(self):
        step=load_supervisor_function('slot_amcl_request_action')
        self.assertEqual('request',step(0,0,False,0.,0.,8.,1,2,3))
        self.assertEqual('wait',step(0,1,True,1.,0.,8.,1,2,3))
        self.assertEqual('request',step(1,1,False,2.,2.,8.,1,2,3))
        self.assertEqual('verify',step(2,2,False,3.,3.,8.,1,2,3))
        self.assertEqual('verify',step(1,3,False,4.,4.,8.,1,2,3))
        self.assertEqual('verify',step(1,3,False,8.01,4.,8.,1,2,3))
        self.assertEqual('hold',step(0,3,False,8.01,4.,8.,1,2,3))

    def test_slot_amcl_queue_limits_active_verifications(self):
        admit=load_supervisor_function('slot_amcl_queue_admit')
        queue=[]
        self.assertTrue(admit(queue,'robot_1',0,2))
        self.assertTrue(admit(queue,'robot_2',1,2))
        self.assertFalse(admit(queue,'robot_3',2,2))
        self.assertEqual(['robot_3'],queue)
        self.assertFalse(admit(queue,'robot_3',2,2))
        self.assertTrue(admit(queue,'robot_3',1,2))
        self.assertEqual([],queue)

    def test_slot_amcl_needs_two_time_separated_observations(self):
        effective=load_supervisor_function('slot_amcl_effective_sample_count')
        first=[{'stamp':1.,'receive_wall':10.}]
        too_close=first+[{'stamp':2.,'receive_wall':10.3}]
        stable=first+[{'stamp':3.,'receive_wall':10.8}]
        self.assertEqual(1,effective(first,2,.8))
        self.assertEqual(1,effective(too_close,2,.8))
        self.assertEqual(2,effective(stable,2,.8))
        with open(os.path.join(ROOT,'scripts/fleet_vendor_omni/supervisor.py')) as stream:
            source=stream.read()
        self.assertIn('SLOT_AMCL_CANDIDATE',source)
        self.assertIn('SLOT_AMCL_DEGRADED_ACCEPTED',source)
        self.assertIn('SLOT_AMCL_CERTIFICATE_REUSED',source)
        self.assertIn('self.slot_amcl_samples_required = max(1, self.samples_required)',source)
        self.assertIn('slot_verify_queue',source)
        self.assertIn('CHECKPOINT_REUSED_SLOT_AMCL',source)
        self.assertIn("detail.get('slot_amcl_verified')",source)

    def test_slot_verify_survives_sensor_pause_without_stale_reservation(self):
        resend=load_supervisor_function('transition_sensor_resend_allowed')
        clear=load_supervisor_function('clear_transition_arrival_state')
        reserved=load_supervisor_function('transition_state_has_reservation')
        state={'target':5,'running':False,
               'slot_verify':{'status':'collecting'},'slot_local_hold':None,
               'goal_settle':None,'connector':None,'teb_retry':None,
               'plan_check':{'target':5},'plan_accepted':True,
               'teb_guard':{'target':5},'retry_inflight':True,
               'recovery_request':{'target':5},'tracking_fault':{'error':.09},
               'direction_recovery':{'next_target':5},
               'direction_local_hold':None,'connector_admission':{'started':1.},
               'slot_corrections':1,'needs_resend':True}

        # Robot3 is collecting post-stop AMCL when a fleet sensor pause occurs.
        self.assertFalse(resend('robot_3',state,set()))
        state['needs_resend']=resend('robot_3',state,set())
        self.assertFalse(state['needs_resend'])

        # A successful slot check must atomically release every stale owner.
        clear(state)
        arrived={'robot_3'}
        self.assertFalse(resend('robot_3',state,arrived))
        self.assertFalse(reserved(state))
        self.assertFalse(state['running'])
        self.assertIsNone(state['plan_check'])
        self.assertIsNone(state['teb_guard'])
        self.assertFalse(state['retry_inflight'])

        # Robot3 no longer contributes a committed-route blocker.
        for robot in ('robot_1','robot_5','robot_6'):
            blockers=['robot_3:committed_route'] if reserved(state) else []
            self.assertEqual([],blockers,robot)

        ordinary={'target':3,'running':False,'connector':None,
                  'teb_retry':None,'slot_verify':None,
                  'slot_local_hold':None,'goal_settle':None}
        self.assertTrue(resend('robot_1',ordinary,set()))
        self.assertFalse(resend('robot_8',ordinary,set(),require_running=True))
        ordinary['running']=True
        self.assertTrue(resend('robot_8',ordinary,set(),require_running=True))
        ordinary['connector_admission']={'started':1.}
        self.assertFalse(resend('robot_7',ordinary,set(),require_running=True))
        self.assertFalse(resend('robot_1',ordinary,{'robot_1'}))

    def test_transition_motion_owner_invalidates_stale_teb_work(self):
        claim=load_supervisor_function('transition_claim_motion_owner')
        ticket=load_supervisor_function('transition_owner_ticket')
        valid=load_supervisor_function('transition_owner_ticket_valid')
        conflict=load_supervisor_function('transition_motion_owner_error')
        state={'target':5,'running':True,'motion_owner':'TEB','owner_epoch':4,
               'connector':None,'slot_verify':None,'slot_local_hold':None,
               'direction_local_hold':None}
        old=ticket(state)
        self.assertTrue(valid(state,old,'TEB'))
        self.assertIsNone(conflict(state))

        previous,epoch=claim(state,'CONNECTOR')
        self.assertEqual(('TEB',5),(previous,epoch))
        self.assertFalse(valid(state,old,'TEB'))
        state.update(running=True,connector={'goal':{}},teb_retry=None,
                     plan_check=None,teb_guard=None,retry_inflight=False)
        self.assertIsNone(conflict(state))

        state['slot_verify']={'status':'queued'}
        self.assertIn('multiple active owners',conflict(state))

        with open(os.path.join(ROOT,'scripts/fleet_vendor_omni/supervisor.py')) as stream:
            source=stream.read()
        self.assertGreaterEqual(source.count('claim_connector_owner(name,state'),4)
        self.assertIn("reason='stale teb retry'",source)
        self.assertIn("'owner_epoch':epoch",source)

    def test_global_plan_recovery_may_return_to_an_older_node(self):
        regressed=load_supervisor_function('transition_progress_regression_fault')
        robot8={'maximum':.494,'direction_recovery':{
            'kind':'global_plan','phase':'return_verified'}}
        self.assertFalse(regressed(robot8,0.,.06))
        robot8['direction_recovery']=None
        self.assertTrue(regressed(robot8,0.,.06))

    def test_short_connector_wait_has_bounded_teb_fallback(self):
        action=load_supervisor_function('bounded_short_connector_action')
        robot7={'started':10.,'initial_target':9,'candidate_target':11}
        self.assertEqual('wait',action(robot7,11.99,2.))
        self.assertEqual('teb',action(robot7,12.01,2.))
        with open(os.path.join(ROOT,'scripts/fleet_vendor_omni/supervisor.py')) as stream:
            source=stream.read()
        self.assertIn("initial_target_index=pending.get('initial_target',target)",source)
        self.assertIn("reason='bounded short connector wait; checking current permitted TEB target'",source)

    def test_map_connector_failure_is_local_and_bounded(self):
        action=load_supervisor_function('connector_problem_action')
        self.assertEqual('wait', action(
            'robot_7: map connector occupied/unknown', 1.0, 20.0))
        self.assertEqual('local_hold', action(
            'robot_7: map connector occupied/unknown', 21.0, 20.0))
        self.assertEqual('wait', action(
            'robot_7: connector conflicts with robot_8', 21.0, 20.0))
        with open(os.path.join(ROOT,'scripts/fleet_vendor_omni/supervisor.py')) as stream:
            source=stream.read()
        self.assertIn('connector_local_hold',source)

    def test_ten_real_enables_only_bounded_slot_amcl_refresh(self):
        with open(os.path.join(ROOT,'scripts/fleet_vendor_omni/supervisor.py')) as stream:
            source=stream.read()
        for event in ('SLOT_AMCL_VERIFY_QUEUED','SLOT_AMCL_NOMOTION_REQUEST',
                      'SLOT_AMCL_NOMOTION_TIMEOUT','SLOT_AMCL_SAMPLE','SLOT_AMCL_VERIFIED',
                      'SLOT_AMCL_CANDIDATE','SLOT_AMCL_VERIFY_WAIT','SLOT_AMCL_REALIGN',
                      'SLOT_AMCL_LOCAL_HOLD'):
            self.assertIn(event,source)
        with open(os.path.join(ROOT,'launch','ten_real.launch')) as stream:
            real=stream.read()
        with open(os.path.join(ROOT,'launch','ten_sim.launch')) as stream:
            sim=stream.read()
        self.assertIn('slot_amcl_verify_enabled" default="true"',real)
        self.assertIn('slot_amcl_max_parallel" default="3"',real)
        self.assertIn('slot_amcl_verify_enabled" default="false"',sim)
        self.assertIn("'reason':'slot_amcl_realign'",source)
        self.assertIn("return False,'LOCAL_GOAL_HOLD'",source)

    def test_restart_near_endpoint_does_not_compare_original_start(self):
        classify=load_supervisor_function('transition_start_action')
        self.assertEqual('endpoint_recovery',classify(
            2.0577,.0915,.0915,1.9828,True,.025,.08,.25,.13))
        self.assertEqual('arrived',classify(
            2.0,.07,.07,1.98,True,.025,.08,.25,.13))
        self.assertEqual('resume',classify(
            1.0,.5,.03,1.2,True,.025,.08,.25,.13))
        self.assertEqual('endpoint_verify',classify(
            2.0,.07,.07,1.98,True,.025,.08,.25,.13,True))
        self.assertEqual('endpoint_recovery',classify(
            2.0,.0915,.0915,1.98,True,.025,.08,.25,.13,True))
        with open(os.path.join(ROOT,'scripts/fleet_vendor_omni/supervisor.py')) as stream:
            source=stream.read()
        self.assertIn('TRANSITION_RESTART_SLOT_VERIFY',source)

    def test_teb_first_command_guard_and_nearby_recovery_node(self):
        direction=load_supervisor_function('teb_command_direction')
        self.assertEqual(('waiting',None),direction(.001,0.,0.,(-.05,.08),.01,0.))
        decision,cosine=direction(-.03,-.07,0.,(-.05,.08),.01,0.)
        self.assertEqual('reverse',decision)
        self.assertLess(cosine,0.)
        self.assertEqual('forward',direction(-.03,.07,0.,(-.05,.08),.01,0.)[0])
        target=load_supervisor_function('recovery_target_index')
        task={'arc':[0.,.096,.192,.288,.384,.480]}
        self.assertEqual(4,target(task,.288,5,.12))

    def test_sp_bypass_and_bounded_direction_recovery(self):
        guard=load_supervisor_function('direction_guard_action')
        self.assertEqual(('bypass',2,10.),
            guard(False,'reverse',2,12.,10.,3,18.))
        rejections=0; first=None
        for now in (0.,1.,2.):
            action,rejections,first=guard(True,'reverse',rejections,now,first,3,18.)
            self.assertEqual('reject',action)
        self.assertEqual(('recover',4,0.),
            guard(True,'reverse',rejections,3.,first,3,18.))

        next_target=load_supervisor_function('direction_recovery_next_target')
        current=5; visited=[]
        while current<12:
            current=next_target(current,12); visited.append(current)
        self.assertEqual(list(range(6,13)),visited)

        hold=load_supervisor_function('direction_recovery_hold_code')
        dispatch=load_supervisor_function('transition_dispatch_allowed')
        scope=load_supervisor_function('transition_failure_scope')
        self.assertTrue(dispatch('robot_6','robot_6'))
        self.assertFalse(dispatch('robot_7','robot_6'))
        self.assertTrue(dispatch('robot_7',None))
        self.assertEqual('LOCAL_GOAL_HOLD',hold([]))
        self.assertEqual('local',scope(hold([])))
        self.assertEqual('DIRECTION_RECOVERY_BLOCKING_HOLD',hold(['robot_7']))
        self.assertEqual('fleet',scope(hold(['robot_7'])))

    def test_direction_recovery_events_and_sp_launch_contract(self):
        with open(os.path.join(ROOT,'scripts/fleet_vendor_omni/supervisor.py')) as stream:
            source=stream.read()
        events=(
            'SP_DIRECTION_GUARD_BYPASSED',
            'DIRECTION_RECOVERY_STARTED',
            'DIRECTION_RECOVERY_PROJECTED',
            'DIRECTION_RECOVERY_VECTOR_NODE',
            'DIRECTION_RECOVERY_TEB_REJOINED',
            'DIRECTION_RECOVERY_SUCCEEDED',
            'DIRECTION_RECOVERY_TIMEOUT',
        )
        for event in events:
            self.assertIn(event,source)
        self.assertNotIn("return False,'TEB_DIRECTION_REJECTED'",source)
        self.assertIn("transition_failure_scope(code)=='fleet'",source)
        for launch in ('ten_real.launch','ten_sim.launch'):
            with open(os.path.join(ROOT,'launch',launch)) as stream:
                content=stream.read()
            self.assertIn('sp_direction_guard_enabled" default="true"',content)
        with open(os.path.join(ROOT,'launch','ten_sp.launch')) as stream:
            self.assertIn('sp_direction_guard_enabled" value="false"',stream.read())

    def test_entry_hysteresis_and_sustained_failure(self):
        step=load_supervisor_function('entry_recovery_action')
        self.assertEqual(('ready',None),step(.02511,0.,.035,.10,2.,None))
        self.assertEqual(('recover',None),step(.09,0.,.035,.10,2.,None))
        action,since=step(.11,10.,.035,.10,2.,None)
        self.assertEqual('wait',action)
        self.assertEqual(('fail',since),step(.11,12.01,.035,.10,2.,since))

    def test_teb_abort_has_local_bounded_retry_budget(self):
        step=load_supervisor_function('teb_abort_action')
        recovery=None
        for attempt,now in enumerate((0.,2.,4.),1):
            action,recovery=step(now,recovery,3,1.5,18.)
            self.assertEqual('retry',action)
            self.assertEqual(attempt,recovery['failures'])
            self.assertEqual(now+1.5,recovery['retry_at'])
            self.assertTrue(recovery['waiting'])
        self.assertEqual('fail',step(6.,recovery,3,1.5,18.)[0])
        self.assertEqual('fail',step(18.01,{'first_abort':0.,'failures':1},
                                     3,1.5,18.)[0])

    def test_global_plan_handshake_blocks_cross_node_preempt(self):
        check_plan=load_supervisor_function('global_plan_check_action')
        preempt=load_supervisor_function('transition_target_preempt_action')
        check={'sent_at':10.0,'target':9}
        self.assertEqual(('waiting',0),check_plan(
            check,{'receive_wall':9.9,'points':40},10.5,1.0))
        self.assertEqual(('empty',0),check_plan(
            check,{'receive_wall':10.1,'points':0},10.2,1.0))
        self.assertEqual(('accepted',42),check_plan(
            check,{'receive_wall':10.1,'points':42},10.2,1.0))
        self.assertEqual(('timeout',0),check_plan(check,{},11.01,1.0))
        self.assertEqual('block',preempt(check,9,12))
        self.assertEqual('allow',preempt(None,9,12))
        self.assertEqual('allow',preempt(check,9,9))

    def test_global_plan_recovery_contract_is_recorded(self):
        with open(os.path.join(ROOT,'scripts/fleet_vendor_omni/supervisor.py')) as stream:
            source=stream.read()
        for event in (
                'GLOBAL_PLAN_CHECK','GLOBAL_PLAN_EMPTY','GLOBAL_PLAN_ACCEPTED',
                'TRANSITION_TARGET_PREEMPT_BLOCKED','TRANSITION_FALLBACK_NODE',
                'TRANSITION_POLYLINE_STEP','TRANSITION_WAIT_FOR_RECOVERY',
                'TRANSITION_RECOVERY_SUCCEEDED','TRANSITION_RECOVERY_TIMEOUT'):
            self.assertIn(event,source)
        self.assertIn('/%s/move_base/GlobalPlanner/plan',source)
        self.assertNotIn("return False,'TEB_RETRY_EXHAUSTED'",source)

    def test_resumed_idle_client_is_not_misread_as_lost(self):
        status=load_supervisor_function('transition_goal_status')
        self.assertIsNone(status({'running':False,'resumed':True},9,3))
        self.assertEqual(1,status({'running':True},1,3))
        self.assertEqual(3,status({'running':False,'connector_done':True},9,3))
        self.assertEqual(3,status({'running':False,
                                  'goal_settle':{'status':'waiting_for_stop'}},9,3))

    def test_route_tracking_warn_retry_recover_and_sustained_failure(self):
        step=load_route_tracking_action()
        action,fault=step(.0807,0.,.08,.10,2.,1.,None)
        self.assertEqual('pause',action)
        self.assertEqual('wait',step(.09,.5,.08,.10,2.,1.,fault)[0])
        self.assertEqual('retry',step(.09,1.,.08,.10,2.,1.,fault)[0])
        self.assertEqual(('recovered',None),step(.07,1.2,.08,.10,2.,1.,fault))
        action,warning=step(.09,0.,.08,.10,2.,1.,None)
        self.assertEqual('retry',step(.09,1.,.08,.10,2.,1.,warning)[0])
        self.assertEqual('recovering',step(.09,2.,.08,.10,2.,1.,warning)[0])
        self.assertEqual('pause',step(.09,3.01,.08,.10,2.,1.,warning)[0])
        action,fault=step(.11,0.,.08,.10,2.,1.,None)
        self.assertEqual('pause',action)
        self.assertEqual('fail',step(.11,2.01,.08,.10,2.,1.,fault)[0])

    def test_via_points_are_transformed_not_just_relabelled(self):
        # Simulation map->odom is identity; explicitly exercise a real nonidentity
        # transform against the actual publisher method, without importing ROS.
        with open(os.path.join(ROOT,'scripts/fleet_vendor_omni/supervisor.py')) as stream:
            tree=ast.parse(stream.read())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='publish_transition_path')
        sent=[]; frames=[]
        class Clock:
            def __init__(self,value): pass
            @staticmethod
            def now(): return 123
        def path(): return NS(header=NS(),poses=[])
        def pose(): return NS(header=NS(),pose=NS(orientation=NS(),position=NS()))
        def lookup(target,source,*args):
            frames.append((target,source))
            return NS(transform=NS(rotation=None,translation=NS(x=10.,y=20.)))
        namespace={'math':math,'rospy':NS(Time=Clock,Duration=lambda n:n),'Path':path,'PoseStamped':pose}
        exec(compile(ast.Module(body=[method],type_ignores=[]),'<publisher-contract>','exec'),namespace)
        owner=NS(via_pubs={'robot_7':NS(get_num_connections=lambda:1,publish=sent.append)},
                 tf_buffer=NS(lookup_transform=lookup),map_frame='robot_1/map',
                 yaw_from_quaternion=lambda q:math.pi/2)
        namespace['publish_transition_path'](owner,'robot_7',
            {'path':[[0,0],[1,2],[3,4]],'arc':[0,.2,.9]}, {'target':2,'progress':0.})
        self.assertEqual([('robot_7/odom','robot_1/map')],frames)
        self.assertEqual(1,len(sent[0].poses))
        self.assertEqual('robot_7/odom',sent[0].header.frame_id)
        self.assertAlmostEqual(8.,sent[0].poses[0].pose.position.x)
        self.assertAlmostEqual(21.,sent[0].poses[0].pose.position.y)

    def test_crossing_releases_before_leader_arrives_and_waits_on_delay(self):
        seed=[{'id':'a','robot':'a','path':[[0,0],[4,0]],'length':4.,'after':[]},
              {'id':'b','robot':'b','path':[[1,-2],[1,2]],'length':4.,'after':['a']}]
        tasks=compile_gates(seed,.65,.1,.1)
        self.assertGreater(permitted_target(tasks[1],{'a':0,'b':0},set(),0),0)
        self.assertLess(permitted_target(tasks[1],{'a':0,'b':0},set(),0),len(tasks[1]['path'])-1)
        self.assertEqual(len(tasks[1]['path'])-1,
                         permitted_target(tasks[1],{'a':2.0,'b':0},set(),0))
        for delayed in (False,True):
            metrics=replay({'tasks':tasks},delayed)
            self.assertEqual(2,metrics['peak'])
            self.assertGreater(metrics['minimum_distance'],.55)

    def test_current_profile_dependencies_and_delayed_execution(self):
        with open(os.path.join(ROOT,'config/fleet_vendor_omni/profiles/fleet10_same_map/mission.yaml')) as stream:
            mission=yaml.safe_load(stream)
        for stage in mission['stages']:
            if stage['mode']!='transition' or stage.get('scheduler')!='progress_gates':
                continue
            validate_gates(stage['tasks'],stage['goals'])
            # Every geometrically conflicting action pair has a prerequisite.
            previous=[]
            for task in stage['tasks']:
                for i,(a,b) in enumerate(zip(task['path'],task['path'][1:]),1):
                    for earlier in previous:
                        for j,(c,d) in enumerate(zip(earlier['path'],earlier['path'][1:]),1):
                            if segment_distance(a,b,c,d)<stage['route_reservation']:
                                self.assertGreaterEqual(task['gates'][i].get(earlier['robot'],0),earlier['arc'][j]-1e-9)
                previous.append(task)
            for delayed in (False,True):
                metrics=replay(stage,delayed)
                print(stage['name'],'delay='+str(delayed),metrics)
                self.assertGreaterEqual(metrics['peak'],3)
                self.assertGreater(metrics['minimum_distance'],.42)
            broken=copy.deepcopy(stage['tasks'])
            broken[0]['gates'][1][broken[-1]['robot']]=.1
            with self.assertRaises(ValueError): validate_gates(broken,stage['goals'])

    def test_progress_does_not_claim_uncommitted_future(self):
        task=compile_gates([{'id':'a','robot':'a','path':[[0,0],[2,0]],'length':2.,'after':[]}],.65,.1,.1)[0]
        deviation,progress=project_progress(task,[1.9,0],.2,5,.1)
        self.assertGreater(deviation,1)
        self.assertLessEqual(progress,.5)
        self.assertEqual([.2,0],remaining_route(task,[.2,0],.2,5)[0])


if __name__=='__main__': unittest.main()
