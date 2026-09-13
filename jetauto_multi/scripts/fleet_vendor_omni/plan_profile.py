#!/usr/bin/env python
"""Generate a inspectable mission from map + formation intent; no robot motion."""
from __future__ import print_function
import argparse
import copy
import hashlib
import json
import math
import os
import sys
sys.path.insert(0,os.path.dirname(os.path.realpath(__file__)))
import yaml
from fleet_geometry import MapGrid, assign_slots, plan_transition, distance
from transition_gates import plan_gated_transition, validate_gates

def formation_catalog(root):
    with open(os.path.join(root,'config/fleet_vendor_omni/formation_catalog.yaml')) as stream:
        return yaml.safe_load(stream)

def template_slots(catalog, phase, count):
    """Transform formation slots only; this never rotates the car's heading."""
    key=phase.get('formation')
    if key not in catalog['formations']:
        raise ValueError('Unknown formation template: %s'%key)
    slots=catalog['formations'][key]['slots']
    if len(slots)!=count:
        raise ValueError('Template slot count differs from robot count')
    anchor=phase.get('anchor')
    if not isinstance(anchor,(list,tuple)) or len(anchor)!=2:
        raise ValueError('Template needs a map-specific anchor: [x,y]')
    scale=float(phase.get('scale',1.0))
    angle=float(phase.get('rotation_deg',0.0))
    values=list(anchor)+[scale,angle]+[v for slot in slots for v in slot]
    if any(not isinstance(v,(float,int)) or math.isnan(v) or math.isinf(v) for v in values) or scale<=0:
        raise ValueError('Template coordinates/scale must be finite; scale must be positive')
    c,s=math.cos(math.radians(angle)),math.sin(math.radians(angle))
    return [[round(anchor[0]+scale*(c*x-s*y),4),round(anchor[1]+scale*(s*x+c*y),4)] for x,y in slots]

def template_assignment(names, poses, slots, phase):
    fixed=phase.get('slot_robots')
    if fixed is None:
        return assign_slots(names,poses,slots)
    if len(fixed)!=len(names) or set(fixed)!=set(names):
        raise ValueError('slot_robots must list each robot exactly once in slot order')
    return dict(zip(fixed,slots))

def generate(root,spec):
    with open(os.path.join(root,spec['baseline'])) as stream: baseline=yaml.safe_load(stream)
    config=copy.deepcopy(baseline)
    config.update(robot_order=spec['robot_order'],robots=spec['robots'],map_frame=spec['map_frame'])
    names=config['robot_order']; half=len(names)//2
    if len(names)!=10: raise ValueError('The rows/columns/v_pairs templates currently require ten robots')
    if len(set(names))!=len(names) or set(spec['robots'])!=set(names):
        raise ValueError('Provide exactly one initial map pose for every robot in robot_order')
    poses=dict((n,[p['initial_x'],p['initial_y']]) for n,p in spec['robots'].items())
    planning=spec['planning']; map_path=os.path.join(root,spec['map_yaml'])
    scheduler=planning.get('transition_scheduler','whole_route')
    if scheduler not in ('whole_route','progress_gates'):
        raise ValueError('Unknown transition_scheduler')
    if scheduler=='progress_gates':
        for key in ('gate_step','release_margin','parked_clearance','route_reservation',
                    'tracking_error','tracking_error_hard','tracking_error_hold',
                    'tracking_recovery_wait','transition_sensor_recovery_timeout',
                    'teb_abort_retry_delay','teb_abort_timeout',
                    'teb_direction_min_speed','route_recovery_advance',
                    'route_recovery_timeout',
                    'connector_recheck_period','connector_clear_stable',
                    'connector_block_timeout','connector_motion_timeout',
                    'dependency_deadlock_timeout'):
            value=planning[key]
            if not isinstance(value,(int,float)) or math.isnan(value) or math.isinf(value) or value<=0:
                raise ValueError('Expected finite positive '+key)
        retries=planning['teb_abort_max_retries']
        if not isinstance(retries,int) or isinstance(retries,bool) or not 1<=retries<=5:
            raise ValueError('teb_abort_max_retries must be an integer from 1 to 5')
        if planning['connector_clear_stable']<2*planning['connector_recheck_period']:
            raise ValueError('connector_clear_stable needs at least two rechecks')
        if planning['connector_block_timeout']<=planning['connector_clear_stable']:
            raise ValueError('connector_block_timeout must exceed connector_clear_stable')
        if planning['teb_abort_timeout']<=planning['teb_abort_retry_delay']:
            raise ValueError('teb_abort_timeout must exceed teb_abort_retry_delay')
        cosine=planning['teb_direction_min_cosine']
        if (not isinstance(cosine,(int,float)) or math.isnan(cosine)
                or math.isinf(cosine) or not -1.0<=cosine<=1.0):
            raise ValueError('teb_direction_min_cosine must be finite in [-1,1]')
        if not .02<planning['route_recovery_advance']<=planning.get('min_teb_distance',.25):
            raise ValueError('route_recovery_advance must be within (0.02,min_teb_distance]')
        if planning['release_margin']<0.05 or planning['gate_step']>0.2:
            raise ValueError('Use release_margin >= 0.05m and gate_step <= 0.2m')
    grid=MapGrid.load(map_path,planning['wall_radius'])
    if scheduler=='progress_gates' and planning['parked_clearance'] < (
            config['checkpoint']['runtime_teammate_min_distance']+planning['tracking_error']+0.04):
        raise ValueError('Nominal parked clearance needs runtime minimum + tracking + 0.04m placement allowance')
    if scheduler=='progress_gates' and not (planning['tracking_error'] <
            planning['tracking_error_hard'] <= (planning['route_reservation']-
            config['checkpoint']['runtime_teammate_min_distance'])/2.0):
        raise ValueError('Tracking recovery exceeds the moving-route safety envelope')
    if scheduler=='progress_gates':
        config['checkpoint']['transition_sensor_recovery_timeout']=(
            planning['transition_sensor_recovery_timeout'])
    for name,pose in poses.items():
        if not grid.free(pose): raise ValueError('Initial body is outside certified free map: '+name)
    phases=[]; groups=None
    catalog=formation_catalog(root) if any(p['layout']=='template' for p in spec['phases']) else None
    if len(set(p['name'] for p in spec['phases']))!=len(spec['phases']):
        raise ValueError('Phase names must be unique')
    for phase in spec['phases']:
        kind=phase['layout']
        if phase['mode'] not in ('transition','formation_line'):
            raise ValueError('Unsupported mode in '+phase['name'])
        if phase['mode']=='formation_line' and kind not in ('rows','translate'):
            raise ValueError('Formation changes need transition mode: '+phase['name'])
        if kind=='rows':
            goals=dict((name,[phase['row_x'][i//half],poses[name][1]]) for i,name in enumerate(names))
            for outer,inner,sign in ((0,1,1),(4,3,-1),(5,6,1),(9,8,-1)):
                goals[names[outer]][1]=goals[names[inner]][1]+sign*phase['outer_spacing']
        elif kind=='columns':
            slots=[[phase['front_x']-row*phase['spacing'],y] for y in phase['lane_y'] for row in range(half)]
            goals=assign_slots(names,poses,slots)
            groups=[[name for name in names if goals[name][1]==y] for y in phase['lane_y']]
        elif kind=='v_pairs':
            if groups is None:
                ordered=sorted(names,key=lambda n:-poses[n][1]); groups=[ordered[:half],ordered[half:]]
            goals={}
            for group,y in zip(groups,phase['lane_y']):
                slots=[[phase['front_x'],y]]
                for row,dy in enumerate(phase['row_dy'],1):
                    slots += [[phase['front_x']-row*phase['row_dx'],y+dy],[phase['front_x']-row*phase['row_dx'],y-dy]]
                goals.update(assign_slots(group,poses,slots))
        elif kind=='template':
            slots=template_slots(catalog,phase,len(names))
            goals=template_assignment(names,poses,slots,phase)
            # A new global assignment must not reuse old two-column membership.
            groups=None
        elif kind=='translate':
            if phase['mode']=='formation_line' and (phase['delta'][0]<=0 or abs(phase['delta'][1])>1e-8):
                raise ValueError('Current formation_line requires map +x translation; use transition for other directions')
            goals=dict((n,[p[0]+phase['delta'][0],p[1]+phase['delta'][1]]) for n,p in poses.items())
        else: raise ValueError('Unknown formation template '+kind)
        gated_tasks=None
        if phase['mode']=='transition' and scheduler=='progress_gates':
            # Ten slots are interchangeable, including across the previous lanes.
            slots=[goals[n] for n in names]
            goals,gated_tasks,dispatch_evidence=plan_gated_transition(
                grid,poses,slots,planning,seed_goals=goals,fixed=bool(phase.get('slot_robots')))
            groups=None
        for name,goal in goals.items():
            if not grid.free(goal): raise ValueError('%s: %s target is not map-clear: %s'%(phase['name'],name,goal))
            if phase['mode']=='formation_line' and goal[0]<poses[name][0]-.05:
                raise ValueError('%s: formation_line goal is behind %s'%(phase['name'],name))
            for other in names:
                if other!=name and distance(goal,goals[other])<config['checkpoint']['goal_teammate_min_distance']-1e-6:
                    raise ValueError('%s: goal spacing too small'%phase['name'])
        result={'name':phase['name'],'mode':phase['mode'],'timeout':phase['timeout'],
                'goals':dict((n,{'x':round(g[0],4),'y':round(g[1],4),'yaw_deg':0.}) for n,g in goals.items())}
        if phase['mode']=='transition':
            if gated_tasks is not None:
                result.update(tasks=gated_tasks,scheduler=scheduler,
                              entry_tolerance=planning.get('entry_tolerance',0.025),
                              entry_release_tolerance=planning.get(
                                  'entry_release_tolerance',0.035),
                              min_teb_distance=planning.get('min_teb_distance',0.25),
                              release_margin=planning['release_margin'],
                              parked_clearance=planning['parked_clearance'],
                              dispatch_evidence=dispatch_evidence)
                validate_gates(gated_tasks,result['goals'])
            else:
                # Explicit rollback route; do not duplicate/comment out a planner.
                try:
                    order=catalog['formations'][phase['formation']]['placement_order'] if kind=='template' else 'shortest'
                    result['tasks']=plan_transition(grid,poses,goals,planning['parked_clearance'],planning['route_reservation'],phase.get('placement_order',order))
                except ValueError as exc:
                    raise ValueError('%s: %s'%(phase['name'],exc))
            result['tracking_error']=planning['tracking_error']
            result['tracking_error_hard']=planning['tracking_error_hard']
            result['tracking_error_hold']=planning['tracking_error_hold']
            result['tracking_recovery_wait']=planning['tracking_recovery_wait']
            result['route_reservation']=planning['route_reservation']
            for key in ('teb_abort_max_retries','teb_abort_retry_delay',
                        'teb_abort_timeout','teb_direction_min_speed',
                        'teb_direction_min_cosine','route_recovery_advance',
                        'route_recovery_timeout','connector_recheck_period',
                        'connector_clear_stable','connector_block_timeout',
                        'connector_motion_timeout','dependency_deadlock_timeout'):
                result[key]=planning[key]
        else:
            for name in names:
                if not grid.line_free(poses[name],goals[name]): raise ValueError(phase['name']+' straight sweep blocked: '+name)
            # synchronized swept center separation, including the initial placement.
            for i in range(101):
                t=i/100.; points={n:[poses[n][j]+t*(goals[n][j]-poses[n][j]) for j in (0,1)] for n in names}
                for a,name in enumerate(names):
                    for other in names[a+1:]:
                        if distance(points[name],points[other])<config['checkpoint']['runtime_teammate_min_distance']:
                            raise ValueError('formation sweep teammate collision')
        phases.append(result); poses=goals
    config['stages']=phases
    with open(map_path,'rb') as stream: map_meta=stream.read()
    with open(os.path.join(os.path.dirname(map_path),yaml.safe_load(map_meta)['image']),'rb') as stream: map_bytes=stream.read()
    config['planning_evidence']={'map_sha256':hashlib.sha256(map_meta+map_bytes).hexdigest(),
                                 'groups':groups or [],'geometry_checked':True}
    return config

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('planning',nargs='?'); parser.add_argument('--output')
    parser.add_argument('--list-formations',action='store_true',help='List nominal template extents; no map feasibility claim')
    parser.add_argument('--package-root',default=os.path.abspath(os.path.join(os.path.dirname(__file__),'../..')))
    args=parser.parse_args()
    if args.list_formations:
        catalog=formation_catalog(args.package_root)
        for name,item in sorted(catalog['formations'].items()):
            slots=item['slots']; xs,ys=zip(*slots)
            print('%s: center_span_x=%.2fm center_span_y=%.2fm'%(name,max(xs)-min(xs),max(ys)-min(ys)))
        print('Schemes: '+', '.join(sorted(catalog['schemes'])))
        return
    if not args.planning or not args.output:
        parser.error('planning and --output are required unless --list-formations is used')
    with open(args.planning) as stream: spec=yaml.safe_load(stream)
    mission=generate(args.package_root,spec)
    with open(args.output,'w') as stream:
        stream.write('# Generated by plan_profile.py. Edit planning.yaml, then regenerate.\n')
        yaml.safe_dump(mission,stream,default_flow_style=False)
    print(json.dumps({'output':args.output,'groups':mission['planning_evidence']['groups'],
                      'stages':[{'name':s['name'],'tasks':len(s.get('tasks',[]))} for s in mission['stages']]},indent=2))

if __name__=='__main__': main()
