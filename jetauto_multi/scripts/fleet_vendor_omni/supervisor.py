#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Stage-gated MoveBase coordinator for the vendor-omni fleet trial."""

from __future__ import print_function

import json
import math
import os
import sys
import threading
import time
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from fleet_geometry import MapGrid, point_path, paths_conflict, distance
from transition_gates import validate_gates, permitted_target, project_progress, remaining_route

import actionlib
import rosgraph
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger, TriggerResponse


def flatten_parameter_tree(tree, prefix=""):
    result = []
    for key, value in tree.items():
        suffix = "/".join(part for part in (prefix, str(key).strip("/")) if part)
        if isinstance(value, dict):
            result.extend(flatten_parameter_tree(value, suffix))
        else:
            result.append((suffix, value))
    return result


def parameter_from_tree(tree, suffix):
    value = tree
    for part in suffix.strip("/").split("/"):
        if not isinstance(value, dict) or part not in value:
            raise KeyError(suffix)
        value = value[part]
    return value


def goal_occupancy_violations(current, goals, min_distance):
    violations = []
    for robot_name in sorted(goals):
        goal = goals[robot_name]
        for other_name in sorted(current):
            if other_name == robot_name:
                continue
            other_pose = goals.get(other_name, current[other_name])
            distance = math.hypot(
                float(goal["x"]) - float(other_pose["x"]),
                float(goal["y"]) - float(other_pose["y"]))
            if distance < min_distance:
                violations.append({
                    "robot": robot_name,
                    "blocking_robot": other_name,
                    "blocking_pose": (
                        "goal" if other_name in goals else "current"),
                    "distance": round(distance, 4),
                    "minimum_distance": round(min_distance, 4),
                })
    return violations


def synchronized_forward_speeds(initial_remaining, remaining, max_speed,
                                min_speed, sync_gain, slow_distance,
                                tolerance):
    """Return a shared progress value and per-robot forward speeds."""
    if not initial_remaining:
        return 1.0, {}
    max_distance = max(initial_remaining.values())
    progress = {}
    for name, distance in initial_remaining.items():
        if distance <= tolerance:
            progress[name] = 1.0
        else:
            progress[name] = max(0.0, min(
                1.0, (distance - remaining[name]) / distance))
    shared_progress = min(progress.values())
    speeds = {}
    for name, distance in initial_remaining.items():
        if remaining[name] <= tolerance:
            speeds[name] = 0.0
            continue
        nominal = max_speed * distance / max(max_distance, tolerance)
        raw_speed = nominal + sync_gain * (
            shared_progress - progress[name]) * distance
        if raw_speed < min_speed:
            speeds[name] = 0.0
            continue
        slowdown = min(1.0, remaining[name] / max(slow_distance, tolerance))
        speeds[name] = min(max_speed, max(min_speed, raw_speed * slowdown))
    return shared_progress, speeds


def route_tracking_action(error, now, soft_limit, hard_limit,
                          hard_hold, recovery_wait, fault):
    """Classify one route error without turning a brief miss into fleet failure."""
    if error <= soft_limit:
        return ('recovered' if fault else 'normal'), None
    if fault is None:
        fault={'retry_at':now+recovery_wait,'recovering':False,
               'recover_until':None,
               'hard_since':now if error>hard_limit else None}
        return 'pause',fault
    if error>hard_limit:
        if fault.get('hard_since') is None:
            fault['hard_since']=now
        if now-fault['hard_since']>=hard_hold:
            return 'fail',fault
    else:
        fault['hard_since']=None
    if fault.get('recovering'):
        if now<fault['recover_until']:
            return 'recovering',fault
        fault.update(retry_at=now+recovery_wait,recovering=False,
                     recover_until=None)
        return 'pause',fault
    if now>=fault['retry_at']:
        fault.update(recovering=True,recover_until=now+hard_hold)
        return 'retry',fault
    return 'wait',fault


def entry_recovery_action(error, now, release_limit, hard_limit,
                          hard_hold, hard_since):
    """Use hysteresis at a route entry and confirm large misses before failing."""
    if error <= release_limit:
        return 'ready',None
    if error <= hard_limit:
        return 'recover',None
    hard_since=hard_since or now
    return ('fail' if now-hard_since>=hard_hold else 'wait'),hard_since


def transition_start_action(start_error, end_error, projected_error, projected,
                            stopped, entry_limit, route_limit,
                            endpoint_recovery_limit, disagreement_limit,
                            slot_verify_enabled=False):
    """Resume a stopped transition without mistaking endpoint overshoot for its start."""
    if end_error <= endpoint_recovery_limit:
        if stopped and end_error <= route_limit:
            return 'endpoint_verify' if slot_verify_enabled else 'arrived'
        return 'endpoint_recovery'
    if projected > .05 and projected_error <= route_limit:
        return 'resume'
    if start_error > disagreement_limit:
        return 'reject'
    return 'entry_recovery' if start_error > entry_limit else 'start'


def transition_goal_action(error, stopped, now, soft_limit, hard_limit,
                           hard_hold, hard_since):
    """Wait for physical stop, then accept or recover one TEB endpoint locally."""
    if not stopped:
        return 'wait_stop',None
    if error <= soft_limit:
        return 'arrive',None
    if error <= hard_limit:
        return 'recover',None
    hard_since=now if hard_since is None else hard_since
    return ('recover' if now-hard_since>=hard_hold else 'confirm'),hard_since


def teb_abort_action(now, recovery, max_retries, retry_delay, timeout):
    """Retry one admitted TEB goal locally; escalate only after a bounded budget."""
    recovery=dict(recovery or {})
    first_abort=recovery.get('first_abort',now)
    failures=int(recovery.get('failures',0))+1
    recovery.update(first_abort=first_abort,failures=failures,
                    retry_at=now+retry_delay,waiting=True,clear_since=None)
    if failures>max_retries or now-first_abort>=timeout:
        return 'fail',recovery
    return 'retry',recovery


def global_plan_check_action(check, evidence, now, timeout):
    """Accept only plan evidence received after the corresponding goal."""
    if not check:
        return 'inactive',0
    if evidence and evidence.get('receive_wall',0.)>=check['sent_at']:
        points=int(evidence.get('points',0))
        return ('accepted' if points>0 else 'empty'),points
    return ('timeout' if now-check['sent_at']>=timeout else 'waiting'),0


def transition_target_preempt_action(check, current_target, candidate_target):
    """Do not replace a goal until its own global-plan result is known."""
    if check and candidate_target>current_target:
        return 'block'
    return 'allow'


def teb_command_direction(vx, vy, yaw, tangent, min_speed, min_cosine):
    """Classify the first meaningful TEB command in the map-frame route direction."""
    speed=math.hypot(vx,vy)
    if speed<min_speed:
        return 'waiting',None
    map_vx=math.cos(yaw)*vx-math.sin(yaw)*vy
    map_vy=math.sin(yaw)*vx+math.cos(yaw)*vy
    length=max(math.hypot(tangent[0],tangent[1]),1e-12)
    cosine=(map_vx*tangent[0]+map_vy*tangent[1])/(speed*length)
    return ('forward' if cosine>=min_cosine else 'reverse'),cosine


def direction_guard_action(enabled, direction, rejections, now, first_reject,
                           max_retries, timeout):
    """Keep the legacy guard switchable and enter recovery only after its budget."""
    if direction!='reverse':
        return direction,rejections,first_reject
    if not enabled:
        return 'bypass',rejections,first_reject
    rejections+=1
    first_reject=now if first_reject is None else first_reject
    action=('recover' if rejections>max_retries or now-first_reject>=timeout
            else 'reject')
    return action,rejections,first_reject


def route_index_at_or_before(task, progress):
    """Last certified polyline node already covered by projected route progress."""
    result=0
    for index,value in enumerate(task['arc']):
        if value>progress+1e-8:
            break
        result=index
    return result


def direction_recovery_next_target(current, original_target):
    """Advance exactly one original polyline node; never jump to a far gate."""
    return min(current+1,original_target)


def direction_recovery_hold_code(blockers):
    """Escalate a spent local recovery only when the parked robot blocks a route."""
    return 'DIRECTION_RECOVERY_BLOCKING_HOLD' if blockers else 'LOCAL_GOAL_HOLD'


def transition_dispatch_allowed(name, direction_owner):
    """Let committed motion finish, but issue no new goal around a recovery owner."""
    return direction_owner is None or name==direction_owner


def transition_failure_scope(code):
    """A settled local hold must not be widened into cancel_all()."""
    return 'local' if code=='LOCAL_GOAL_HOLD' else 'fleet'


def connector_problem_action(problem, elapsed, timeout):
    """Classify connector admission without turning a transient map result into a fleet fault."""
    if not problem:
        return 'clear'
    if 'map connector occupied/unknown' in problem:
        return 'local_hold' if elapsed >= timeout else 'wait'
    return 'wait'


def slot_amcl_request_action(sample_count, request_count, inflight, now,
                             next_request, deadline, required, preferred,
                             max_requests):
    """Advance one bounded per-robot AMCL refresh without a fleet barrier."""
    if sample_count>=preferred:
        return 'verify'
    if now>=deadline:
        return 'verify' if sample_count>=required else 'hold'
    if inflight:
        return 'wait'
    if request_count<max_requests and now>=next_request:
        return 'request'
    if request_count>=max_requests and sample_count>=required:
        return 'verify'
    return 'wait'


def slot_amcl_effective_sample_count(samples, required, minimum_span):
    """Count time-separated preferred observations; retain a single candidate."""
    if len(samples)<required:
        return len(samples)
    receive_times=[item.get('receive_wall',0.) for item in samples]
    return (len(samples) if max(receive_times)-min(receive_times)>=minimum_span
            else required-1)


def slot_certificate_reusable(certificate, goal, pose, stopped,
                              position_tolerance, yaw_tolerance):
    """Only reuse the same slot certificate while its robot remains stopped."""
    if not certificate or not stopped or certificate.get('goal')!=goal:
        return False
    previous=certificate.get('pose')
    if not previous or any(math.isnan(v) or math.isinf(v) for v in pose):
        return False
    yaw_delta=math.atan2(math.sin(pose[2]-previous[2]),
                         math.cos(pose[2]-previous[2]))
    return (math.hypot(pose[0]-previous[0],pose[1]-previous[1])<=position_tolerance
            and abs(yaw_delta)<=yaw_tolerance)


def slot_amcl_queue_admit(queue, name, active_count, limit):
    """A place is occupied until verification completes, not until RPC returns."""
    if name not in queue:
        queue.append(name)
    if active_count>=limit or queue[0]!=name:
        return False
    queue.pop(0)
    return True


def transition_claim_motion_owner(state, owner):
    """Change the single command owner and invalidate every older owner ticket."""
    if owner not in ('IDLE','TEB','CONNECTOR','SLOT_VERIFY','LOCAL_HOLD'):
        raise ValueError('unknown transition motion owner: '+str(owner))
    previous=state.get('motion_owner','IDLE')
    if previous!=owner:
        state['owner_epoch']=int(state.get('owner_epoch',0))+1
        state['motion_owner']=owner
    else:
        state.setdefault('owner_epoch',0)
    return previous,state['owner_epoch']


def transition_owner_ticket(state, target=None):
    return {'owner':state.get('motion_owner','IDLE'),
            'owner_epoch':int(state.get('owner_epoch',0)),
            'target':state.get('target') if target is None else target}


def transition_owner_ticket_valid(state, ticket, owner=None):
    return bool(ticket and
        ticket.get('owner')==(owner or state.get('motion_owner','IDLE')) and
        ticket.get('owner_epoch')==int(state.get('owner_epoch',0)) and
        ticket.get('target')==state.get('target'))


def transition_motion_owner_error(state):
    """Return a compact invariant error when two controllers claim one robot."""
    active=[]
    if state.get('connector'):
        active.append('CONNECTOR')
    if state.get('slot_verify'):
        active.append('SLOT_VERIFY')
    if state.get('slot_local_hold') or state.get('direction_local_hold'):
        active.append('LOCAL_HOLD')
    if (state.get('running') and not state.get('connector') and
            not state.get('goal_settle')):
        active.append('TEB')
    active=list(set(active))
    owner=state.get('motion_owner','IDLE')
    if len(active)>1:
        return 'multiple active owners: '+','.join(sorted(active))
    if active and owner!=active[0]:
        return 'owner %s does not match %s'%(owner,active[0])
    return None


def transition_state_has_reservation(state):
    retry=state.get('teb_retry') or {}
    return bool(state.get('running') or state.get('tracking_fault') or
                state.get('connector') or state.get('recovery_request') or
                state.get('goal_settle') or state.get('direction_recovery') or
                state.get('direction_local_hold') or state.get('slot_verify') or
                state.get('slot_local_hold') or retry.get('waiting'))


def transition_sensor_resend_allowed(name,state,arrived,require_running=False):
    """Do not revive a TEB goal while endpoint/slot handling owns the robot."""
    retry=state.get('teb_retry') or {}
    return bool(state.get('motion_owner','TEB')=='TEB' and
                (state.get('running') or not require_running) and
                name not in arrived and state.get('target') and
                not state.get('connector') and
                not state.get('connector_admission') and
                not retry.get('waiting') and
                not state.get('slot_verify') and
                not state.get('slot_local_hold') and
                not state.get('goal_settle'))


def transition_progress_regression_fault(state,progress,margin):
    """A certified recovery may intentionally return to an older route node."""
    return bool(not state.get('direction_recovery') and
                progress<state.get('maximum',0.)-margin)


def bounded_short_connector_action(pending,now,timeout):
    """Stop an occupied short connector from owning admission forever."""
    if pending is None or now-pending.get('started',now)<timeout:
        return 'wait'
    return 'teb'


def clear_transition_arrival_state(state):
    """Release every transient controller/reservation after slot acceptance."""
    state.update(running=False,needs_resend=False,plan_check=None,
                 plan_accepted=False,teb_guard=None,retry_inflight=False,
                 teb_retry=None,recovery_request=None,tracking_fault=None,
                 direction_recovery=None,direction_local_hold=None,
                 connector=None,connector_admission=None,slot_verify=None,
                 slot_local_hold=None,slot_verify_pending=False,
                 slot_corrections=0)
    for key in ('goal_settle','short_since','entry_drift_since','slot_retry_after'):
        state.pop(key,None)
    if state.get('motion_owner','IDLE')!='IDLE':
        state['owner_epoch']=int(state.get('owner_epoch',0))+1
        state['motion_owner']='IDLE'


def transition_controller_name(state):
    if state.get('slot_local_hold'):
        return 'SLOT_LOCALIZATION_HOLD'
    if state.get('slot_verify'):
        return 'SLOT_AMCL_REFRESH'
    if state.get('direction_local_hold'):
        return 'LOCAL_GOAL_HOLD'
    if state.get('direction_recovery'):
        return ('GLOBAL_PLAN_RECOVERING'
                if state['direction_recovery'].get('kind')=='global_plan'
                else 'DIRECTION_RECOVERING')
    if state.get('connector'):
        return 'connector'
    if state.get('goal_settle'):
        return 'goal_settle'
    if (state.get('teb_retry') or {}).get('waiting'):
        return 'teb_retry'
    if state.get('tracking_fault'):
        return 'tracking_recovery'
    return 'teb' if state['running'] else 'idle'


def yaw_excursion_action(error, limit, now, hold, since):
    """Confirm a yaw excursion before handing only that robot to recovery."""
    if limit<=0 or error<=limit:
        return 'normal',None
    since=now if since is None else since
    return ('recover' if now-since>=hold else 'confirm'),since


def short_connector_action(grid, direct_route, original_route):
    """Prefer a clear connector, otherwise preserve the certified polyline."""
    if grid is None:
        return 'wait'
    if grid.line_free(*direct_route):
        return 'connector'
    if (len(original_route)>=2 and all(grid.line_free(a,b)
            for a,b in zip(original_route,original_route[1:]))):
        return 'teb'
    return 'wait'


def recovery_target_index(task, progress, target, advance):
    """Pick one nearby approved route node for fixed-vector re-entry."""
    candidates=[i for i in range(1,target+1)
                if task['arc'][i]>progress+0.02
                and task['arc'][i]<=progress+advance+1e-8]
    if candidates:
        return candidates[-1]
    candidates=[i for i in range(1,target+1) if task['arc'][i]>=progress-1e-8]
    return candidates[0] if candidates else target


def route_tangent(task, progress, target):
    """Return the forward tangent of the committed route segment."""
    for i in range(1,target+1):
        if task['arc'][i]>progress+0.02:
            a,b=task['path'][i-1:i+1]
            return b[0]-a[0],b[1]-a[1]
    a,b=task['path'][max(0,target-1):target+1]
    return b[0]-a[0],b[1]-a[1]


def transition_goal_status(state, client_state, succeeded_status):
    """Do not interpret an idle, not-yet-dispatched action client as LOST."""
    if state.get('connector_done') or state.get('goal_settle'):
        return succeeded_status
    return (client_state if state.get('running') and
            state.get('motion_owner','TEB')=='TEB' else None)


def vector_command(pose, goal, cfg, previous, dt):
    """One fixed-heading vector tick, shared by entry and mixed short segments."""
    x,y,yaw=pose; dx,dy=goal['x']-x,goal['y']-y
    norm=math.hypot(dx,dy)
    error=math.atan2(math.sin(math.radians(goal['yaw_deg'])-yaw),
                     math.cos(math.radians(goal['yaw_deg'])-yaw))
    arrived=(norm<=cfg.get('position_tolerance',.025)
             and abs(error)<=math.radians(cfg.get('yaw_tolerance_deg',5.0)))
    if arrived:
        return (0.,0.,0.),True
    limit=float(cfg.get('speed',.05))
    speed=min(limit,max(float(cfg.get('min_speed',.025)),.8*norm))
    vx,vy=dx/max(norm,1e-9)*speed,dy/max(norm,1e-9)*speed
    angular=float(cfg.get('max_angular',.08))
    desired=(math.cos(yaw)*vx+math.sin(yaw)*vy,
             -math.sin(yaw)*vx+math.cos(yaw)*vy,
             max(-angular,min(angular,float(cfg.get('yaw_gain',1.2))*error)))
    acc=[float(cfg.get(k,d)) for k,d in
         (('acc_lim_x',.10),('acc_lim_y',.10),('acc_lim_theta',.15))]
    dt=max(.001,min(.20,dt))
    return tuple(max(p-a*dt,min(p+a*dt,v)) for p,a,v in zip(previous,acc,desired)),False


class FleetVendorOmniSupervisor(object):

    RUNTIME_PARAMETER_SUFFIXES = {
        "imu_bias_estimation": "imu_filter/do_bias_estimation",
        "max_vel_x": "move_base/TebLocalPlannerROS/max_vel_x",
        "max_vel_y": "move_base/TebLocalPlannerROS/max_vel_y",
        "max_vel_x_backwards": (
            "move_base/TebLocalPlannerROS/max_vel_x_backwards"),
        "max_vel_theta": "move_base/TebLocalPlannerROS/max_vel_theta",
        "acc_lim_x": "move_base/TebLocalPlannerROS/acc_lim_x",
        "acc_lim_y": "move_base/TebLocalPlannerROS/acc_lim_y",
        "acc_lim_theta": "move_base/TebLocalPlannerROS/acc_lim_theta",
        "amcl_update_min_d": "amcl/update_min_d",
        "amcl_update_min_a": "amcl/update_min_a",
        "amcl_recovery_alpha_slow": "amcl/recovery_alpha_slow",
        "amcl_recovery_alpha_fast": "amcl/recovery_alpha_fast",
    }

    ACTIVE_GOAL_STATES = (
        GoalStatus.PENDING,
        GoalStatus.ACTIVE,
        GoalStatus.PREEMPTING,
        GoalStatus.RECALLING,
    )

    def __init__(self):
        rospy.init_node("fleet_vendor_omni")
        self.lock = threading.RLock()
        self.master_lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.stop_requested = threading.Event()
        self.motion_enabled = bool(rospy.get_param("~motion_enabled", False))
        self.allow_full_run = bool(rospy.get_param("~allow_full_run", False))
        self.sp_direction_guard_enabled = bool(rospy.get_param(
            "~sp_direction_guard_enabled", True))
        self.slot_amcl_verify_enabled = bool(rospy.get_param(
            "~slot_amcl_verify_enabled", False))
        self.slot_amcl_max_parallel = max(1, min(3, int(rospy.get_param(
            "~slot_amcl_max_parallel", 3))))
        self.map_frame = str(rospy.get_param("~map_frame", "robot_1/map")).strip("/")
        self.robot_order = list(rospy.get_param(
            "~robot_order", ["robot_1", "robot_2", "robot_3"]))
        self.robot_config = rospy.get_param("~robots", {})
        self.runtime_profiles = rospy.get_param("~runtime_profiles", {})
        self.stages = rospy.get_param("~stages", [])
        self.required_parameters = flatten_parameter_tree(rospy.get_param(
            "~required_parameter_suffixes", {}))
        self.checkpoint_config = rospy.get_param("~checkpoint", {})
        self.formation_line_config = rospy.get_param("~formation_line", {})
        self.validate_config()
        self.transition_grid = (MapGrid.load(rospy.get_param('~transition_map_yaml'), .34)
            if any(s.get('scheduler')=='progress_gates' for s in self.stages) else None)

        cfg = self.checkpoint_config
        self.settle_stable_duration = float(cfg.get("settle_stable_duration", 0.8))
        self.settle_timeout = float(cfg.get("settle_timeout", 4.0))
        self.cmd_linear_threshold = float(cfg.get("cmd_linear_threshold", 0.005))
        self.cmd_angular_threshold = float(cfg.get("cmd_angular_threshold", 0.01))
        self.odom_linear_threshold = float(cfg.get("odom_linear_threshold", 0.01))
        self.odom_angular_threshold = float(cfg.get("odom_angular_threshold", 0.02))
        self.imu_wz_threshold = float(cfg.get("imu_wz_threshold", 0.02))
        self.sensor_max_age = float(cfg.get("sensor_max_age", 0.8))
        self.transition_sensor_recovery_timeout = float(
            cfg.get("transition_sensor_recovery_timeout", 3.5))
        self.samples_required = max(1, int(cfg.get("amcl_samples_required", 1)))
        self.samples_preferred = max(
            self.samples_required, int(cfg.get("amcl_samples_preferred", 2)))
        self.nomotion_max_requests = max(
            self.samples_preferred, int(cfg.get("nomotion_max_requests", 3)))
        self.nomotion_request_interval = float(
            cfg.get("nomotion_request_interval", 0.35))
        self.nomotion_service_timeout = float(
            cfg.get("nomotion_service_timeout", 1.0))
        self.amcl_callback_timeout = float(cfg.get("amcl_callback_timeout", 1.0))
        self.checkpoint_timeout = float(cfg.get("checkpoint_timeout", 6.0))
        self.sample_position_spread = float(cfg.get("sample_position_spread", 0.03))
        self.sample_yaw_spread = math.radians(float(
            cfg.get("sample_yaw_spread_deg", 2.0)))
        self.expected_position_error = float(cfg.get("expected_position_error", 0.10))
        self.expected_yaw_error = math.radians(float(
            cfg.get("expected_yaw_error_deg", 5.0)))
        self.tf_settle_time = float(cfg.get("tf_settle_time", 0.30))
        self.tf_position_error = float(cfg.get("tf_position_error", 0.04))
        self.tf_yaw_error = math.radians(float(cfg.get("tf_yaw_error_deg", 2.0)))
        self.initial_position_error = float(cfg.get("initial_position_error", 0.15))
        self.initial_yaw_error = math.radians(float(
            cfg.get("initial_yaw_error_deg", 8.0)))
        self.yaw_excursion_guard = math.radians(float(
            cfg.get("yaw_excursion_guard_deg", 0.0)))
        self.yaw_excursion_hold = float(
            cfg.get("yaw_excursion_guard_hold", 0.5))
        self.goal_teammate_min_distance = float(
            cfg.get("goal_teammate_min_distance", 0.0))
        self.runtime_teammate_min_distance = float(
            cfg.get("runtime_teammate_min_distance", 0.0))
        self.preflight_cache_max_age = float(
            cfg.get("preflight_cache_max_age", 2.5))
        self.slot_amcl_timeout = float(cfg.get(
            "slot_amcl_timeout", self.transition_sensor_recovery_timeout))
        self.slot_amcl_max_corrections = max(1, int(cfg.get(
            "slot_amcl_max_corrections", 2)))
        # One fresh post-stop frame is sufficient when it satisfies the
        # existing pose/TF/scan checks; a second frame remains preferred.
        self.slot_amcl_samples_required = max(1, self.samples_required)
        self.slot_amcl_min_observation_span = self.settle_stable_duration

        self.latest = dict((name, {}) for name in self.robot_order)
        self.clients = dict((name, actionlib.SimpleActionClient(
            "/%s/move_base" % name, MoveBaseAction)) for name in self.robot_order)
        self.via_pubs = {}
        if any(s.get('scheduler')=='progress_gates' for s in self.stages):
            self.via_pubs = dict((name,rospy.Publisher(
                '/%s/move_base/TebLocalPlannerROS/via_points'%name,Path,queue_size=1))
                for name in self.robot_order)
        self.nomotion_clients = dict((name, rospy.ServiceProxy(
            "/%s/request_nomotion_update" % name, Empty))
            for name in self.robot_order)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.master = rosgraph.Master(rospy.get_name())
        self.expected = self.initial_expected()

        self.state = "STARTING"
        self.stage_index = 0
        self.active = False
        self.last_error = None
        self.last_preflight = ["startup checks have not completed"]
        self.preflight_checked_wall = 0.0
        self.static_preflight = ["static checks have not completed"]
        self.static_preflight_complete = False
        self.startup_nomotion_attempts = dict(
            (name, 0) for name in self.robot_order)
        self.startup_nomotion_inflight = set()
        self.checkpoint_state = {"state": "IDLE"}
        self.pending_expected = None
        self.stage_motion_complete = False
        self.worker = None
        # Keep certificates for the current supervisor process so a manual
        # start_all can resume an untouched stage without re-verifying it.
        self.slot_certificates = {}

        self.state_pub = rospy.Publisher("~state_json", String, queue_size=1, latch=True)
        self.event_pub = rospy.Publisher("~event_json", String, queue_size=20)
        for name in self.robot_order:
            self.register_robot(name)
        rospy.Service("~start_next", Trigger, self.start_next)
        rospy.Service("~start_all", Trigger, self.start_all)
        rospy.Service("~stop", Trigger, self.stop)
        rospy.Service("~reset", Trigger, self.reset)
        rospy.on_shutdown(self.shutdown)
        rospy.Timer(rospy.Duration(1.0), self.health_timer)
        self.publish_state()

    def validate_config(self):
        if not self.robot_order or len(set(self.robot_order)) != len(self.robot_order):
            raise rospy.ROSException("~robot_order must contain unique robot names")
        if not isinstance(self.robot_config, dict):
            raise rospy.ROSException("~robots must be a dictionary")
        if not isinstance(self.runtime_profiles, dict):
            raise rospy.ROSException("~runtime_profiles must be a dictionary")
        for name in self.robot_order:
            if name not in self.robot_config:
                raise rospy.ROSException("missing robot config for %s" % name)
            for key in ("initial_x", "initial_y", "initial_yaw_deg"):
                if key not in self.robot_config[name]:
                    raise rospy.ROSException("%s is missing %s" % (name, key))
            profile = self.runtime_profiles.get(name)
            if not isinstance(profile, dict):
                raise rospy.ROSException("missing runtime profile for %s" % name)
            missing = [key for key in self.RUNTIME_PARAMETER_SUFFIXES
                       if key not in profile]
            if "fuse_imu_yaw" not in profile:
                missing.append("fuse_imu_yaw")
            if missing:
                raise rospy.ROSException("%s runtime profile is missing %s" % (
                    name, ", ".join(sorted(missing))))
        if not isinstance(self.stages, list) or not self.stages:
            raise rospy.ROSException("~stages must be a non-empty list")
        names = set()
        for index, stage in enumerate(self.stages):
            if not isinstance(stage, dict) or not stage.get("name"):
                raise rospy.ROSException("invalid stage %d" % index)
            if stage["name"] in names:
                raise rospy.ROSException("duplicate stage %s" % stage["name"])
            names.add(stage["name"])
            if stage.get("mode", "move_base") not in (
                    "move_base", "formation_line", "transition"):
                raise rospy.ROSException(
                    "invalid mode in stage %s" % stage["name"])
            goals = stage.get("goals", {})
            if not isinstance(goals, dict) or not goals:
                raise rospy.ROSException("stage %s has no goals" % stage["name"])
            for robot_name, goal in goals.items():
                if robot_name not in self.robot_order:
                    raise rospy.ROSException("unknown robot %s" % robot_name)
                if not all(key in goal for key in ("x", "y", "yaw_deg")):
                    raise rospy.ROSException("incomplete goal in %s" % stage["name"])
            if stage.get('mode') == 'transition':
                if stage.get('scheduler') == 'progress_gates':
                    try:
                        entry=float(stage.get('entry_tolerance',.025))
                        entry_release=float(stage.get('entry_release_tolerance',entry+.01))
                        minimum=float(stage.get('min_teb_distance',.25))
                        if (not 0.0<entry<entry_release<stage['tracking_error']
                                or not .15<=minimum<=.5):
                            raise ValueError('Invalid transition entry/minimum TEB distance')
                        validate_gates(stage['tasks'],goals)
                        for key in ('tracking_error','tracking_error_hard',
                                    'tracking_error_hold','tracking_recovery_wait',
                                    'route_reservation','parked_clearance','release_margin'):
                            value=float(stage[key])
                            if math.isnan(value) or math.isinf(value) or value<=0:
                                raise ValueError('Invalid gated '+key)
                        minimum=float(self.checkpoint_config.get('runtime_teammate_min_distance',0.0))
                        if minimum<=0 or stage['release_margin']<0.05 or stage['parked_clearance']<minimum+stage['tracking_error']+0.04:
                            raise ValueError('Gated tracking/parked envelope is inconsistent')
                        if stage['route_reservation']<minimum+2*stage['tracking_error']+.04:
                            raise ValueError('Gated moving/moving envelope is inconsistent')
                        if not (stage['tracking_error']<stage['tracking_error_hard']
                                <=(stage['route_reservation']-minimum)/2.0):
                            raise ValueError('Gated tracking recovery envelope is inconsistent')
                        if not (0<float(stage['teb_direction_min_speed'])<=.10
                                and -1.<=float(stage['teb_direction_min_cosine'])<=1.
                                and .02<float(stage['route_recovery_advance'])
                                <=float(stage['min_teb_distance'])
                                and float(stage['route_recovery_timeout'])>0):
                            raise ValueError('Invalid TEB direction/route recovery parameters')
                    except (KeyError,TypeError,ValueError) as exc:
                        raise rospy.ROSException(str(exc))
                    continue
                if stage.get('scheduler','whole_route')!='whole_route':
                    raise rospy.ROSException('Unknown transition scheduler')
                tasks = stage.get('tasks', [])
                seen = set()
                robots = set()
                for task in tasks:
                    if (task['id'] in seen or task['robot'] in robots
                            or task['robot'] not in goals or len(task['path']) < 2
                            or any(dep not in seen for dep in task.get('after', []))):
                        raise rospy.ROSException('invalid transition dependency/path: '+stage['name'])
                    if distance(task['path'][-1], (goals[task['robot']]['x'],goals[task['robot']]['y'])) > 0.001:
                        raise rospy.ROSException('transition endpoint does not match goal')
                    seen.add(task['id']); robots.add(task['robot'])
                if robots != set(goals):
                    raise rospy.ROSException('transition tasks do not cover goals')

    def initial_expected(self):
        result = {}
        for name in self.robot_order:
            item = self.robot_config[name]
            result[name] = {
                "x": float(item["initial_x"]),
                "y": float(item["initial_y"]),
                "yaw": math.radians(float(item["initial_yaw_deg"])),
            }
        return result

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def wrap_angle(value):
        return math.atan2(math.sin(value), math.cos(value))

    @staticmethod
    def values_equal(actual, expected):
        if isinstance(expected, bool):
            return isinstance(actual, bool) and actual == expected
        if isinstance(expected, (int, float)) and not isinstance(expected, bool):
            try:
                return abs(float(actual) - float(expected)) <= 1e-6
            except (TypeError, ValueError):
                return False
        return str(actual) == str(expected)

    def register_robot(self, name):
        rospy.Subscriber("/%s/amcl_pose" % name, PoseWithCovarianceStamped,
                         self.amcl_callback, callback_args=name, queue_size=10)
        rospy.Subscriber("/%s/odom" % name, Odometry,
                         self.odom_callback, callback_args=name, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/%s/imu" % name, Imu,
                         self.imu_callback, callback_args=name, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/%s/scan" % name, LaserScan,
                         self.scan_callback, callback_args=name, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/%s/jetauto_controller/cmd_vel" % name, Twist,
                         self.cmd_callback, callback_args=name, queue_size=20)
        rospy.Subscriber("/%s/move_base/GlobalPlanner/plan" % name, Path,
                         self.global_plan_callback, callback_args=name, queue_size=1)

    @staticmethod
    def amcl_message_error(msg):
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        covariance = msg.pose.covariance
        values = [msg.header.stamp.to_sec(), position.x, position.y, position.z,
                  q.x, q.y, q.z, q.w] + list(covariance)
        if any(math.isnan(value) or math.isinf(value) for value in values):
            return "non-finite pose, quaternion, stamp or covariance"
        if abs(sum(value * value for value in (q.x, q.y, q.z, q.w)) - 1.0) > 1e-3:
            return "invalid quaternion norm"
        # AMCL output moments can round to tiny negative variances (~1e-16).
        # This tolerance is NOT used for initial_cov_* particle sampling.
        if len(covariance) != 36 or any(covariance[i] < -1e-12
                                       for i in (0, 7, 14, 21, 28, 35)):
            return "invalid covariance diagonal"
        return None

    def amcl_callback(self, msg, name):
        with self.condition:
            error = self.amcl_message_error(msg)
            if error:
                # Do not retain a previous good pose after a new invalid frame.
                self.latest[name].pop("amcl", None)
                self.latest[name]["amcl_invalid"] = {
                    "reason": error, "receive_wall": time.time()}
                self.condition.notify_all()
                return
            self.latest[name].pop("amcl_invalid", None)
            self.latest[name]["amcl"] = {
                "receive_wall": time.time(),
                "stamp": msg.header.stamp.to_sec(),
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "yaw": self.yaw_from_quaternion(msg.pose.pose.orientation),
                "cov_x": msg.pose.covariance[0],
                "cov_y": msg.pose.covariance[7],
                "cov_yaw": msg.pose.covariance[35],
            }
            self.condition.notify_all()

    def odom_callback(self, msg, name):
        with self.lock:
            self.latest[name]["odom"] = {
                "receive_wall": time.time(), "stamp": msg.header.stamp.to_sec(),
                "vx": msg.twist.twist.linear.x,
                "vy": msg.twist.twist.linear.y,
                "wz": msg.twist.twist.angular.z,
            }
            if (math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y)>
                    self.odom_linear_threshold or
                    abs(msg.twist.twist.angular.z)>self.odom_angular_threshold):
                for certificates in self.slot_certificates.values():
                    certificates.pop(name,None)

    def imu_callback(self, msg, name):
        with self.lock:
            self.latest[name]["imu"] = {
                "receive_wall": time.time(), "stamp": msg.header.stamp.to_sec(),
                "wz": msg.angular_velocity.z,
            }

    def scan_callback(self, msg, name):
        with self.lock:
            self.latest[name]["scan"] = {
                "receive_wall": time.time(), "stamp": msg.header.stamp.to_sec(),
                "frame": msg.header.frame_id.strip("/"),
                "message": msg,
            }

    def cmd_callback(self, msg, name):
        with self.lock:
            self.latest[name]["cmd"] = {
                "receive_wall": time.time(), "vx": msg.linear.x,
                "vy": msg.linear.y, "wz": msg.angular.z,
            }
            if (math.hypot(msg.linear.x,msg.linear.y)>self.cmd_linear_threshold or
                    abs(msg.angular.z)>self.cmd_angular_threshold):
                for certificates in self.slot_certificates.values():
                    certificates.pop(name,None)

    def global_plan_callback(self, msg, name):
        with self.lock:
            self.latest[name]["global_plan"] = {
                "receive_wall": time.time(), "stamp": msg.header.stamp.to_sec(),
                "frame": msg.header.frame_id, "points": len(msg.poses),
            }

    def graph_state(self):
        with self.master_lock:
            publishers, _subscribers, services = self.master.getSystemState()
        return (
            dict((topic, list(nodes)) for topic, nodes in publishers),
            dict((service, list(nodes)) for service, nodes in services),
        )

    def lookup_ready(self, target, source):
        try:
            return self.tf_buffer.can_transform(
                target, source, rospy.Time(0), rospy.Duration(0.0))
        except Exception:
            return False

    def check_static_parameters(self):
        errors = []
        for name in self.robot_order:
            try:
                parameters = rospy.get_param("/%s" % name)
            except Exception as exc:
                errors.append("cannot read /%s parameters: %s" % (name, exc))
                continue
            for suffix, expected in self.required_parameters:
                parameter = "/%s/%s" % (name, suffix.strip("/"))
                try:
                    actual = parameter_from_tree(parameters, suffix)
                except KeyError:
                    errors.append("missing parameter %s" % parameter)
                    continue
                if not self.values_equal(actual, expected):
                    errors.append("parameter %s=%r expected %r" % (
                        parameter, actual, expected))
            profile = self.runtime_profiles[name]
            for key, suffix in self.RUNTIME_PARAMETER_SUFFIXES.items():
                parameter = "/%s/%s" % (name, suffix)
                try:
                    actual = parameter_from_tree(parameters, suffix)
                except KeyError:
                    errors.append("missing parameter %s" % parameter)
                    continue
                expected = profile[key]
                if not self.values_equal(actual, expected):
                    errors.append("parameter %s=%r expected %r" % (
                        parameter, actual, expected))
            try:
                imu_config = parameter_from_tree(
                    parameters, "ekf_localization/imu0_config")
            except KeyError:
                imu_config = None
            if not isinstance(imu_config, list) or len(imu_config) < 12:
                errors.append("%s missing EKF imu0_config" % name)
            else:
                expected_yaw = bool(profile["fuse_imu_yaw"])
                if bool(imu_config[5]) != expected_yaw or not bool(imu_config[11]):
                    errors.append(
                        "%s EKF yaw contract mismatch: yaw=%r expected %r, "
                        "yaw_rate=%r expected True" % (
                            name, bool(imu_config[5]), expected_yaw,
                            bool(imu_config[11])))
        return errors

    def prime_startup_amcl(self):
        for name in self.robot_order:
            if self.lookup_ready(self.map_frame, "%s/odom" % name):
                continue
            if self.startup_nomotion_attempts[name] >= self.nomotion_max_requests:
                continue
            with self.lock:
                if name in self.startup_nomotion_inflight:
                    continue
                self.startup_nomotion_inflight.add(name)
                self.startup_nomotion_attempts[name] += 1
            thread = threading.Thread(
                target=self.request_startup_nomotion, args=(name,))
            thread.daemon = True
            thread.start()

    def request_startup_nomotion(self, name):
        service = "/%s/request_nomotion_update" % name
        try:
            rospy.wait_for_service(service, timeout=0.05)
            self.nomotion_clients[name]()
            rospy.loginfo("requested startup AMCL update for %s", name)
        except Exception:
            pass
        finally:
            with self.lock:
                self.startup_nomotion_inflight.discard(name)

    def preflight(self):
        errors = []
        now = time.time()
        graph_error = None
        try:
            graph_publishers, graph_services = self.graph_state()
        except Exception as exc:
            graph_publishers, graph_services = {}, {}
            graph_error = str(exc)
            errors.append("cannot inspect ROS graph: %s" % exc)
        for name in self.robot_order:
            if not self.clients[name].wait_for_server(rospy.Duration(0.01)):
                errors.append("%s move_base unavailable" % name)
            service = "/%s/request_nomotion_update" % name
            if graph_error is None and not graph_services.get(service):
                errors.append("%s no-motion service unavailable" % name)
            with self.lock:
                latest = dict(self.latest[name])
            # An idle move_base may not publish cmd_vel until its first goal.
            # Command ownership is checked below; live motion telemetry is
            # required only by the post-goal stationary gate.
            for key in ("amcl", "odom", "imu", "scan"):
                item = latest.get(key)
                if item is None:
                    if key == "amcl" and latest.get("amcl_invalid"):
                        errors.append("%s invalid AMCL: %s" % (
                            name, latest["amcl_invalid"]["reason"]))
                    else:
                        errors.append("%s missing %s" % (name, key))
                elif key != "amcl" and now - item["receive_wall"] > self.sensor_max_age:
                    errors.append("%s stale %s" % (name, key))
            for target, source in (
                    (self.map_frame, "%s/odom" % name),
                    ("%s/odom" % name, "%s/base_footprint" % name),
                    (self.map_frame, "%s/base_footprint" % name)):
                if not self.lookup_ready(target, source):
                    errors.append("TF unavailable: %s <- %s" % (target, source))
            scan = latest.get("scan")
            if (scan is not None
                    and not self.lookup_ready("%s/odom" % name, scan["frame"])):
                errors.append("%s scan TF unavailable" % name)
            if graph_error is None:
                topic = "/%s/jetauto_controller/cmd_vel" % name
                publishers = graph_publishers.get(topic, [])
                expected_publisher = "/%s/move_base" % name
                if publishers != [expected_publisher]:
                    errors.append("%s command publishers=%s" % (name, publishers))

        if graph_error is None:
            map_publishers = graph_publishers.get("/"+self.map_frame, [])
            if len(map_publishers) != 1:
                errors.append("map publisher count=%d" % len(map_publishers))

        if (self.stage_index == 0 and not self.stage_motion_complete
                and self.state not in ("GOAL_HOLD", "LOCALIZATION_HOLD")):
            for name in self.robot_order:
                with self.lock:
                    sample = self.latest[name].get("amcl")
                if sample is None:
                    continue
                expected = self.expected[name]
                position_error = math.hypot(
                    sample["x"] - expected["x"], sample["y"] - expected["y"])
                yaw_error = abs(self.wrap_angle(sample["yaw"] - expected["yaw"]))
                if position_error > self.initial_position_error:
                    errors.append("%s initial position error %.3f" % (
                        name, position_error))
                if yaw_error > self.initial_yaw_error:
                    errors.append("%s initial yaw error %.1f deg" % (
                        name, math.degrees(yaw_error)))
        return errors

    def state_value(self):
        with self.lock:
            next_stage = (self.stages[self.stage_index]
                          if self.stage_index < len(self.stages) else None)
            robots = {}
            now = time.time()
            for name in self.robot_order:
                values = self.latest[name]
                robots[name] = dict((key + "_age", round(
                    now - item["receive_wall"], 3))
                    for key, item in values.items() if "receive_wall" in item)
                if values.get("amcl"):
                    robots[name]["amcl"] = {
                        "x": round(values["amcl"]["x"], 4),
                        "y": round(values["amcl"]["y"], 4),
                        "yaw_deg": round(math.degrees(values["amcl"]["yaw"]), 2),
                        "stamp": values["amcl"]["stamp"],
                    }
                if values.get("amcl_invalid"):
                    robots[name]["amcl_invalid"] = values["amcl_invalid"]["reason"]
            return {
                "state": self.state, "active": self.active,
                "motion_enabled": self.motion_enabled,
                "allow_full_run": self.allow_full_run,
                "stage_index": self.stage_index, "stage_count": len(self.stages),
                "next_stage": next_stage,
                "stage_motion_complete": self.stage_motion_complete,
                "last_error": self.last_error,
                "preflight_errors": list(self.last_preflight),
                "preflight_age": (
                    round(now - self.preflight_checked_wall, 3)
                    if self.preflight_checked_wall > 0.0 else None),
                "checkpoint": dict(self.checkpoint_state),
                "robots": robots, "ros_time": rospy.Time.now().to_sec(),
            }

    def publish_state(self):
        self.state_pub.publish(String(data=json.dumps(
            self.state_value(), sort_keys=True)))

    def publish_event(self, kind, **values):
        event = {
            "kind": kind, "stage_index": self.stage_index,
            "ros_time": rospy.Time.now().to_sec(), "wall_time": time.time(),
        }
        event.update(values)
        self.event_pub.publish(String(data=json.dumps(event, sort_keys=True)))
        rospy.loginfo("fleet vendor event: %s", json.dumps(event, sort_keys=True))

    def health_timer(self, _event):
        with self.lock:
            active = self.active
            state = self.state
        if not active and state != "SUCCEEDED":
            if not self.static_preflight_complete:
                static_errors = self.check_static_parameters()
                with self.lock:
                    self.static_preflight = static_errors
                    self.static_preflight_complete = True
            self.prime_startup_amcl()
            errors = list(self.static_preflight) + self.preflight()
            with self.lock:
                self.last_preflight = errors
                self.preflight_checked_wall = time.time()
                if self.state in ("STARTING", "READY", "NOT_READY"):
                    self.state = "READY" if not errors else "NOT_READY"
                    if not errors:
                        self.last_error = None
        self.publish_state()

    def cached_preflight_errors(self):
        with self.lock:
            checked_wall = self.preflight_checked_wall
            errors = list(self.last_preflight)
            errors.extend("%s invalid AMCL: %s" % (
                name, values["amcl_invalid"]["reason"])
                for name, values in self.latest.items() if values.get("amcl_invalid"))
        if checked_wall <= 0.0:
            return ["preflight has not completed"]
        age = time.time() - checked_wall
        if age > self.preflight_cache_max_age:
            return ["preflight status stale (%.2fs)" % age]
        return errors

    def stationary_snapshot(self):
        now = time.time()
        detail = {}
        all_stopped = True
        with self.lock:
            latest = dict((name, dict(values))
                          for name, values in self.latest.items())
        for name in self.robot_order:
            values = latest[name]
            cmd = values.get("cmd")
            odom = values.get("odom")
            imu = values.get("imu")
            scan = values.get("scan")
            item = {"cmd": cmd, "odom": odom, "imu": imu, "scan": scan}
            if odom is None or imu is None or scan is None:
                item["reason"] = "motion telemetry missing"
                all_stopped = False
            elif any(now - value["receive_wall"] > self.sensor_max_age
                     for value in (odom, imu, scan)):
                item["reason"] = "sensor telemetry stale"
                all_stopped = False
            else:
                cmd_is_fresh = (
                    cmd is not None
                    and now - cmd["receive_wall"] <= self.sensor_max_age)
                effective_cmd = cmd if cmd_is_fresh else {
                    "vx": 0.0, "vy": 0.0, "wz": 0.0,
                }
                item["cmd_status"] = "fresh" if cmd_is_fresh else "idle_or_stale"
                stopped = (
                    max(abs(effective_cmd["vx"]), abs(effective_cmd["vy"]))
                    <= self.cmd_linear_threshold
                    and abs(effective_cmd["wz"]) <= self.cmd_angular_threshold
                    and max(abs(odom["vx"]), abs(odom["vy"])) <= self.odom_linear_threshold
                    and abs(odom["wz"]) <= self.odom_angular_threshold
                    and abs(imu["wz"]) <= self.imu_wz_threshold)
                if not stopped:
                    item["reason"] = "motion remains above stop gate"
                    all_stopped = False
            detail[name] = item
        return all_stopped, detail

    def wait_for_stationary(self):
        started = time.time()
        stable_since = None
        last_detail = {}
        while not rospy.is_shutdown() and time.time() - started < self.settle_timeout:
            if self.stop_requested.is_set():
                return False, "STOP_REQUESTED", last_detail
            stopped, last_detail = self.stationary_snapshot()
            now = time.time()
            if stopped:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= self.settle_stable_duration:
                    return True, "SETTLED", last_detail
            else:
                stable_since = None
            self.stop_requested.wait(0.05)
        return False, "SETTLE_TIMEOUT", last_detail

    def call_nomotion(self, name):
        try:
            self.nomotion_clients[name]()
            return "OK"
        except Exception as exc:
            return "AMCL_SERVICE_FAILED: %s" % exc

    def amcl_timeout_reason(self, name, service_status):
        now = time.time()
        with self.lock:
            if self.latest[name].get("amcl_invalid"):
                return "AMCL_INVALID"
            scan = dict(self.latest[name].get("scan", {}))
        if not scan or now - scan["receive_wall"] > self.sensor_max_age:
            return "SCAN_STALE"
        try:
            stamp = (rospy.Time.from_sec(scan["stamp"])
                     if scan["stamp"] > 0.0 else rospy.Time(0))
            self.tf_buffer.lookup_transform(
                "%s/odom" % name, scan["frame"], stamp, rospy.Duration(0.05))
        except Exception:
            return "TF_AT_SCAN_MISSING"
        if service_status != "OK":
            return service_status.split(":", 1)[0]
        return "AMCL_CALLBACK_LATE"

    def collect_amcl_samples(self, robot_names):
        with self.lock:
            initial = dict((name, self.latest[name].get("amcl"))
                           for name in robot_names)
        seen = dict((name, set([sample["stamp"]]) if sample else set())
                    for name, sample in initial.items())
        samples = dict((name, []) for name in robot_names)
        deadline = time.time() + self.checkpoint_timeout
        last_status = dict((name, "AMCL_CALLBACK_LATE") for name in robot_names)

        for request_index in range(self.nomotion_max_requests):
            targets = [name for name in robot_names
                       if len(samples[name]) < self.samples_preferred]
            if not targets or time.time() >= deadline:
                break
            request_started = dict((name, time.time()) for name in targets)
            service_results = {}
            threads = []

            def call_one(robot_name):
                service_results[robot_name] = self.call_nomotion(robot_name)

            for name in targets:
                thread = threading.Thread(target=call_one, args=(name,))
                thread.daemon = True
                thread.start()
                threads.append(thread)
            for thread in threads:
                thread.join(self.nomotion_service_timeout + 0.2)

            wait_deadline = min(deadline, time.time() + self.amcl_callback_timeout)
            with self.condition:
                while time.time() < wait_deadline and not self.stop_requested.is_set():
                    progress = False
                    for name in targets:
                        sample = self.latest[name].get("amcl")
                        if (sample and sample["receive_wall"] >= request_started[name]
                                and sample["stamp"] not in seen[name]):
                            seen[name].add(sample["stamp"])
                            samples[name].append(dict(sample))
                            progress = True
                    if all(len(samples[name]) >= self.samples_preferred
                           for name in robot_names):
                        break
                    if not progress:
                        self.condition.wait(min(0.05, wait_deadline - time.time()))

            for name in targets:
                status = service_results.get(name, "AMCL_SERVICE_TIMEOUT")
                if len(samples[name]) < self.samples_required:
                    last_status[name] = self.amcl_timeout_reason(name, status)
                self.publish_event(
                    "AMCL_NOMOTION_REQUEST", robot=name,
                    request_index=request_index + 1, service_status=status,
                    sample_count=len(samples[name]))
            if request_index + 1 < self.nomotion_max_requests:
                self.stop_requested.wait(self.nomotion_request_interval)

        with self.lock:
            invalid = dict((name, self.latest[name]["amcl_invalid"]["reason"])
                           for name in robot_names
                           if self.latest[name].get("amcl_invalid"))
        if invalid:
            return False, "AMCL_INVALID", samples, {"invalid": invalid}
        missing = [name for name in robot_names
                   if len(samples[name]) < self.samples_required]
        if missing:
            code = last_status[missing[0]]
            return False, code, samples, {
                "missing": missing, "status": last_status,
                "counts": dict((name, len(value)) for name, value in samples.items()),
            }
        return True, "AMCL_SAMPLES_READY", samples, {}

    def verify_samples(self, samples, expected, robot_names):
        metrics = {}
        failures = []
        means = {}
        for name in robot_names:
            values = samples[name]
            count = float(len(values))
            mean_x = sum(item["x"] for item in values) / count
            mean_y = sum(item["y"] for item in values) / count
            mean_yaw = math.atan2(
                sum(math.sin(item["yaw"]) for item in values) / count,
                sum(math.cos(item["yaw"]) for item in values) / count)
            position_spread = max(math.hypot(
                item["x"] - mean_x, item["y"] - mean_y) for item in values)
            yaw_spread = max(abs(self.wrap_angle(
                item["yaw"] - mean_yaw)) for item in values)
            goal_position_error = math.hypot(
                mean_x - expected[name]["x"], mean_y - expected[name]["y"])
            goal_yaw_error = abs(self.wrap_angle(
                mean_yaw - expected[name]["yaw"]))
            means[name] = (mean_x, mean_y, mean_yaw)
            metrics[name] = {
                "sample_count": len(values),
                "sample_status": ("preferred" if len(values) >= self.samples_preferred
                                  else "minimum"),
                "mean_x": round(mean_x, 4), "mean_y": round(mean_y, 4),
                "mean_yaw_deg": round(math.degrees(mean_yaw), 3),
                "position_spread": round(position_spread, 4),
                "yaw_spread_deg": round(math.degrees(yaw_spread), 3),
                "expected_position_error": round(goal_position_error, 4),
                "expected_yaw_error_deg": round(math.degrees(goal_yaw_error), 3),
            }
            if (position_spread > self.sample_position_spread
                    or yaw_spread > self.sample_yaw_spread):
                failures.append(("AMCL_UNSTABLE", name))
            if (goal_position_error > self.expected_position_error
                    or goal_yaw_error > self.expected_yaw_error):
                failures.append(("AMCL_GOAL_DISAGREEMENT", name))

        if self.stop_requested.wait(self.tf_settle_time):
            return False, "STOP_REQUESTED", metrics
        for name in robot_names:
            try:
                tf_x, tf_y, tf_yaw = self.robot_pose(name, 0.10)
                mean_x, mean_y, mean_yaw = means[name]
                tf_position_error = math.hypot(tf_x - mean_x, tf_y - mean_y)
                tf_yaw_error = abs(self.wrap_angle(tf_yaw - mean_yaw))
                metrics[name]["tf_position_error"] = round(tf_position_error, 4)
                metrics[name]["tf_yaw_error_deg"] = round(
                    math.degrees(tf_yaw_error), 3)
                if (tf_position_error > self.tf_position_error
                        or tf_yaw_error > self.tf_yaw_error):
                    failures.append(("TF_APPLY_MISMATCH", name))
            except Exception as exc:
                metrics[name]["tf_error"] = str(exc)
                failures.append(("TF_AFTER_AMCL_MISSING", name))
        if failures:
            code, name = failures[0]
            metrics["failed_robot"] = name
            return False, code, metrics
        return True, "CHECKPOINT_PASSED", metrics

    def run_checkpoint(self, expected, moving):
        with self.lock:
            self.state = "CHECKPOINT"
            self.checkpoint_state = {"state": "SETTLING"}
        self.publish_state()
        settled, code, detail = self.wait_for_stationary()
        if not settled:
            return False, code, detail
        self.publish_event("CHECKPOINT_SETTLED")
        with self.lock:
            self.checkpoint_state = {"state": "COLLECTING_AMCL"}
        ready, code, samples, detail = self.collect_amcl_samples(moving)
        if not ready:
            return False, code, detail
        return self.verify_samples(samples, expected, moving)

    def build_goal(self, raw):
        yaw = math.radians(float(raw["yaw_deg"]))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(raw["x"])
        goal.target_pose.pose.position.y = float(raw["y"])
        goal.target_pose.pose.orientation.z = math.sin(0.5 * yaw)
        goal.target_pose.pose.orientation.w = math.cos(0.5 * yaw)
        return goal

    def goal_occupancy(self, goals):
        if self.goal_teammate_min_distance <= 0.0:
            return []
        with self.lock:
            current = dict((name, dict(self.latest[name]["amcl"]))
                           for name in self.robot_order
                           if self.latest[name].get("amcl") is not None)
        return goal_occupancy_violations(
            current, goals, self.goal_teammate_min_distance)

    def cancel_all(self):
        for publisher in getattr(self,'via_pubs',{}).values():
            publisher.publish(Path())
        for client in self.clients.values():
            try:
                client.cancel_all_goals()
            except Exception:
                pass

    def robot_pose(self, name, timeout=0.0):
        with self.lock:
            if self.latest[name].get("amcl_invalid"):
                raise ValueError("%s invalid AMCL: %s" % (
                    name, self.latest[name]["amcl_invalid"]["reason"]))
        transform = self.tf_buffer.lookup_transform(
            self.map_frame, "%s/base_footprint" % name,
            rospy.Time(0), rospy.Duration(timeout))
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            self.yaw_from_quaternion(transform.transform.rotation),
        )

    def teammate_clearance_violations(self):
        if self.runtime_teammate_min_distance <= 0.0:
            return []
        poses = {}
        for name in self.robot_order:
            try:
                x, y, _yaw = self.robot_pose(name)
            except Exception:
                continue
            poses[name] = (x, y)
        violations = []
        names = sorted(poses)
        for index, first in enumerate(names):
            for second in names[index + 1:]:
                distance = math.hypot(
                    poses[first][0] - poses[second][0],
                    poses[first][1] - poses[second][1])
                if distance < self.runtime_teammate_min_distance:
                    violations.append({
                        "robots": [first, second],
                        "distance": round(distance, 4),
                        "minimum_distance": round(
                            self.runtime_teammate_min_distance, 4),
                    })
        return violations

    @staticmethod
    def publish_zeros(publishers, repeats=1):
        for _index in range(repeats):
            for publisher in publishers.values():
                try:
                    publisher.publish(Twist())
                except Exception:
                    pass
            if repeats > 1:
                rospy.sleep(0.03)

    def formation_line_poses(self, robot_names):
        now = time.time()
        with self.lock:
            odom = dict((name, self.latest[name].get("odom"))
                        for name in robot_names)
        errors = []
        poses = {}
        for name in robot_names:
            sample = odom[name]
            if sample is None or now - sample["receive_wall"] > self.sensor_max_age:
                errors.append("%s odom stale" % name)
                continue
            try:
                poses[name] = self.robot_pose(name, 0.02)
            except Exception:
                errors.append("%s map pose unavailable" % name)
        return poses, errors

    def connector_clear(self, goals, poses, committed=None, tracking=.08, reservation=None):
        """Certify a short straight connector; never turn it into an obstacle planner."""
        routes=dict((n,[poses[n][:2],[g['x'],g['y']]]) for n,g in goals.items())
        committed=dict(committed or {})
        committed.update(routes)
        for name,route in routes.items():
            if self.transition_grid is None or not self.transition_grid.line_free(*route):
                return name+': map connector occupied/unknown'
            for other,pose in poses.items():
                if other==name:
                    continue
                clearance=(reservation if other in committed and reservation is not None
                           else self.runtime_teammate_min_distance+tracking)
                if paths_conflict(route,committed.get(other,[pose[:2],pose[:2]]),clearance):
                    return name+': connector conflicts with '+other
            with self.lock:
                sample=dict(self.latest[name].get('scan',{}))
            scan=sample.get('message')
            if (scan is None or time.time()-sample['receive_wall']>self.sensor_max_age
                    or abs(rospy.Time.now().to_sec()-sample['stamp'])>self.sensor_max_age):
                return name+': connector scan stale'
            try:
                tr=self.tf_buffer.lookup_transform(self.map_frame,scan.header.frame_id,
                    scan.header.stamp,rospy.Duration(.02)).transform
            except Exception:
                return name+': connector scan TF unavailable'
            yaw=self.yaw_from_quaternion(tr.rotation); observed=False
            for i,r in enumerate(scan.ranges):
                if math.isnan(r) or r<scan.range_min:
                    continue
                observed=True
                if math.isinf(r) or r>scan.range_max:
                    continue
                a=yaw+scan.angle_min+i*scan.angle_increment
                point=(tr.translation.x+r*math.cos(a),tr.translation.y+r*math.sin(a))
                if point_path(point,route)<.23:
                    return name+': obstacle in short connector'
            if not observed:
                return name+': connector scan blind'
        return None

    def robot_stopped(self, name):
        with self.lock:
            odom=self.latest[name].get('odom',{}); cmd=self.latest[name].get('cmd',{})
        return (time.time()-odom.get('receive_wall',0)<self.sensor_max_age
            and math.hypot(odom.get('vx',1),odom.get('vy',1))<=self.odom_linear_threshold
            and abs(odom.get('wz',1))<=self.odom_angular_threshold
            and math.hypot(cmd.get('vx',1),cmd.get('vy',1))<=self.cmd_linear_threshold
            and abs(cmd.get('wz',1))<=self.cmd_angular_threshold)

    def slot_robot_stopped(self, name):
        """Use the six-robot stop gate, including fresh IMU yaw rate."""
        if not self.robot_stopped(name):
            return False
        with self.lock:
            imu=self.latest[name].get('imu',{})
        return (time.time()-imu.get('receive_wall',0)<self.sensor_max_age
                and abs(imu.get('wz',1))<=self.imu_wz_threshold)

    def tick_connector(self, name, state, stage, poses, committed, publisher):
        """Nonblocking: this robot owns direct velocity; other TEB clients keep running."""
        item=state['connector']; now=time.time(); goal=item['goal']
        if self.clients[name].get_state() in self.ACTIVE_GOAL_STATES:
            return 'TRANSITION_COMMAND_OWNER_CONFLICT'
        if item.get('deadline') is not None and now>=item['deadline']:
            return 'ROUTE_RECOVERY_TIMEOUT'
        dt=max(0.,min(.20,now-item['last_tick']))
        if item.get('motion_elapsed',0.)>stage.get('connector_motion_timeout',20.):
            return 'TRANSITION_CONNECTOR_TIMEOUT'
        route_error=point_path(poses[name],item['route'])
        if route_error>stage['tracking_error_hard']:
            item['route_hard_since']=item.get('route_hard_since') or now
            if now-item['route_hard_since']>=stage['tracking_error_hold']:
                return 'ROUTE_DEVIATION'
        else:
            item['route_hard_since']=None
        recheck=stage.get('connector_recheck_period',.20)
        if now-item.get('last_check',0.)>=recheck:
            connector_reservation=min(stage['route_reservation'],
                self.runtime_teammate_min_distance+2*stage['tracking_error'])
            problem=self.connector_clear({name:goal},poses,committed,
                                         stage['tracking_error'],connector_reservation)
            item['last_check']=now
            if problem:
                if problem!=item.get('blocked'):
                    self.publish_event('TRANSITION_CONNECTOR_WAIT',stage=stage['name'],
                        robot=name,reason=problem)
                item['blocked']=problem
                item['blocked_since']=item.get('blocked_since') or now
                item['clear_since']=None
            elif item.get('blocked'):
                item['clear_since']=item.get('clear_since') or now
                if now-item['clear_since']>=stage.get('connector_clear_stable',.50):
                    self.publish_event('TRANSITION_CONNECTOR_RESUMED',stage=stage['name'],
                        robot=name,reason='clear')
                    item['blocked']=None; item['blocked_since']=None
                    item['clear_since']=None
        if (item.get('blocked') and item.get('blocked_since') is not None and
                now-item['blocked_since']>=stage.get('connector_block_timeout',20.)):
            if 'map connector occupied/unknown' in item['blocked']:
                return 'TRANSITION_CONNECTOR_LOCAL_HOLD'
            return ('TRANSITION_SENSOR_STALE' if ('stale' in item['blocked'].lower()
                or 'tf unavailable' in item['blocked'].lower()) else
                'TRANSITION_CONNECTOR_BLOCKED')
        ready=now-item['started']>=.20 and publisher.get_num_connections()>0
        values,arrived=((0.,0.,0.),False) if item.get('blocked') or not ready else vector_command(
            poses[name],goal,item['cfg'],item['command'],dt)
        cmd=Twist(); cmd.linear.x,cmd.linear.y,cmd.angular.z=values
        publisher.publish(cmd); item['command']=values; item['last_tick']=now
        if not item.get('blocked') and ready:
            item['motion_elapsed']=item.get('motion_elapsed',0.)+dt
        # A soft reservation/scan wait holds ONLY this robot. The admitted route
        # remains reserved; actual clearance/TF/route faults still stop the fleet.
        if arrived and self.robot_stopped(name):
            item['stable']=item['stable'] or now
            if now-item['stable']>=.2:
                publisher.unregister()
                yaw_realign=item.get('reason')=='yaw_realign'
                if not yaw_realign:
                    state['last_verified_target']=max(
                        state.get('last_verified_target',0),state['target'])
                state.update(connector=None,connector_done=not yaw_realign,
                             yaw_realign_done=yaw_realign,running=False)
                transition_claim_motion_owner(state,'IDLE')
                self.publish_event('TRANSITION_CONNECTOR_END',stage=stage['name'],robot=name,
                                   reason=item.get('reason','short_gate'),ok=True,
                                   code='FORMATION_LINE_SUCCEEDED')
        else:
            item['stable']=None
        return None

    def run_formation_line(self, stage, moving):
        cfg = dict(self.formation_line_config)
        cfg.update(stage.get("formation_line", {}))
        vector_mode=bool(cfg.get('path_from_start',False))
        max_speed = float(cfg.get("speed", 0.05))
        min_speed = float(cfg.get("min_speed", 0.025))
        max_lateral = float(cfg.get("max_lateral", 0.02))
        max_angular = float(cfg.get("max_angular", 0.08))
        acc_x = float(cfg.get("acc_lim_x", 0.10))
        acc_y = float(cfg.get("acc_lim_y", 0.10))
        acc_yaw = float(cfg.get("acc_lim_theta", 0.15))
        sync_gain = float(cfg.get("sync_gain", 1.0))
        cross_gain = float(cfg.get("cross_gain", 0.8))
        yaw_gain = float(cfg.get("yaw_gain", 1.2))
        slow_distance = float(cfg.get("slow_distance", 0.18))
        tolerance = float(cfg.get("position_tolerance", 0.05))
        cross_tolerance = float(cfg.get("cross_tolerance", 0.05))
        yaw_tolerance = math.radians(float(cfg.get("yaw_tolerance_deg", 5.0)))
        stable_duration = float(cfg.get("stable_duration", 0.8))
        recovery_timeout = float(cfg.get("sensor_recovery_timeout", 1.5))
        loop_hz = max(5.0, float(cfg.get("loop_hz", 20.0)))

        if max_speed <= 0.0 or min_speed <= 0.0 or min_speed > max_speed:
            return False, "FORMATION_CONFIG_INVALID", {
                "speed": max_speed, "min_speed": min_speed}

        goals = stage["goals"]
        poses, errors = self.formation_line_poses(moving)
        if errors:
            return False, "FORMATION_SENSOR_STALE", {"errors": errors}
        routes = {}
        initial_remaining = {}
        for name in moving:
            goal = goals[name]
            goal_yaw = math.radians(float(goal["yaw_deg"]))
            path_x = math.cos(goal_yaw)
            path_y = math.sin(goal_yaw)
            normal_x = -path_y
            normal_y = path_x
            x, y, _yaw = poses[name]
            if vector_mode:
                angle=math.atan2(float(goal['y'])-y,float(goal['x'])-x)
                path_x,path_y=math.cos(angle),math.sin(angle)
                normal_x,normal_y=-path_y,path_x
            distance = path_x * (float(goal["x"]) - x) + path_y * (
                float(goal["y"]) - y)
            if distance < -tolerance:
                return False, "FORMATION_GOAL_BEHIND", {
                    "robot": name, "distance": round(distance, 4)}
            initial_remaining[name] = max(0.0, distance)
            routes[name] = (path_x, path_y, normal_x, normal_y, goal_yaw)

        self.cancel_all()
        publishers = dict((name, rospy.Publisher(
            "/%s/jetauto_controller/cmd_vel" % name, Twist, queue_size=1))
            for name in moving)
        rospy.sleep(0.20)
        self.publish_zeros(publishers, repeats=2)

        deadline = time.time() + float(stage.get("timeout", 90.0))
        fault_since = None
        yaw_bad_since = dict((name, None) for name in moving)
        stable_since = None
        last_report = 0.0
        last_command_time = time.time()
        last_commands = dict((name, (0.0, 0.0, 0.0)) for name in moving)
        rate = rospy.Rate(loop_hz)
        try:
            while not rospy.is_shutdown() and time.time() < deadline:
                if self.stop_requested.is_set():
                    return False, "STOP_REQUESTED", {}

                poses, errors = self.formation_line_poses(moving)
                if errors:
                    self.publish_zeros(publishers)
                    last_commands = dict(
                        (name, (0.0, 0.0, 0.0)) for name in moving)
                    last_command_time = time.time()
                    if fault_since is None:
                        fault_since = time.time()
                        self.publish_event(
                            "FORMATION_LINE_PAUSED", stage=stage["name"],
                            errors=errors)
                    elif time.time() - fault_since > recovery_timeout:
                        return False, "FORMATION_SENSOR_STALE", {
                            "errors": errors,
                            "duration": round(time.time() - fault_since, 3)}
                    rate.sleep()
                    continue
                if fault_since is not None:
                    self.publish_event(
                        "FORMATION_LINE_RESUMED", stage=stage["name"])
                    fault_since = None

                clearance = self.teammate_clearance_violations()
                if clearance:
                    return False, "TEAMMATE_CLEARANCE", {
                        "violations": clearance}
                remaining = {}
                cross = {}
                yaw_error = {}
                arrived = {}
                now = time.time()
                for name in moving:
                    x, y, yaw = poses[name]
                    path_x, path_y, normal_x, normal_y, goal_yaw = routes[name]
                    goal = goals[name]
                    remaining[name] = path_x * (float(goal["x"]) - x) + path_y * (
                        float(goal["y"]) - y)
                    cross[name] = normal_x * (x - float(goal["x"])) + normal_y * (
                        y - float(goal["y"]))
                    yaw_error[name] = self.wrap_angle(goal_yaw - yaw)
                    arrived[name] = (
                        (math.hypot(remaining[name],cross[name]) if vector_mode else remaining[name]) <= tolerance
                        and abs(cross[name]) <= cross_tolerance
                        and abs(yaw_error[name]) <= yaw_tolerance)
                    if (self.yaw_excursion_guard > 0.0
                            and abs(yaw_error[name]) > self.yaw_excursion_guard):
                        if yaw_bad_since[name] is None:
                            yaw_bad_since[name] = now
                        elif now - yaw_bad_since[name] >= self.yaw_excursion_hold:
                            return False, "YAW_EXCURSION", {
                                "robot": name,
                                "error_deg": round(math.degrees(
                                    abs(yaw_error[name])), 3),
                                "limit_deg": round(math.degrees(
                                    self.yaw_excursion_guard), 3)}
                    else:
                        yaw_bad_since[name] = None

                if all(arrived.values()):
                    self.publish_zeros(publishers)
                    if stable_since is None:
                        stable_since = time.time()
                    if time.time() - stable_since >= stable_duration:
                        return True, "FORMATION_LINE_SUCCEEDED", {
                            "shared_progress": 1.0}
                    rate.sleep()
                    continue
                stable_since = None

                shared_progress, forward_speeds = synchronized_forward_speeds(
                    initial_remaining, remaining, max_speed, min_speed,
                    sync_gain, slow_distance, tolerance)
                report = {}
                command_time = time.time()
                command_dt = max(0.001, min(0.20,
                    command_time - last_command_time))
                for name in moving:
                    x, y, yaw = poses[name]
                    path_x, path_y, normal_x, normal_y, _goal_yaw = routes[name]
                    forward = forward_speeds[name]
                    lateral = max(-max_lateral, min(
                        max_lateral, -cross_gain * cross[name]))
                    map_vx = path_x * forward + normal_x * lateral
                    map_vy = path_y * forward + normal_y * lateral
                    command = Twist()
                    if not arrived[name]:
                        command.linear.x = max(-max_speed, min(
                            max_speed,
                            math.cos(yaw) * map_vx + math.sin(yaw) * map_vy))
                        y_limit=max_speed if vector_mode else max_lateral
                        command.linear.y = max(-y_limit, min(
                            y_limit,
                            -math.sin(yaw) * map_vx + math.cos(yaw) * map_vy))
                        command.angular.z = max(-max_angular, min(
                            max_angular, yaw_gain * yaw_error[name]))
                        previous = last_commands[name]
                        command.linear.x = max(
                            previous[0] - acc_x * command_dt,
                            min(previous[0] + acc_x * command_dt,
                                command.linear.x))
                        command.linear.y = max(
                            previous[1] - acc_y * command_dt,
                            min(previous[1] + acc_y * command_dt,
                                command.linear.y))
                        command.angular.z = max(
                            previous[2] - acc_yaw * command_dt,
                            min(previous[2] + acc_yaw * command_dt,
                                command.angular.z))
                    if vector_mode:
                        values,_arrived=vector_command(poses[name],goals[name],cfg,
                                                       last_commands[name],command_dt)
                        command.linear.x,command.linear.y,command.angular.z=values
                    publishers[name].publish(command)
                    last_commands[name] = (
                        command.linear.x, command.linear.y, command.angular.z)
                    report[name] = {
                        "remaining": round(remaining[name], 4),
                        "cross": round(cross[name], 4),
                        "yaw_error_deg": round(math.degrees(yaw_error[name]), 3),
                        "forward": round(forward, 4),
                    }
                last_command_time = command_time

                if time.time() - last_report >= 1.0:
                    self.publish_event(
                        "FORMATION_LINE_PROGRESS", stage=stage["name"],
                        shared_progress=round(shared_progress, 4), robots=report)
                    last_report = time.time()
                rate.sleep()
            return False, "GOAL_TIMEOUT", {
                "mode": "formation_line", "timeout": stage.get("timeout", 90.0)}
        finally:
            self.publish_zeros(publishers, repeats=3)
            for publisher in publishers.values():
                try:
                    publisher.unregister()
                except Exception:
                    pass

    def wait_for_goals(self, moving, goals, timeout):
        deadline = time.time() + timeout
        guard_started = time.time()
        yaw_bad_since = dict((name, None) for name in moving)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.stop_requested.is_set():
                return False, "STOP_REQUESTED", {}
            clearance = self.teammate_clearance_violations()
            if clearance:
                return False, "TEAMMATE_CLEARANCE", {
                    "violations": clearance,
                }
            if self.yaw_excursion_guard > 0.0:
                now = time.time()
                with self.lock:
                    amcl = dict((name, self.latest[name].get("amcl"))
                                for name in moving)
                for name in moving:
                    sample = amcl[name]
                    if (sample is None
                            or sample["receive_wall"] < guard_started
                            or now - sample["receive_wall"] > self.sensor_max_age):
                        yaw_bad_since[name] = None
                        continue
                    goal_yaw = math.radians(float(goals[name]["yaw_deg"]))
                    error = abs(self.wrap_angle(sample["yaw"] - goal_yaw))
                    if error > self.yaw_excursion_guard:
                        if yaw_bad_since[name] is None:
                            yaw_bad_since[name] = now
                        elif now - yaw_bad_since[name] >= self.yaw_excursion_hold:
                            return False, "YAW_EXCURSION", {
                                "robot": name,
                                "yaw_deg": round(math.degrees(sample["yaw"]), 3),
                                "goal_yaw_deg": round(math.degrees(goal_yaw), 3),
                                "error_deg": round(math.degrees(error), 3),
                                "limit_deg": round(math.degrees(
                                    self.yaw_excursion_guard), 3),
                            }
                    else:
                        yaw_bad_since[name] = None
            states = dict((name, self.clients[name].get_state()) for name in moving)
            failed = dict((name, state) for name, state in states.items()
                          if state not in self.ACTIVE_GOAL_STATES
                          and state != GoalStatus.SUCCEEDED)
            if failed:
                return False, "MOVE_BASE_FAILED", failed
            if all(state == GoalStatus.SUCCEEDED for state in states.values()):
                return True, "GOALS_SUCCEEDED", states
            self.stop_requested.wait(0.10)
        return False, "GOAL_TIMEOUT", dict(
            (name, self.clients[name].get_state()) for name in moving)

    def run_transition(self, stage):
        """Release disjoint reserved routes as dependencies finish; no fixed hold."""
        # Shared dispatch entry: legacy profiles keep whole-route reservations.
        if stage.get('scheduler') == 'progress_gates':
            return self.run_gated_transition(stage)
        tasks = list(stage['tasks'])
        pending = dict((task['id'], task) for task in tasks)
        active = {}
        finished = set()
        deadline = time.time() + float(stage.get('timeout', 300.0))
        tracking = float(stage.get('tracking_error', 0.12))
        reservation = float(stage.get('route_reservation', 0.65))
        peak = 0
        yaw_bad_since = {}

        def send(task, index):
            name = task['robot']
            x, y = task['path'][index]
            self.clients[name].send_goal(self.build_goal({
                'x': x, 'y': y, 'yaw_deg': stage['goals'][name]['yaw_deg']}))

        while not rospy.is_shutdown() and time.time() < deadline:
            if self.stop_requested.is_set():
                return False, 'STOP_REQUESTED', {}
            poses, errors = self.formation_line_poses(self.robot_order)
            if errors:
                return False, 'TRANSITION_SENSOR_STALE', {'errors': errors}
            clearance = self.teammate_clearance_violations()
            if clearance:
                return False, 'TEAMMATE_CLEARANCE', {'violations': clearance}
            for task_id, state in list(active.items()):
                task, index = state['task'], state['index']
                name = task['robot']
                deviation = point_path(poses[name][:2], task['path'])
                if deviation > tracking:
                    return False, 'ROUTE_DEVIATION', {'robot':name,'error':deviation,'limit':tracking}
                yaw_error = abs(self.wrap_angle(poses[name][2]-math.radians(stage['goals'][name]['yaw_deg'])))
                if self.yaw_excursion_guard>0 and yaw_error>self.yaw_excursion_guard:
                    yaw_bad_since.setdefault(name,time.time())
                    if time.time()-yaw_bad_since[name]>=self.yaw_excursion_hold:
                        return False,'YAW_EXCURSION',{'robot':name,'error_deg':math.degrees(yaw_error)}
                else:
                    yaw_bad_since.pop(name,None)
                status = self.clients[name].get_state()
                if status == GoalStatus.SUCCEEDED:
                    if distance(poses[name][:2], task['path'][index]) > self.expected_position_error+0.02:
                        return False,'TRANSITION_GOAL_DISAGREEMENT',{'robot':name,'task':task_id}
                    if index+1 < len(task['path']):
                        state['index'] += 1
                        send(task, state['index'])
                        self.publish_event('TRANSITION_WAYPOINT',task=task_id,index=state['index'])
                    else:
                        # Check this robot's current motion, without waiting for
                        # unrelated robots or requesting new AMCL samples.
                        with self.lock:
                            odom = self.latest[name].get('odom',{})
                            cmd = self.latest[name].get('cmd',{})
                        if (math.hypot(odom.get('vx',1),odom.get('vy',1))>self.odom_linear_threshold
                                or abs(odom.get('wz',1))>self.odom_angular_threshold
                                or math.hypot(cmd.get('vx',1),cmd.get('vy',1))>self.cmd_linear_threshold
                                or abs(cmd.get('wz',1))>self.cmd_angular_threshold):
                            continue
                        finished.add(task_id)
                        del active[task_id]
                        self.publish_event('TRANSITION_ARRIVED',task=task_id,robot=name,stage=stage['name'])
                elif status not in self.ACTIVE_GOAL_STATES:
                    return False,'MOVE_BASE_FAILED',{'robot':name,'status':status,'task':task_id}
            for task in tasks:
                task_id=task['id']
                if task_id not in pending or not set(task.get('after',[])).issubset(finished):
                    continue
                if any(paths_conflict(task['path'],state['task']['path'],reservation) for state in active.values()):
                    continue
                name=task['robot']
                if distance(poses[name][:2],task['path'][0])>self.expected_position_error+0.03:
                    return False,'TRANSITION_START_DISAGREEMENT',{'robot':name,'task':task_id}
                active[task_id]={'task':task,'index':1}
                del pending[task_id]
                send(task,1)
                peak=max(peak,len(active))
                self.publish_event('TRANSITION_RELEASED',task=task_id,robot=name,
                                   stage=stage['name'],active_count=len(active))
            if not active and not pending:
                return True,'TRANSITION_SUCCEEDED',{'peak_concurrent':peak,'tasks':len(finished)}
            if not active:
                return False,'TRANSITION_DEPENDENCY_DEADLOCK',{'pending':list(pending)}
            self.stop_requested.wait(0.05)
        return False,'GOAL_TIMEOUT',{'pending':list(pending),'active':list(active)}

    def publish_transition_path(self, name, task, state):
        publisher=self.via_pubs[name]
        if not publisher.get_num_connections():
            raise RuntimeError('TEB via_points subscriber unavailable: '+name)
        # Melodic customViaPointsCB ignores the Path header: transform coordinates
        # explicitly into the local costmap's odom frame before publishing.
        frame=name+'/odom'
        transform=self.tf_buffer.lookup_transform(frame,self.map_frame,rospy.Time(0),rospy.Duration(0.02))
        yaw=self.yaw_from_quaternion(transform.transform.rotation)
        c,s=math.cos(yaw),math.sin(yaw)
        translation=transform.transform.translation
        message=Path(); message.header.frame_id=frame; message.header.stamp=rospy.Time.now()
        for i in range(1,state['target']+1):
            if state['progress']+0.05<task['arc'][i]<=state['progress']+0.80:
                x,y=task['path'][i]
                pose=PoseStamped(); pose.header=message.header; pose.pose.orientation.w=1.0
                pose.pose.position.x=translation.x+c*x-s*y
                pose.pose.position.y=translation.y+s*x+c*y
                message.poses.append(pose)
        publisher.publish(message)

    def run_gated_transition(self, stage):
        publishers={}
        try:
            return self.execute_gated_transition(stage,publishers)
        finally:
            self.publish_zeros(publishers,repeats=3)
            for publisher in publishers.values():
                publisher.unregister()

    def execute_gated_transition(self, stage, publishers):
        """One loop for TEB and certified short segments; progress releases gates."""
        tasks=stage['tasks']; by_name=dict((t['robot'],t) for t in tasks)
        entry=stage.get('entry_tolerance',.025)
        entry_release=stage.get('entry_release_tolerance',entry+.01)
        tracking=stage['tracking_error']; margin=stage['release_margin']
        tracking_hard=stage['tracking_error_hard']
        tracking_hold=stage['tracking_error_hold']
        tracking_wait=stage['tracking_recovery_wait']
        recovery_timeout=self.transition_sensor_recovery_timeout
        teb_retries=stage.get('teb_abort_max_retries',3)
        teb_retry_delay=stage.get('teb_abort_retry_delay',1.5)
        teb_timeout=stage.get('teb_abort_timeout',18.0)
        direction_min_speed=stage.get('teb_direction_min_speed',.01)
        direction_min_cosine=stage.get('teb_direction_min_cosine',0.)
        recovery_advance=stage.get('route_recovery_advance',.12)
        route_recovery_timeout=stage.get('route_recovery_timeout',8.)
        retry_clear_stable=stage.get('connector_clear_stable',.50)
        deadlock_timeout=stage.get('dependency_deadlock_timeout',8.0)
        short_connector_wait_timeout=min(2.0,deadlock_timeout)
        global_plan_timeout=1.0

        # A brief VM/ROS delivery gap is recoverable before any transition
        # command is admitted.  This also prevents a repeated start request
        # from being needed merely because the first snapshot was late.
        poses,errors=self.formation_line_poses(self.robot_order)
        wait_started=time.time()
        while errors and time.time()-wait_started<=recovery_timeout:
            rospy.sleep(.05)
            poses,errors=self.formation_line_poses(self.robot_order)
        if errors:
            return False,'TRANSITION_SENSOR_STALE',{'errors':errors,
                'duration':round(time.time()-wait_started,3)}

        reused_certificates={}
        def classify_start(snapshot,tolerance):
            entry_goals={}; resumed={}; initial_arrived=set(); endpoint_recovery={}
            for task in tasks:
                name=task['robot']; pose=snapshot[name]
                start_error=distance(pose,task['path'][0])
                end_error=distance(pose,task['path'][-1])
                with self.lock:
                    certificate=self.slot_certificates.get(stage['name'],{}).get(name)
                if (self.slot_amcl_verify_enabled and end_error<=tracking and
                        slot_certificate_reusable(certificate,stage['goals'][name],
                            pose,self.slot_robot_stopped(name),
                            self.sample_position_spread,self.sample_yaw_spread)):
                    initial_arrived.add(name)
                    reused_certificates[name]=certificate
                    self.publish_event('SLOT_AMCL_CERTIFICATE_REUSED',
                        stage=stage['name'],robot=name,target_index=len(task['path'])-1,
                        reason='unchanged_pose_and_target')
                    continue
                with self.lock:
                    self.slot_certificates.get(stage['name'],{}).pop(name,None)
                projected_error,projected=project_progress(
                    task,pose,0.0,len(task['path'])-1,0.0)
                action=transition_start_action(start_error,end_error,
                    projected_error,projected,self.robot_stopped(name),tolerance,
                    tracking,stage.get('min_teb_distance',.25),
                    self.expected_position_error+.03,
                    self.slot_amcl_verify_enabled)
                if action=='arrived':
                    initial_arrived.add(name)
                    continue
                if action in ('endpoint_recovery','endpoint_verify'):
                    endpoint_recovery[name]=end_error
                    if action=='endpoint_verify':
                        self.publish_event('TRANSITION_RESTART_SLOT_VERIFY',
                            stage=stage['name'],robot=name,
                            target_index=len(task['path'])-1,
                            endpoint_error=round(end_error,4),
                            reason='near endpoint is not verified arrival')
                    continue
                if action=='resume':
                    resumed[name]=projected
                    continue
                if action=='reject':
                    return None,None,None,None,('TRANSITION_START_DISAGREEMENT',
                        {'robot':name,'error':start_error})
                if action=='entry_recovery':
                    x,y=task['path'][0]
                    entry_goals[name]={'x':x,'y':y,
                        'yaw_deg':stage['goals'][name]['yaw_deg']}
            return entry_goals,resumed,initial_arrived,endpoint_recovery,None

        (_entry_goals,resumed,initial_arrived,endpoint_recovery,
         classification_error)=classify_start(poses,entry)
        if classification_error:
            return False,classification_error[0],classification_error[1]

        states={}
        for name,task in by_name.items():
            progress=(task['arc'][-1] if name in endpoint_recovery or name in initial_arrived
                      else resumed.get(name,0.0))
            states[name]={'target':len(task['path'])-1 if name in endpoint_recovery or name in initial_arrived else 0,
                'progress':progress,'maximum':progress,'running':False,
                'needs_resend':False,'resumed':name in resumed,
                'tracking_fault':None,'teb_retry':None,'teb_guard':None,
                'teb_direction_rejections':0,'recovery_request':None,
                'last_verified_target':route_index_at_or_before(task,progress),
                'direction_recovery':None,'direction_local_hold':None,
                'slot_verify':None,'slot_verify_pending':False,
                'slot_local_hold':None,'slot_corrections':0,
                'slot_certificate':reused_certificates.get(name),
                'plan_check':None,'plan_accepted':False,'retry_inflight':False,
                'motion_owner':'IDLE','owner_epoch':0}
            if name in endpoint_recovery:
                states[name]['goal_settle']={'started':time.time(),
                    'hard_since':None,'restart':True,
                    'status':'waiting_for_stop'}
        cleared=dict((n,resumed.get(n,0.0)) for n in by_name)
        for name in endpoint_recovery:
            cleared[name]=by_name[name]['arc'][-1]-1e-6
        for name in initial_arrived:
            cleared[name]=by_name[name]['arc'][-1]
        arrived=set(initial_arrived); released=set()
        deadline=time.time()+float(stage['timeout']); last_report=0.0
        yaw_bad={}; peak=0; idle_since=None; last_guidance={}
        sensor_fault_since=None; sensor_recovered_since=None; sensor_cancelled=False
        direction_control={'owner':None}
        slot_nomotion_inflight=set()
        slot_verify_queue=[]

        def claim_motion_owner(name,state,owner,reason):
            previous,epoch=transition_claim_motion_owner(state,owner)
            if previous!=owner:
                self.publish_event('TRANSITION_MOTION_OWNER',stage=stage['name'],
                    robot=name,previous_owner=previous,motion_owner=owner,
                    owner_epoch=epoch,reason=reason,target_index=state.get('target'))
            return epoch

        def invalidate_teb_state(state):
            state['teb_retry']=None; state['teb_guard']=None
            state['plan_check']=None; state['plan_accepted']=False
            state['retry_inflight']=False; state['needs_resend']=False

        def prepare_local_recovery(name,state,reason):
            claim_motion_owner(name,state,'IDLE',reason)
            invalidate_teb_state(state)
            state['running']=False

        def claim_connector_owner(name,state,reason):
            epoch=claim_motion_owner(name,state,'CONNECTOR',reason)
            invalidate_teb_state(state)
            return epoch

        def send_teb_goal(name,state,expected_ticket=None):
            task=by_name[name]; target=state['target']; x,y=task['path'][target]
            if (state.get('connector') or state.get('slot_verify') or
                    state.get('slot_local_hold') or state.get('direction_local_hold')):
                self.publish_event('TRANSITION_STALE_OWNER_BLOCKED',stage=stage['name'],
                    robot=name,requested_owner='TEB',motion_owner=state.get('motion_owner'),
                    owner_epoch=state.get('owner_epoch'),target_index=target,
                    reason='incompatible active controller')
                return False
            if expected_ticket is not None and not transition_owner_ticket_valid(
                    state,expected_ticket,'TEB'):
                self.publish_event('TRANSITION_STALE_OWNER_BLOCKED',stage=stage['name'],
                    robot=name,requested_owner='TEB',motion_owner=state.get('motion_owner'),
                    owner_epoch=state.get('owner_epoch'),target_index=target,
                    reason='stale owner ticket')
                return False
            epoch=claim_motion_owner(name,state,'TEB','send_teb_goal')
            state.pop('goal_settle',None)
            self.publish_transition_path(name,task,state)
            sent_at=time.time()
            state['plan_check']={'sent_at':sent_at,'target':target,
                                 'preempt_reported':False,'owner_epoch':epoch}
            state['plan_accepted']=False
            state['retry_inflight']=bool(state.get('teb_retry'))
            self.clients[name].send_goal(self.build_goal({'x':x,'y':y,
                'yaw_deg':stage['goals'][name]['yaw_deg']}))
            state['connector_done']=False
            state['teb_guard']={'sent_at':sent_at,'target':target,
                                'owner_epoch':epoch}
            publish_plan_event('GLOBAL_PLAN_CHECK',name,state,'goal_sent',0,
                               requested_target_index=target)
            return True

        def has_reservation(state):
            return transition_state_has_reservation(state)

        def retry_inputs_ready(name):
            with self.lock:
                scan=dict(self.latest[name].get('scan',{}))
            message=scan.get('message'); now=time.time()
            if (message is None or now-scan.get('receive_wall',0)>self.sensor_max_age
                    or abs(rospy.Time.now().to_sec()-scan.get('stamp',0))>self.sensor_max_age):
                return False,'scan stale'
            try:
                self.tf_buffer.lookup_transform(self.map_frame,
                    name+'/base_footprint',rospy.Time(0),rospy.Duration(.02))
                self.tf_buffer.lookup_transform(name+'/odom',message.header.frame_id,
                    message.header.stamp,rospy.Duration(.02))
            except Exception:
                return False,'TF not stable'
            if not self.robot_stopped(name):
                return False,'waiting for robot stop'
            return True,'ready'

        def connector_admission(name,state,goal,snapshot,reserved):
            now=time.time(); pending=state.get('connector_admission')
            recheck=stage.get('connector_recheck_period',.20)
            if pending and now-pending['last_check']<recheck:
                return False,None,pending['reason']
            reservation=min(stage['route_reservation'],
                self.runtime_teammate_min_distance+2*tracking)
            problem=self.connector_clear({name:goal},snapshot,reserved,tracking,
                                         reservation)
            if problem:
                if pending is None:
                    pending={'started':now,'last_check':now,'reason':problem,
                             'clear_since':None}
                    state['connector_admission']=pending
                    self.publish_event('TRANSITION_CONNECTOR_WAIT',stage=stage['name'],
                        robot=name,reason=problem,phase='admission')
                else:
                    if problem!=pending['reason']:
                        self.publish_event('TRANSITION_CONNECTOR_WAIT',stage=stage['name'],
                            robot=name,reason=problem,phase='admission')
                    pending.update(last_check=now,reason=problem,clear_since=None)
                elapsed=now-pending['started']
                if connector_problem_action(problem,elapsed,
                                            stage.get('connector_block_timeout',20.))=='local_hold':
                    return False,'TRANSITION_CONNECTOR_LOCAL_HOLD',problem
                if elapsed>=stage.get('connector_block_timeout',20.):
                    code=('TRANSITION_SENSOR_STALE' if ('stale' in problem.lower()
                        or 'tf unavailable' in problem.lower()) else
                        'TRANSITION_CONNECTOR_BLOCKED')
                    return False,code,problem
                return False,None,problem
            if pending:
                pending['last_check']=now
                pending['clear_since']=pending.get('clear_since') or now
                pending['reason']='stabilizing clear connector'
                if now-pending['clear_since']<stage.get('connector_clear_stable',.50):
                    return False,None,pending['reason']
                state.pop('connector_admission',None)
                self.publish_event('TRANSITION_CONNECTOR_RESUMED',stage=stage['name'],
                    robot=name,reason='clear',phase='admission')
            return True,None,None

        def recovery_blockers(name,state,snapshot):
            route=remaining_route(by_name[name],snapshot[name],state['progress'],state['target'])
            blockers=[]
            for other,other_state in states.items():
                if other==name:
                    continue
                if has_reservation(other_state):
                    other_route=([list(snapshot[other][:2]),other_state['connector']['route'][-1]]
                        if other_state.get('connector') else remaining_route(by_name[other],snapshot[other],
                                                other_state['progress'],other_state['target']))
                    if paths_conflict(route,other_route,stage['route_reservation']):
                        blockers.append(other+':committed_route')
                elif point_path(snapshot[other],route)<self.runtime_teammate_min_distance+tracking_hard:
                    blockers.append(other+':parked_pose')
            return blockers

        def publish_direction_event(kind,name,state,reason,**extra):
            recovery=(state.get('direction_recovery') or
                      state.get('direction_local_hold') or {})
            values={'robot':name,'stage':stage['name'],
                'target_index':recovery.get('original_target',state['target']),
                'route_progress':round(state.get('progress',0.),3),
                'direction_cosine':recovery.get('direction_cosine'),
                'reason':reason}
            values.update(extra)
            self.publish_event(kind,**values)

        def publish_plan_event(kind,name,state,reason,point_count,**extra):
            recovery=state.get('direction_recovery') or {}
            values={'robot':name,'stage':stage['name'],
                'requested_target_index':recovery.get('original_target',state['target']),
                'active_target_index':state['target'],
                'fallback_target_index':recovery.get('verified_target'),
                'global_plan_point_count':int(point_count),
                'route_progress':round(state.get('progress',0.),3),
                'reason':reason}
            values.update(extra)
            self.publish_event(kind,**values)

        def finish_recovery_node(name,state,pose):
            task=by_name[name]; target=state['target']
            recovery=state.get('direction_recovery')
            if recovery:
                # Rebase progress from the measured pose after an intentional
                # return; the old forward maximum must not poison normal mode.
                _error,projected=project_progress(task,pose,0.,target,
                                                  task['arc'][-1])
                state['progress']=projected
                state['maximum']=projected
            else:
                state['progress']=max(state['progress'],task['arc'][target])
                state['maximum']=max(state['maximum'],state['progress'])
            state['last_verified_target']=max(state['last_verified_target'],target)
            cleared[name]=min(state['progress'],task['arc'][-1]-1e-6)
            if not recovery:
                return
            recovery['verified_target']=target
            recovery['last_progress_at']=time.time()
            if target>=recovery['original_target']:
                if recovery.get('kind')=='global_plan':
                    publish_plan_event('TRANSITION_RECOVERY_SUCCEEDED',name,state,
                        'original_target_reached',recovery.get('global_plan_point_count',0),
                        fallback_target_index=target)
                else:
                    publish_direction_event('DIRECTION_RECOVERY_SUCCEEDED',
                        name,state,'original_target_reached',
                        verified_target_index=target,next_target_index=target)
                state['direction_recovery']=None
                state['teb_direction_rejections']=0
                state.pop('teb_direction_first_reject',None)
                direction_control['owner']=None
                with self.lock:
                    self.state='EXECUTING'
            else:
                recovery['phase']='polyline'
                recovery['next_target']=target+1

        def begin_global_plan_recovery(name,state,task,progress,reason,point_count):
            if state.get('direction_recovery'):
                begin_direction_local_hold(name,state,
                    'global_plan_candidates_exhausted')
                return
            now=time.time(); original=state['target']
            verified=min(state.get('last_verified_target',0),original)
            recovery={'kind':'global_plan','started':now,
                'original_target':original,'projected_progress':progress,
                'verified_target':verified,'next_target':verified,
                'direction_cosine':None,'phase':'return_verified',
                'last_progress_at':now,'global_plan_point_count':int(point_count)}
            direction_control['owner']=name
            with self.lock:
                self.state='TRANSITION_RECOVERING'
            state['direction_recovery']=recovery
            state['direction_local_hold']=None
            prepare_local_recovery(name,state,'global_plan_recovery')
            state['recovery_request']={'target':verified,
                'reason':'global_plan_recovery','started':now}
            self.clients[name].cancel_goal()
            publish_plan_event('TRANSITION_FALLBACK_NODE',name,state,reason,
                point_count,fallback_target_index=verified)
            publish_plan_event('TRANSITION_WAIT_FOR_RECOVERY',name,state,reason,
                point_count,fallback_target_index=verified)
            state['progress']=task['arc'][verified]
            state['maximum']=task['arc'][verified]
            cleared[name]=task['arc'][verified]

        def begin_direction_recovery(name,state,task,progress,cosine):
            now=time.time(); verified=min(state.get('last_verified_target',0),
                                          state['target'])
            recovery={'started':now,'original_target':state['target'],
                'projected_progress':progress,'verified_target':verified,
                'next_target':verified,'direction_cosine':round(cosine,3),
                'phase':'return_verified','last_progress_at':now}
            direction_control['owner']=name
            with self.lock:
                self.state='DIRECTION_RECOVERING'
            state['direction_recovery']=recovery
            state['direction_local_hold']=None
            state['teb_direction_rejections']=0
            state.pop('teb_direction_first_reject',None)
            prepare_local_recovery(name,state,'direction_recovery')
            state['recovery_request']={'target':verified,
                'reason':'direction_recovery','started':now}
            self.clients[name].cancel_goal()
            publish_direction_event('DIRECTION_RECOVERY_STARTED',name,state,
                'teb_initial_direction_reverse',
                verified_target_index=verified,next_target_index=verified)
            publish_direction_event('DIRECTION_RECOVERY_PROJECTED',name,state,
                'route_projection',projected_progress=round(progress,3),
                verified_target_index=verified,next_target_index=verified)
            state['progress']=task['arc'][verified]
            state['maximum']=task['arc'][verified]
            cleared[name]=task['arc'][verified]

        def begin_direction_local_hold(name,state,reason):
            if state.get('direction_local_hold'):
                return
            recovery=state.get('direction_recovery') or {}
            self.clients[name].cancel_goal()
            publisher=publishers.pop(name,None)
            if publisher is not None:
                publisher.publish(Twist()); publisher.unregister()
            prepare_local_recovery(name,state,'direction_local_hold')
            state['connector']=None; state['recovery_request']=None
            state['direction_local_hold']=dict(recovery,reason=reason,
                hold_started=time.time())
            state['direction_recovery']=None
            claim_motion_owner(name,state,'LOCAL_HOLD','direction_local_hold')
            if recovery.get('kind')=='global_plan':
                publish_plan_event('TRANSITION_RECOVERY_TIMEOUT',name,state,reason,
                    recovery.get('global_plan_point_count',0),
                    fallback_target_index=recovery.get('verified_target'))
            else:
                publish_direction_event('DIRECTION_RECOVERY_TIMEOUT',name,state,reason,
                    verified_target_index=recovery.get('verified_target'),
                    next_target_index=recovery.get('next_target'))

        def begin_connector_local_hold(name,state,reason):
            """Hold only a connector-blocked robot; keep the transition loop alive."""
            if state.get('connector_local_hold'):
                return
            self.clients[name].cancel_goal()
            publisher=publishers.pop(name,None)
            if publisher is not None:
                publisher.publish(Twist()); publisher.unregister()
            item=state.pop('connector',None) or {}
            prepare_local_recovery(name,state,'connector_local_hold')
            state['recovery_request']=None
            state['connector_local_hold']={'reason':reason,
                'hold_started':time.time(),'target':state.get('target')}
            state['running']=False
            claim_motion_owner(name,state,'LOCAL_HOLD','connector_local_hold')
            self.publish_event('TRANSITION_CONNECTOR_LOCAL_HOLD',stage=stage['name'],
                robot=name,reason=reason,target_index=state.get('target'),
                blocked_since=item.get('blocked_since'))

        def publish_slot_event(kind,name,state,reason,**extra):
            item=state.get('slot_verify') or state.get('slot_local_hold') or {}
            values={'robot':name,'stage':stage['name'],
                'target_index':state['target'],'reason':reason,
                'request_count':item.get('request_count',0),
                'sample_count':len(item.get('samples',[]))}
            values.update(extra)
            self.publish_event(kind,**values)

        def begin_slot_verification(name,state):
            now=time.time()
            active_count=sum(bool(s.get('slot_verify')) for s in states.values())
            if not slot_amcl_queue_admit(slot_verify_queue,name,active_count,
                                          self.slot_amcl_max_parallel):
                first_wait=not state.get('slot_verify_pending')
                state['slot_verify_pending']=True
                settle=state.get('goal_settle') or {}
                settle['status']='slot_verify_queued'
                state['goal_settle']=settle
                if first_wait:
                    self.publish_event('SLOT_AMCL_VERIFY_QUEUED',
                        stage=stage['name'],robot=name,target_index=state['target'],
                        reason='verification_capacity',active_verifications=active_count,
                        parallel_limit=self.slot_amcl_max_parallel,deadline=None)
                return
            with self.lock:
                initial=dict(self.latest[name].get('amcl',{}))
            state.pop('goal_settle',None)
            state['running']=False
            invalidate_teb_state(state)
            state['slot_verify']={'started':now,
                'deadline':now+self.slot_amcl_timeout,'next_request':now,
                'request_count':0,'request_started':0.,'request_token':0,
                'request_active':False,'service_status':None,'samples':[],
                'seen':set([initial['stamp']]) if initial else set(),
                'request_ros_started':0.,
                'status':'queued',
                'stable_since':now-self.settle_stable_duration}
            state['slot_verify_pending']=False
            claim_motion_owner(name,state,'SLOT_VERIFY','slot_amcl_verification')
            publish_slot_event('SLOT_AMCL_VERIFY_QUEUED',name,state,
                'final_target_stopped',stable_duration=self.settle_stable_duration,
                active_verifications=active_count+1,
                parallel_limit=self.slot_amcl_max_parallel,
                deadline=state['slot_verify']['deadline'])

        def request_slot_nomotion(name,state,token):
            status=self.call_nomotion(name)
            with self.lock:
                item=state.get('slot_verify')
                if item and item.get('request_token')==token:
                    item['service_status']=status
                    item['request_active']=False
                    item['status']='collecting'
                slot_nomotion_inflight.discard((name,token))

        def begin_slot_local_hold(name,state,reason,metrics=None):
            if state.get('slot_local_hold'):
                return
            self.clients[name].cancel_goal()
            publisher=publishers.pop(name,None)
            if publisher is not None:
                publisher.publish(Twist()); publisher.unregister()
            item=state.pop('slot_verify',None) or {}
            state['slot_verify_pending']=False
            prepare_local_recovery(name,state,'slot_local_hold')
            state['connector']=None; state['recovery_request']=None
            state.pop('goal_settle',None)
            state['slot_local_hold']=dict(item,reason=reason,
                metrics=metrics or {},hold_started=time.time())
            claim_motion_owner(name,state,'LOCAL_HOLD','slot_local_hold')
            publish_slot_event('SLOT_AMCL_LOCAL_HOLD',name,state,reason,
                metrics=metrics or {})

        def tick_slot_verification(name,state,task):
            item=state['slot_verify']; now=time.time()
            if not self.slot_robot_stopped(name):
                item['stable_since']=None; item['status']='waiting_for_stop'
                item['samples']=[]
                return
            item['stable_since']=item.get('stable_since') or now
            if now-item['stable_since']<self.settle_stable_duration:
                item['status']='settling'
                return
            with self.lock:
                amcl=dict(self.latest[name].get('amcl',{}))
                scan=dict(self.latest[name].get('scan',{}))
            scan_fresh=(now-scan.get('receive_wall',0.)<=self.sensor_max_age
                and abs(rospy.Time.now().to_sec()-scan.get('stamp',0.))
                    <=self.sensor_max_age)
            if (item['request_count']>0 and amcl
                    and amcl.get('receive_wall',0.)>=item['request_started']
                    and amcl['stamp'] not in item['seen']
                    and scan_fresh):
                item['seen'].add(amcl['stamp']); item['samples'].append(amcl)
                publish_slot_event('SLOT_AMCL_SAMPLE',name,state,'fresh_after_stop',
                    x=round(amcl['x'],4),y=round(amcl['y'],4),
                    yaw_deg=round(math.degrees(amcl['yaw']),3),
                    amcl_stamp=amcl['stamp'],
                    amcl_receive_wall=round(amcl['receive_wall'],6),
                    cov_x=amcl.get('cov_x'),cov_y=amcl.get('cov_y'),
                    cov_yaw=amcl.get('cov_yaw'),
                    scan_stamp=scan.get('stamp'),
                    scan_age=round(now-scan.get('receive_wall',0.),3),
                    service_status=item.get('service_status'))
                if len(item['samples'])==1:
                    item['status']='candidate'
                    publish_slot_event('SLOT_AMCL_CANDIDATE',name,state,
                        'first_fresh_sample_waiting_for_confirmation',
                        minimum_sample_count=self.slot_amcl_samples_required,
                        minimum_observation_span=self.slot_amcl_min_observation_span)
            if (item['request_active'] and
                    now-item['request_started']>self.nomotion_service_timeout+.2):
                with self.lock:
                    expired_token=item['request_token']
                    item['request_active']=False
                    item['service_status']='AMCL_SERVICE_TIMEOUT'
                    item['request_token']+=1
                    slot_nomotion_inflight.discard((name,expired_token))
                publish_slot_event('SLOT_AMCL_NOMOTION_TIMEOUT',name,state,
                    'request_nomotion_update_timeout')
            effective_count=slot_amcl_effective_sample_count(item['samples'],
                self.slot_amcl_samples_required,
                self.slot_amcl_min_observation_span)
            action=slot_amcl_request_action(effective_count,
                item['request_count'],item['request_active'],now,
                item['next_request'],item['deadline'],self.slot_amcl_samples_required,
                self.samples_preferred,self.nomotion_max_requests)
            if action=='request':
                with self.lock:
                    if len(slot_nomotion_inflight)>=self.slot_amcl_max_parallel:
                        item['status']='queued'
                        return
                    item['request_count']+=1; item['request_token']+=1
                    item['request_started']=now; item['request_active']=True
                    item['service_status']=None; item['status']='requesting'
                    item['next_request']=(now+self.amcl_callback_timeout+
                                          self.nomotion_request_interval)
                    token=item['request_token']
                    slot_nomotion_inflight.add((name,token))
                publish_slot_event('SLOT_AMCL_NOMOTION_REQUEST',name,state,
                    'request_nomotion_update',parallel_inflight=len(slot_nomotion_inflight),
                    parallel_limit=self.slot_amcl_max_parallel)
                thread=threading.Thread(target=request_slot_nomotion,
                    args=(name,state,token)); thread.daemon=True; thread.start()
                return
            if action=='wait':
                item['status']='collecting' if item['request_count'] else 'queued'
                return
            if action=='hold':
                # A missing preferred second frame is a bounded retry, not a
                # position disagreement.  Keep the slot reserved while the
                # next verification ticket is queued.
                if item['samples'] and len(item['samples']) >= self.slot_amcl_samples_required:
                    publish_slot_event('SLOT_AMCL_DEGRADED_ACCEPTED',name,state,
                        'minimum_fresh_amcl_satisfied',sample_count=len(item['samples']))
                    action='verify'
                else:
                    state['slot_verify']=None
                    state['slot_verify_pending']=True
                    state['slot_retry_after']=now+self.nomotion_request_interval
                    state['goal_settle']={'started':now,'hard_since':None,
                        'status':'slot_verify_retry_wait'}
                    claim_motion_owner(name,state,'SLOT_VERIFY','slot_amcl_retry_wait')
                    slot_verify_queue.append(name)
                    publish_slot_event('SLOT_AMCL_VERIFY_WAIT',name,state,
                        'no_fresh_post_stop_sample',sample_count=len(item['samples']))
                    return
            expected={name:{'x':float(stage['goals'][name]['x']),
                'y':float(stage['goals'][name]['y']),
                'yaw':math.radians(float(stage['goals'][name]['yaw_deg']))}}
            ok,code,metrics=self.verify_samples(
                {name:list(item['samples'])},expected,[name])
            detail=metrics.get(name,{})
            if ok and detail.get('expected_position_error',0.)>tracking:
                ok=False; code='AMCL_GOAL_DISAGREEMENT'
            if ok:
                if len(item['samples']) < self.samples_preferred:
                    publish_slot_event('SLOT_AMCL_DEGRADED_ACCEPTED',name,state,
                        'minimum_fresh_amcl_satisfied',sample_count=len(item['samples']))
                publish_slot_event('SLOT_AMCL_VERIFIED',name,state,
                    'stable_fresh_amcl_matches_slot',metrics=detail,
                    observation_span=round(item['samples'][-1]['receive_wall']-
                                           item['samples'][0]['receive_wall'],3))
                certificate={'verified_wall':now,'metrics':detail,
                    'target_index':state['target'],'goal':dict(stage['goals'][name]),
                    'pose':list(poses[name]),
                    'sample_stamps':[sample['stamp'] for sample in item['samples']],
                    'observation_span':round(item['samples'][-1]['receive_wall']-
                                             item['samples'][0]['receive_wall'],3)}
                self.slot_certificates.setdefault(stage['name'],{})[name]=certificate
                clear_transition_arrival_state(state)
                state['slot_certificate']=certificate
                if direction_control.get('owner')==name:
                    direction_control['owner']=None
                arrived.add(name); cleared[name]=task['arc'][-1]
                self.via_pubs[name].publish(Path())
                self.publish_event('TRANSITION_ARRIVED',task=task['id'],
                    robot=name,stage=stage['name'],slot_amcl_verified=True)
                return
            if code=='AMCL_GOAL_DISAGREEMENT' and (
                    state['slot_corrections']<self.slot_amcl_max_corrections):
                state['slot_corrections']+=1
                publish_slot_event('SLOT_AMCL_REALIGN',name,state,
                    'amcl_goal_disagreement',metrics=detail,
                    correction_attempt=state['slot_corrections'])
                state['slot_verify']=None
                claim_motion_owner(name,state,'IDLE','slot_amcl_realign')
                state['recovery_request']={'target':len(task['path'])-1,
                    'reason':'slot_amcl_realign','started':now}
                return
            if now<item['deadline'] and item['request_count']<self.nomotion_max_requests:
                item['samples']=item['samples'][-1:]
                item['next_request']=now+self.nomotion_request_interval
                item['status']='retrying_'+code.lower()
                publish_slot_event('SLOT_AMCL_VERIFY_WAIT',name,state,code,
                    metrics=detail)
                return
            begin_slot_local_hold(name,state,code.lower(),detail)

        def direction_hold_blockers(owner,snapshot):
            blockers=[]; owner_pose=snapshot[owner]
            for other,other_state in states.items():
                if other==owner or other in arrived:
                    continue
                route=remaining_route(by_name[other],snapshot[other],
                    other_state['progress'],len(by_name[other]['path'])-1)
                if point_path(owner_pose,route)<self.runtime_teammate_min_distance+tracking_hard:
                    blockers.append(other)
            return blockers
        while not rospy.is_shutdown() and time.time()<deadline:
            if self.stop_requested.is_set():
                return False,'STOP_REQUESTED',{}
            poses,errors=self.formation_line_poses(self.robot_order)
            if errors:
                self.publish_zeros(publishers)
                now=time.time()
                if sensor_fault_since is None:
                    sensor_fault_since=now; sensor_recovered_since=None
                    self.publish_event('TRANSITION_SENSOR_PAUSED',
                        stage=stage['name'],errors=errors)
                if not sensor_cancelled:
                    self.cancel_all(); sensor_cancelled=True
                    for name,state in states.items():
                        state['needs_resend']=(transition_owner_ticket(state)
                            if transition_sensor_resend_allowed(
                                name,state,arrived,require_running=True) else False)
                        state['running']=False
                if now-sensor_fault_since>recovery_timeout:
                    return False,'TRANSITION_SENSOR_STALE',{'errors':errors,
                        'duration':round(now-sensor_fault_since,3)}
                self.stop_requested.wait(.05)
                continue
            if sensor_fault_since is not None:
                sensor_recovered_since=sensor_recovered_since or time.time()
                if time.time()-sensor_recovered_since<.5:
                    self.publish_zeros(publishers)
                    self.stop_requested.wait(.05)
                    continue
                for name,state in states.items():
                    resend_ticket=state.pop('needs_resend',False)
                    if (resend_ticket and
                            transition_sensor_resend_allowed(name,state,arrived) and
                            transition_owner_ticket_valid(state,resend_ticket,'TEB')):
                        try:
                            state['running']=True
                            if not send_teb_goal(name,state,resend_ticket):
                                state['running']=False
                        except Exception as exc:
                            return (False,'TRANSITION_GOAL_SEND_FAILED',
                                {'robot':name,'error':str(exc)})
                self.publish_event('TRANSITION_SENSOR_RESUMED',stage=stage['name'])
                sensor_fault_since=None; sensor_recovered_since=None
                sensor_cancelled=False
            violations=self.teammate_clearance_violations()
            if violations:
                return False,'TEAMMATE_CLEARANCE',{'violations':violations}
            recovery_owner=direction_control.get('owner')
            if recovery_owner:
                recovery=states[recovery_owner].get('direction_recovery')
                if (recovery and time.time()-recovery['last_progress_at']>=
                        route_recovery_timeout):
                    begin_direction_local_hold(recovery_owner,states[recovery_owner],
                                               'direction_recovery_no_progress_timeout')
            committed={}
            for n,s in states.items():
                if has_reservation(s):
                    committed[n]=([list(poses[n][:2]),s['connector']['route'][-1]]
                        if s.get('connector') else remaining_route(by_name[n],poses[n],s['progress'],s['target']))
            for name,state in states.items():
                if name in arrived or (not state['target'] and
                                       state['progress']<=0.0 and
                                       not state.get('connector')):
                    continue
                if state.get('direction_local_hold') or state.get('slot_local_hold'):
                    continue
                task=by_name[name]
                owner_error=transition_motion_owner_error(state)
                if owner_error:
                    return False,'TRANSITION_OWNER_STATE_INVALID',{
                        'robot':name,'error':owner_error,
                        'motion_owner':state.get('motion_owner'),
                        'owner_epoch':state.get('owner_epoch')}
                explicit_recovery=bool(state.get('direction_recovery'))
                if (state.get('slot_verify_pending') and
                        sum(bool(s.get('slot_verify')) for s in states.values()) <
                        self.slot_amcl_max_parallel and
                        (not state.get('slot_retry_after') or
                         time.time() >= state['slot_retry_after'])):
                    state.pop('goal_settle',None)
                    begin_slot_verification(name,state)
                    continue
                if state.get('slot_verify'):
                    tick_slot_verification(name,state,task)
                    continue
                retry=state.get('teb_retry') or {}
                if (retry.get('waiting') and
                        time.time()-retry['first_abort']>=teb_timeout):
                    begin_global_plan_recovery(name,state,task,state['progress'],
                        'move_base_retry_timeout',0)
                    continue
                if state.get('connector'):
                    error=point_path(poses[name],state['connector']['route'])
                    progress=state['maximum']
                else:
                    committed_limit=(state['target'] if state['target'] else
                        (len(task['path'])-1 if state.get('resumed') else 0))
                    error,progress=project_progress(task,poses[name],
                        0. if explicit_recovery else state['maximum'],
                        committed_limit,
                        task['arc'][-1] if explicit_recovery else margin)
                if transition_progress_regression_fault(state,progress,margin):
                    return False,'ROUTE_DEVIATION',{'robot':name,'error':error,
                        'progress':progress,'previous':state['maximum'],
                        'reason':'progress_regression','limit':tracking}
                state['progress']=progress; state['maximum']=max(state['maximum'],progress)
                cleared[name]=min(progress,task['arc'][-1]-1e-6)
                if error<=tracking and not explicit_recovery:
                    state['last_verified_target']=max(state['last_verified_target'],
                        route_index_at_or_before(task,progress))
                plan_check=state.get('plan_check')
                if (plan_check and not transition_owner_ticket_valid(state,{
                        'owner':'TEB','owner_epoch':plan_check.get('owner_epoch'),
                        'target':plan_check.get('target')},'TEB')):
                    state['plan_check']=None; state['retry_inflight']=False
                    plan_check=None
                if plan_check and state['running'] and not state.get('connector'):
                    with self.lock:
                        plan_evidence=dict(self.latest[name].get('global_plan',{}))
                    plan_action,point_count=global_plan_check_action(
                        plan_check,plan_evidence,time.time(),global_plan_timeout)
                    if plan_action=='waiting':
                        continue
                    state['plan_check']=None; state['retry_inflight']=False
                    if plan_action=='accepted':
                        state['plan_accepted']=True
                        publish_plan_event('GLOBAL_PLAN_ACCEPTED',name,state,
                            'global_plan_nonempty',point_count)
                    else:
                        state['plan_accepted']=False; state['running']=False
                        state['teb_guard']=None
                        publish_plan_event('GLOBAL_PLAN_EMPTY',name,state,
                            'global_plan_empty' if plan_action=='empty'
                            else 'global_plan_timeout',point_count)
                        begin_global_plan_recovery(name,state,task,progress,
                            'global_plan_empty' if plan_action=='empty'
                            else 'global_plan_timeout',point_count)
                        continue
                guard=state.get('teb_guard')
                if (guard and not transition_owner_ticket_valid(state,{
                        'owner':'TEB','owner_epoch':guard.get('owner_epoch'),
                        'target':guard.get('target')},'TEB')):
                    state['teb_guard']=None; guard=None
                if guard and state['running'] and not state.get('connector'):
                    with self.lock:
                        cmd=dict(self.latest[name].get('cmd',{}))
                    if cmd.get('receive_wall',0.)>=guard['sent_at']:
                        direction,cosine=teb_command_direction(
                            cmd.get('vx',0.),cmd.get('vy',0.),poses[name][2],
                            route_tangent(task,progress,state['target']),
                            direction_min_speed,direction_min_cosine)
                        action,rejections,first_reject=direction_guard_action(
                            self.sp_direction_guard_enabled,direction,
                            state['teb_direction_rejections'],time.time(),
                            state.get('teb_direction_first_reject'),teb_retries,teb_timeout)
                        state['teb_direction_rejections']=rejections
                        if first_reject is None:
                            state.pop('teb_direction_first_reject',None)
                        else:
                            state['teb_direction_first_reject']=first_reject
                        if action=='forward':
                            state['teb_guard']=None
                            state['teb_direction_rejections']=0
                            state.pop('teb_direction_first_reject',None)
                            self.publish_event('TRANSITION_TEB_DIRECTION_ACCEPTED',
                                stage=stage['name'],robot=name,
                                target_index=state['target'],cosine=round(cosine,3))
                            if state.get('direction_recovery'):
                                recovery=state['direction_recovery']
                                if not recovery.get('teb_rejoined'):
                                    recovery['teb_rejoined']=True
                                    publish_direction_event(
                                        'DIRECTION_RECOVERY_TEB_REJOINED',name,state,
                                        'teb_direction_accepted',
                                        verified_target_index=recovery['verified_target'],
                                        next_target_index=state['target'])
                        elif action=='bypass':
                            state['teb_guard']=None
                            publish_direction_event('SP_DIRECTION_GUARD_BYPASSED',name,state,
                                'teb_initial_direction_reverse',
                                direction_cosine=round(cosine,3))
                        elif action=='recover':
                            recovery_owner=direction_control.get('owner')
                            if recovery_owner and recovery_owner!=name:
                                self.clients[name].cancel_goal(); state['running']=False
                                state['teb_guard']=None
                                state['teb_retry']={'first_abort':time.time(),'failures':0,
                                    'retry_at':time.time(),'waiting':True,'clear_since':None,
                                    'wait_reason':'direction recovery: '+recovery_owner,
                                    'owner_epoch':state.get('owner_epoch',0)}
                            elif state.get('direction_recovery'):
                                begin_direction_local_hold(name,state,
                                    'direction_candidates_exhausted')
                            else:
                                begin_direction_recovery(name,state,task,progress,cosine)
                            continue
                        elif action=='reject':
                            now=time.time()
                            fallback=recovery_target_index(task,progress,state['target'],
                                                           recovery_advance)
                            self.clients[name].cancel_goal()
                            prepare_local_recovery(name,state,'teb_direction_realign')
                            state['recovery_request']={'target':fallback,
                                'reason':'teb_direction','started':now}
                            self.publish_event('TRANSITION_TEB_DIRECTION_REJECTED',
                                stage=stage['name'],robot=name,
                                target_index=state['target'],fallback_index=fallback,
                                cosine=round(cosine,3),
                                attempt=state['teb_direction_rejections'])
                            continue
                request=state.get('recovery_request')
                if request and not state.get('connector'):
                    if time.time()-request['started']>=route_recovery_timeout:
                        if state.get('direction_recovery'):
                            begin_direction_local_hold(name,state,
                                'direction_recovery_timeout')
                            continue
                        return False,'ROUTE_RECOVERY_TIMEOUT',{'robot':name,
                            'task':task['id'],'reason':request['reason'],
                            'duration':round(time.time()-request['started'],3)}
                    state['running']=False
                    continue
                previous_fault=state.get('tracking_fault')
                goal_succeeded=(state.get('goal_settle') or
                    (not state.get('connector') and
                     self.clients[name].get_state()==GoalStatus.SUCCEEDED))
                action,fault=(('normal',None) if goal_succeeded or explicit_recovery else
                    route_tracking_action(error,time.time(),tracking,
                        tracking_hard,tracking_hold,tracking_wait,previous_fault))
                state['tracking_fault']=fault
                if action=='fail':
                    return False,'ROUTE_DEVIATION',{'robot':name,'error':error,
                        'progress':progress,'previous':state['maximum'],
                        'limit':tracking_hard,'duration':tracking_hold}
                if action=='pause':
                    if state.get('connector'):
                        publishers[name].publish(Twist())
                        state['connector']['command']=(0.,0.,0.)
                        state['connector']['last_tick']=time.time()
                    else:
                        self.clients[name].cancel_goal()
                        fallback=recovery_target_index(task,progress,state['target'],
                                                       recovery_advance)
                        prepare_local_recovery(name,state,'route_realign')
                        state['recovery_request']={'target':fallback,
                            'reason':'route_realign','started':time.time()}
                    state['running']=False
                    self.publish_event('ROUTE_TRACKING_WARN',stage=stage['name'],
                        robot=name,error=round(error,4),soft_limit=tracking,
                        hard_limit=tracking_hard,retry_after=tracking_wait)
                    continue
                if action=='wait':
                    if state.get('connector'):
                        publishers[name].publish(Twist())
                    state['running']=False
                    continue
                retry=state.get('teb_retry') or {}
                if retry.get('waiting'):
                    now=time.time()
                    if (direction_control.get('owner') and
                            direction_control['owner']!=name):
                        retry['wait_reason']='direction recovery: '+direction_control['owner']
                        continue
                    if now-retry['first_abort']>=teb_timeout:
                        begin_global_plan_recovery(name,state,task,progress,
                            'move_base_retry_timeout',0)
                        continue
                    if now<retry['retry_at']:
                        retry['wait_reason']='retry delay'
                        continue
                    retry_ticket={'owner':'TEB',
                        'owner_epoch':retry.get('owner_epoch'),
                        'target':state['target']}
                    if not transition_owner_ticket_valid(state,retry_ticket,'TEB'):
                        state['teb_retry']=None
                        self.publish_event('TRANSITION_STALE_OWNER_BLOCKED',
                            stage=stage['name'],robot=name,requested_owner='TEB',
                            motion_owner=state.get('motion_owner'),
                            owner_epoch=state.get('owner_epoch'),
                            target_index=state['target'],reason='stale teb retry')
                        continue
                    ready,reason=retry_inputs_ready(name)
                    blockers=recovery_blockers(name,state,poses) if ready else []
                    if not ready or blockers:
                        retry['clear_since']=None
                        retry['wait_reason']=reason if not ready else blockers
                        continue
                    retry['clear_since']=retry.get('clear_since') or now
                    retry['wait_reason']='stabilizing'
                    if now-retry['clear_since']<retry_clear_stable:
                        continue
                    try:
                        state['running']=True; retry['waiting']=False
                        retry['clear_since']=None
                        if not send_teb_goal(name,state,retry_ticket):
                            state['running']=False
                            continue
                    except Exception as exc:
                        return False,'TRANSITION_GOAL_SEND_FAILED',{
                            'robot':name,'error':str(exc)}
                    self.publish_event('TRANSITION_TEB_RETRY',stage=stage['name'],
                        robot=name,task=task['id'],attempt=retry['failures'],
                        target_index=state['target'])
                    continue
                if action=='retry':
                    blockers=recovery_blockers(name,state,poses)
                    if blockers:
                        fault.update(retry_at=time.time()+tracking_wait,
                                     recovering=False,hard_since=None)
                        state['running']=False
                        continue
                    state['running']=True
                    if not state.get('connector'):
                        try:
                            if not send_teb_goal(name,state):
                                state['running']=False
                                continue
                        except Exception as exc:
                            return False,'TRANSITION_GOAL_SEND_FAILED',{
                                'robot':name,'error':str(exc)}
                    self.publish_event('ROUTE_TRACKING_REPLAN',stage=stage['name'],
                        robot=name,error=round(error,4),progress=round(progress,3))
                elif action=='recovered':
                    if not state['running'] and not state.get('connector'):
                        try:
                            state['running']=True
                            if not send_teb_goal(name,state):
                                state['running']=False
                                continue
                        except Exception as exc:
                            return False,'TRANSITION_GOAL_SEND_FAILED',{
                                'robot':name,'error':str(exc)}
                    self.publish_event('ROUTE_TRACKING_RECOVERED',stage=stage['name'],
                        robot=name,error=round(error,4),progress=round(progress,3))
                if not state.get('connector') and time.time()-last_guidance.get(name,0.0)>0.15:
                    try:
                        self.publish_transition_path(name,task,state)
                    except Exception as exc:
                        return False,'TRANSITION_GUIDANCE_UNAVAILABLE',{'robot':name,'error':str(exc)}
                    last_guidance[name]=time.time()
                yaw_error=abs(self.wrap_angle(poses[name][2]-math.radians(stage['goals'][name]['yaw_deg'])))
                yaw_action,yaw_since=(('normal',None) if state.get('connector') or
                    not state['running'] else yaw_excursion_action(yaw_error,
                        self.yaw_excursion_guard,time.time(),self.yaw_excursion_hold,
                        yaw_bad.get(name)))
                if yaw_since is None:
                    yaw_bad.pop(name,None)
                else:
                    yaw_bad[name]=yaw_since
                if yaw_action=='recover':
                    self.clients[name].cancel_goal()
                    prepare_local_recovery(name,state,'yaw_realign')
                    yaw_bad.pop(name,None)
                    state['recovery_request']={'target':state['target'],
                        'reason':'yaw_realign','started':time.time()}
                    self.publish_event('TRANSITION_YAW_RECOVERY',stage=stage['name'],
                        robot=name,target_index=state['target'],
                        error_deg=round(math.degrees(yaw_error),3))
                    continue
                if state.get('connector'):
                    fault=self.tick_connector(name,state,stage,poses,committed,publishers[name])
                    if fault:
                        if fault=='TRANSITION_CONNECTOR_LOCAL_HOLD':
                            begin_connector_local_hold(name,state,
                                'map_connector_block_timeout')
                            continue
                        if state['connector'].get('reason')=='slot_amcl_realign':
                            begin_slot_local_hold(name,state,
                                'slot_realign_'+fault.lower())
                            continue
                        if (state.get('direction_recovery') and fault in (
                                'ROUTE_RECOVERY_TIMEOUT','TRANSITION_CONNECTOR_TIMEOUT',
                                'TRANSITION_CONNECTOR_INVALID')):
                            begin_direction_local_hold(name,state,
                                'direction_candidates_exhausted')
                            continue
                        return False,fault,{'robot':name,'controller':'connector',
                            'reason':state['connector'].get('blocked'),
                            'duration':round(time.time()-state['connector']['started'],3)}
                    if state.get('connector'):
                        continue
                    del publishers[name]
                    if state.get('connector_done'):
                        finish_recovery_node(name,state,poses[name])
                    if state.pop('yaw_realign_done',False):
                        try:
                            state['running']=True
                            if not send_teb_goal(name,state):
                                state['running']=False
                                continue
                        except Exception as exc:
                            return False,'TRANSITION_GOAL_SEND_FAILED',{
                                'robot':name,'error':str(exc)}
                        self.publish_event('TRANSITION_YAW_RECOVERED',stage=stage['name'],
                            robot=name,target_index=state['target'],
                            progress=round(state['progress'],3))
                        continue
                status=transition_goal_status(state,self.clients[name].get_state(),
                                              GoalStatus.SUCCEEDED)
                if status is None:
                    continue
                state['running']=status in self.ACTIVE_GOAL_STATES
                if status==GoalStatus.SUCCEEDED:
                    if state.get('teb_retry'):
                        self.publish_event('TRANSITION_TEB_RECOVERED',stage=stage['name'],
                            robot=name,task=task['id'],
                            attempts=state['teb_retry']['failures'])
                        state['teb_retry']=None
                    now=time.time()
                    error=distance(poses[name],task['path'][state['target']])
                    stopped=self.robot_stopped(name)
                    settle=state.get('goal_settle') or {
                        'started':now,'hard_since':None,'status':'waiting_for_stop'}
                    action,hard_since=transition_goal_action(error,stopped,now,
                        tracking,tracking_hard,tracking_hold,
                        settle.get('hard_since'))
                    settle.update(error=error,hard_since=hard_since)
                    state['goal_settle']=settle
                    state['running']=False
                    if action=='wait_stop':
                        settle.pop('stopped_since',None)
                        settle['status']='waiting_for_stop'
                        if not settle.get('reported'):
                            self.publish_event('TRANSITION_GOAL_SETTLING',stage=stage['name'],
                                robot=name,target_index=state['target'],
                                error=round(error,4))
                            settle['reported']=True
                        continue
                    if action=='confirm':
                        state['running']=False; settle['status']='confirming_large_error'
                        if not settle.get('hard_reported'):
                            self.publish_event('TRANSITION_GOAL_RECOVERY_WAIT',
                                stage=stage['name'],robot=name,
                                target_index=state['target'],error=round(error,4),
                                hard_limit=tracking_hard,hold=tracking_hold)
                            settle['hard_reported']=True
                        continue
                    if action=='recover':
                        prepare_local_recovery(name,state,'goal_realign')
                        state.pop('goal_settle',None)
                        state['recovery_request']={'target':state['target'],
                            'reason':'goal_realign','started':now}
                        self.publish_event('TRANSITION_GOAL_RECOVERY',
                            stage=stage['name'],robot=name,
                            target_index=state['target'],error=round(error,4),
                            soft_limit=tracking,hard_limit=tracking_hard)
                        continue
                    if (self.slot_amcl_verify_enabled
                            and state['target']==len(task['path'])-1):
                        if not self.slot_robot_stopped(name):
                            settle.pop('stopped_since',None)
                            settle['status']='slot_waiting_for_stop'
                            continue
                        stopped_since=settle.get('stopped_since') or now
                        settle['stopped_since']=stopped_since
                        settle['status']='slot_stopping'
                        if now-stopped_since<self.settle_stable_duration:
                            continue
                        begin_slot_verification(name,state)
                        continue
                    state.pop('goal_settle',None); state['running']=False
                    claim_motion_owner(name,state,'IDLE','teb_goal_completed')
                    finish_recovery_node(name,state,poses[name])
                    if state['target']==len(task['path'])-1:
                        arrived.add(name); cleared[name]=task['arc'][-1]
                        self.via_pubs[name].publish(Path())
                        self.publish_event('TRANSITION_ARRIVED',task=task['id'],robot=name,stage=stage['name'])
                elif status==GoalStatus.ABORTED:
                    with self.lock:
                        plan_evidence=dict(self.latest[name].get('global_plan',{}))
                    sent_at=(state.get('plan_check') or {}).get('sent_at',
                        (state.get('teb_guard') or {}).get('sent_at',0.))
                    point_count=(int(plan_evidence.get('points',0))
                        if plan_evidence.get('receive_wall',0.)>=sent_at else 0)
                    if not state.get('plan_accepted') or point_count==0:
                        state['plan_check']=None; state['retry_inflight']=False
                        state['running']=False; state['teb_guard']=None
                        publish_plan_event('GLOBAL_PLAN_EMPTY',name,state,
                            'move_base_aborted_without_plan',point_count)
                        begin_global_plan_recovery(name,state,task,progress,
                            'move_base_aborted_without_plan',point_count)
                        continue
                    action,retry=teb_abort_action(time.time(),state.get('teb_retry'),
                        teb_retries,teb_retry_delay,teb_timeout)
                    retry['owner_epoch']=state.get('owner_epoch',0)
                    state['teb_retry']=retry; state['running']=False
                    state['plan_check']=None; state['retry_inflight']=False
                    self.clients[name].cancel_goal()
                    if action=='fail':
                        if state.get('direction_recovery'):
                            begin_direction_local_hold(name,state,
                                'direction_candidates_exhausted')
                            continue
                        begin_global_plan_recovery(name,state,task,progress,
                            'move_base_retry_exhausted',point_count)
                        continue
                    self.publish_event('TRANSITION_TEB_RETRY_WAIT',stage=stage['name'],
                        robot=name,task=task['id'],attempt=retry['failures'],
                        max_retries=teb_retries,retry_after=teb_retry_delay)
                    continue
                elif status not in self.ACTIVE_GOAL_STATES:
                    if state.get('direction_recovery'):
                        begin_direction_local_hold(name,state,
                            'direction_candidates_exhausted')
                        continue
                    return False,'MOVE_BASE_FAILED',{'robot':name,'status':status,'task':task['id']}
            if len(arrived)==len(tasks):
                certificates=dict((name,state.get('slot_certificate'))
                    for name,state in states.items() if state.get('slot_certificate'))
                return True,'TRANSITION_SUCCEEDED',{
                    'peak_concurrent':peak,'tasks':len(arrived),
                    'slot_amcl_verified':bool(self.slot_amcl_verify_enabled and
                        len(certificates)==len(tasks)),
                    'slot_certificates':certificates}
            waiting={}
            for task in tasks:
                name=task['robot']; state=states[name]
                if name in arrived:
                    continue
                recovery_owner=direction_control.get('owner')
                if not transition_dispatch_allowed(name,recovery_owner):
                    waiting[name]='direction_recovery: '+recovery_owner
                    continue
                if state.get('direction_local_hold'):
                    waiting[name]='LOCAL_GOAL_HOLD'
                    continue
                request=state.get('recovery_request')
                if request:
                    waiting[name]='route_recovery: '+request['reason']
                    if (self.clients[name].get_state() in self.ACTIVE_GOAL_STATES
                            or not self.robot_stopped(name)):
                        continue
                    permitted=permitted_target(task,cleared,arrived,state['progress'])
                    target=min(request['target'],permitted)
                    if target<0:
                        continue
                    x,y=(poses[name][:2] if request['reason']=='yaw_realign'
                         else task['path'][target])
                    goal={'x':x,'y':y,'yaw_deg':stage['goals'][name]['yaw_deg']}
                    reserved=dict((n,([list(poses[n][:2]),s['connector']['route'][-1]]
                        if s.get('connector') else remaining_route(by_name[n],poses[n],
                                                s['progress'],s['target'])))
                        for n,s in states.items() if has_reservation(s))
                    admitted,fault,reason=connector_admission(
                        name,state,goal,poses,reserved)
                    if fault:
                        if fault=='TRANSITION_CONNECTOR_LOCAL_HOLD':
                            begin_connector_local_hold(name,state,
                                'map_connector_block_timeout')
                            continue
                        if request['reason']=='slot_amcl_realign':
                            begin_slot_local_hold(name,state,
                                'slot_realign_'+fault.lower())
                            continue
                        if state.get('direction_recovery'):
                            begin_direction_local_hold(name,state,
                                'direction_candidates_exhausted')
                            continue
                        return False,fault,{'robot':name,'controller':'connector',
                            'reason':reason,'phase':'recovery_admission'}
                    if not admitted:
                        waiting[name]=reason
                        continue
                    cfg=dict(self.formation_line_config)
                    # Previous short-connector speed caps: 0.05, then 0.07 m/s.
                    cfg.update(speed=min(.10,float(cfg.get('speed',.05))),
                               min_speed=min(.10,float(cfg.get('min_speed',.025))),
                               position_tolerance=entry)
                    if not 0<float(cfg.get('min_speed',.025))<=cfg['speed']:
                        return False,'FORMATION_CONFIG_INVALID',{'robot':name}
                    self.via_pubs[name].publish(Path())
                    publishers[name]=rospy.Publisher(
                        '/%s/jetauto_controller/cmd_vel'%name,Twist,queue_size=1)
                    claim_connector_owner(name,state,request['reason'])
                    state.update(target=target,running=True,connector_done=False,
                        teb_guard=None,recovery_request=None,connector={
                        'goal':goal,'route':[list(poses[name][:2]),[x,y]],'cfg':cfg,
                        'started':time.time(),'last_tick':time.time(),'last_check':0.,
                        'deadline':request['started']+route_recovery_timeout,
                        'command':(0.,0.,0.),'stable':None,'blocked':None,
                        'blocked_since':None,'clear_since':None,'route_hard_since':None,
                        'motion_elapsed':0.,'reason':request['reason']})
                    if state.get('direction_recovery'):
                        state['direction_recovery']['next_target']=target
                        if state['direction_recovery'].get('kind')=='global_plan':
                            publish_plan_event('TRANSITION_POLYLINE_STEP',name,state,
                                'return_to_verified_node',0,
                                fallback_target_index=target)
                        else:
                            publish_direction_event('DIRECTION_RECOVERY_VECTOR_NODE',
                                name,state,'return_to_verified_node',
                                verified_target_index=state['direction_recovery']['verified_target'],
                                next_target_index=target)
                    self.publish_event('TRANSITION_CONNECTOR_START',stage=stage['name'],
                        robot=name,reason=request['reason'],goals={name:goal},
                        target_index=target)
                    continue
                if state.get('tracking_fault'):
                    waiting[name]='route_tracking_recovery'
                    continue
                retry=state.get('teb_retry') or {}
                if retry.get('waiting'):
                    waiting[name]='teb_retry: '+str(retry.get('wait_reason','waiting'))
                    continue
                plan_check=state.get('plan_check')
                if plan_check:
                    candidate=permitted_target(task,cleared,arrived,state['progress'])
                    if transition_target_preempt_action(
                            plan_check,state['target'],candidate)=='block':
                        if not plan_check.get('preempt_reported'):
                            publish_plan_event('TRANSITION_TARGET_PREEMPT_BLOCKED',
                                name,state,'global_plan_pending',0,
                                requested_target_index=candidate)
                            plan_check['preempt_reported']=True
                    waiting[name]='global_plan_pending'
                    continue
                if state.get('goal_settle'):
                    waiting[name]='goal_settle: '+state['goal_settle'].get('status','waiting')
                    continue
                if state.get('slot_verify'):
                    waiting[name]='slot_amcl: '+state['slot_verify'].get('status','waiting')
                    continue
                if state.get('slot_local_hold'):
                    waiting[name]='SLOT_LOCALIZATION_HOLD'
                    continue
                if state.get('connector_local_hold'):
                    waiting[name]='CONNECTOR_LOCAL_GOAL_HOLD'
                    continue
                if state.get('connector'):
                    waiting[name]=state['connector'].get('blocked') or 'executing_short_segment'
                    continue
                direction_recovery=state.get('direction_recovery')
                if direction_recovery:
                    target=direction_recovery_next_target(
                        state['target'],direction_recovery['original_target'])
                    direction_recovery['next_target']=target
                else:
                    target=permitted_target(task,cleared,arrived,state['progress'])
                if not state['target'] and state['progress']<=.05:
                    entry_error=distance(poses[name],task['path'][0])
                    entry_action,entry_since=entry_recovery_action(
                        entry_error,time.time(),entry_release,tracking_hard,
                        tracking_hold,state.get('entry_drift_since'))
                    state['entry_drift_since']=entry_since
                    if entry_action=='fail':
                        return False,'TRANSITION_ENTRY_DRIFT',{'robot':name,
                            'error':entry_error,'limit':tracking_hard,
                            'duration':tracking_hold}
                    if entry_action=='wait':
                        waiting[name]='entry_drift_confirming'
                        continue
                    if entry_action=='recover':
                        waiting[name]='entry_realign'
                        if (self.clients[name].get_state() in self.ACTIVE_GOAL_STATES
                                or not self.robot_stopped(name)):
                            continue
                        x,y=task['path'][0]
                        goal={'x':x,'y':y,'yaw_deg':stage['goals'][name]['yaw_deg']}
                        reserved=dict((n,([list(poses[n][:2]),s['connector']['route'][-1]]
                            if s.get('connector') else remaining_route(by_name[n],poses[n],
                                                    s['progress'],s['target'])))
                            for n,s in states.items() if has_reservation(s))
                        admitted,fault,reason=connector_admission(
                            name,state,goal,poses,reserved)
                        if fault:
                            if fault=='TRANSITION_CONNECTOR_LOCAL_HOLD':
                                begin_connector_local_hold(name,state,
                                    'map_connector_block_timeout')
                                continue
                            if direction_recovery:
                                begin_direction_local_hold(name,state,
                                    'direction_candidates_exhausted')
                                continue
                            return False,fault,{'robot':name,'controller':'connector',
                                'reason':reason,'phase':'admission'}
                        if not admitted:
                            waiting[name]=reason
                            continue
                        cfg=dict(self.formation_line_config)
                        # Previous short-connector speed caps: 0.05, then 0.07 m/s.
                        cfg.update(speed=min(.10,float(cfg.get('speed',.05))),
                                   min_speed=min(.10,float(cfg.get('min_speed',.025))),
                                   position_tolerance=entry,
                                   cross_tolerance=entry)
                        self.via_pubs[name].publish(Path())
                        publishers[name]=rospy.Publisher(
                            '/%s/jetauto_controller/cmd_vel'%name,Twist,queue_size=1)
                        claim_connector_owner(name,state,'entry_realign')
                        state.update(running=True,connector_done=False,connector={
                            'goal':goal,'route':[list(poses[name][:2]),[x,y]],'cfg':cfg,
                            'started':time.time(),'last_tick':time.time(),
                            'last_check':0.,'command':(0.,0.,0.),'stable':None,
                            'blocked':None,'blocked_since':None,
                            'clear_since':None,'route_hard_since':None,'motion_elapsed':0.,
                            'reason':'entry_realign'})
                        self.publish_event('TRANSITION_CONNECTOR_START',stage=stage['name'],
                            robot=name,reason='entry_realign',goals={name:goal},target_index=0)
                        continue
                if target<=state['target']:
                    waiting[name]='dependency' if not state['running'] else 'executing'
                    continue
                # Coalesce geometric milestones into an original straight leg.
                # Extend a near gate before TEB stops; never stream 10cm goals.
                if state['running'] and target not in task['corners'] and (
                        task['arc'][target]-task['arc'][state['target']]<0.20
                        and distance(poses[name],task['path'][state['target']])>0.22):
                    continue
                route=remaining_route(task,poses[name],state['progress'],target)
                blockers=[]
                for other,other_state in states.items():
                    if other==name:
                        continue
                    if has_reservation(other_state):
                        other_route=([list(poses[other][:2]),other_state['connector']['route'][-1]]
                            if other_state.get('connector') else remaining_route(by_name[other],poses[other],
                                                    other_state['progress'],other_state['target']))
                        if paths_conflict(route,other_route,stage['route_reservation']):
                            blockers.append(other+':committed_route')
                    # Nominal parked_clearance already includes placement error.
                    # Here the parked center is measured, so reserve the runtime
                    # minimum plus the moving robot's enforced tracking envelope.
                    elif point_path(poses[other],route)<self.runtime_teammate_min_distance+tracking:
                        blockers.append(other+':parked_pose')
                if blockers:
                    waiting[name]=blockers
                    continue
                if distance(poses[name],task['path'][target])<stage.get('min_teb_distance',.25):
                    waiting[name]='coalescing_short_prefix'
                    # A bounded coalescing window, never an all-fleet-idle barrier.
                    state.setdefault('short_since',time.time())
                    if state['running'] or time.time()-state['short_since']<.30:
                        continue
                    if (self.clients[name].get_state() in self.ACTIVE_GOAL_STATES
                            or not self.robot_stopped(name)):
                        waiting[name]='waiting_for_own_stop'
                        continue
                    x,y=task['path'][target]
                    goal={'x':x,'y':y,'yaw_deg':stage['goals'][name]['yaw_deg']}
                    short_action=short_connector_action(self.transition_grid,
                        [list(poses[name][:2]),[x,y]],route)
                    if short_action=='teb':
                        state.pop('connector_admission',None)
                        self.publish_event('TRANSITION_CONNECTOR_TEB_FALLBACK',
                            stage=stage['name'],robot=name,target_index=target,
                            reason='direct map connector unavailable; original route clear')
                    elif short_action=='wait':
                        reason=name+': direct and original route occupied/unknown'
                        pending=state.get('connector_admission')
                        if pending is None:
                            state['connector_admission']={'started':time.time(),
                                'last_check':time.time(),'reason':reason,
                                'clear_since':None,'initial_target':target,
                                'candidate_target':target}
                            self.publish_event('TRANSITION_CONNECTOR_WAIT',
                                stage=stage['name'],robot=name,reason=reason,
                                phase='route_fallback')
                            pending=state['connector_admission']
                        else:
                            pending.update(last_check=time.time(),reason=reason,
                                           clear_since=None,
                                           candidate_target=target)
                        if bounded_short_connector_action(pending,time.time(),
                                                          short_connector_wait_timeout)=='wait':
                            waiting[name]=reason
                            continue
                        state.pop('connector_admission',None)
                        self.publish_event('TRANSITION_CONNECTOR_TEB_FALLBACK',
                            stage=stage['name'],robot=name,target_index=target,
                            initial_target_index=pending.get('initial_target',target),
                            reason='bounded short connector wait; checking current permitted TEB target')
                    else:
                        # Recheck teammate, scan and TF conditions before direct control.
                        reserved=dict((n,([list(poses[n][:2]),s['connector']['route'][-1]]
                            if s.get('connector') else remaining_route(by_name[n],poses[n],s['progress'],s['target'])))
                            for n,s in states.items() if has_reservation(s))
                        admitted,fault,reason=connector_admission(
                            name,state,goal,poses,reserved)
                        if fault:
                            if fault=='TRANSITION_CONNECTOR_LOCAL_HOLD':
                                begin_connector_local_hold(name,state,
                                    'map_connector_block_timeout')
                                continue
                            if direction_recovery:
                                begin_direction_local_hold(name,state,
                                    'direction_candidates_exhausted')
                                continue
                            return False,fault,{'robot':name,'controller':'connector',
                                'reason':reason,'phase':'admission'}
                        if not admitted:
                            waiting[name]=reason
                            continue
                        cfg=dict(self.formation_line_config)
                        # Previous short-connector speed caps: 0.05, then 0.07 m/s.
                        cfg.update(speed=min(.10,float(cfg.get('speed',.05))),
                                   min_speed=min(.10,float(cfg.get('min_speed',.025))),
                                   position_tolerance=entry)
                        if not 0<float(cfg.get('min_speed',.025))<=cfg['speed']:
                            return False,'FORMATION_CONFIG_INVALID',{'robot':name}
                        self.via_pubs[name].publish(Path())
                        publishers[name]=rospy.Publisher('/%s/jetauto_controller/cmd_vel'%name,Twist,queue_size=1)
                        connector_reason=('direction_recovery' if direction_recovery
                                          else 'short_gate')
                        claim_connector_owner(name,state,connector_reason)
                        state.update(target=target,running=True,connector_done=False,connector={
                            'goal':goal,'route':[list(poses[name][:2]),[x,y]],'cfg':cfg,
                            'started':time.time(),'last_tick':time.time(),'last_check':0.,
                            'command':(0.,0.,0.),'stable':None,'blocked':None,
                            'blocked_since':None,'clear_since':None,
                            'route_hard_since':None,'motion_elapsed':0.,
                            'reason':connector_reason})
                        state.pop('short_since',None)
                        if direction_recovery:
                            if direction_recovery.get('kind')=='global_plan':
                                publish_plan_event('TRANSITION_POLYLINE_STEP',name,state,
                                    'short_connector_step',0,
                                    fallback_target_index=target)
                            else:
                                publish_direction_event('DIRECTION_RECOVERY_VECTOR_NODE',
                                    name,state,'polyline_step',
                                    verified_target_index=direction_recovery['verified_target'],
                                    next_target_index=target)
                        self.publish_event('TRANSITION_CONNECTOR_START',stage=stage['name'],robot=name,
                            reason=connector_reason,goals={name:goal},target_index=target,
                            active_teb=sum(s['running'] and not s.get('connector') for s in states.values()))
                        continue
                state.pop('short_since',None)
                state['target']=target; state['running']=True; state['connector_done']=False
                state['teb_retry']=None
                if direction_recovery:
                    if direction_recovery.get('kind')=='global_plan':
                        publish_plan_event('TRANSITION_POLYLINE_STEP',name,state,
                            'polyline_teb_step',0,fallback_target_index=target)
                    else:
                        publish_direction_event('DIRECTION_RECOVERY_VECTOR_NODE',name,state,
                            'polyline_teb_step',
                            verified_target_index=direction_recovery['verified_target'],
                            next_target_index=target)
                try:
                    if not send_teb_goal(name,state):
                        state['running']=False
                        waiting[name]='motion_owner_blocked'
                        continue
                except Exception as exc:
                    return False,'TRANSITION_GUIDANCE_UNAVAILABLE',{'robot':name,'error':str(exc)}
                last_guidance[name]=time.time()
                active_count=sum(s['running'] for s in states.values()); peak=max(peak,active_count)
                first=name not in released; released.add(name)
                self.publish_event('TRANSITION_RELEASED' if first else 'TRANSITION_GATE_GRANTED',
                    task=task['id'],robot=name,stage=stage['name'],target_index=target,
                    target_s=round(task['arc'][target],3),active_count=active_count)
            peak=max(peak,sum(s['running'] for s in states.values()))
            recovery_owner=direction_control.get('owner')
            if (recovery_owner and
                    states[recovery_owner].get('direction_local_hold')):
                still_moving=[name for name,state in states.items()
                    if name!=recovery_owner and (state['running'] or
                        self.clients[name].get_state() in self.ACTIVE_GOAL_STATES or
                        not self.robot_stopped(name))]
                if not still_moving:
                    blockers=direction_hold_blockers(recovery_owner,poses)
                    detail={'robot':recovery_owner,
                        'stage':stage['name'],'blockers':blockers,
                        'reason':states[recovery_owner]['direction_local_hold']['reason']}
                    return False,direction_recovery_hold_code(blockers),detail
            slot_holds=[name for name,state in states.items()
                        if state.get('slot_local_hold')]
            if slot_holds:
                still_moving=[name for name,state in states.items()
                    if name not in slot_holds and (state['running'] or
                        self.clients[name].get_state() in self.ACTIVE_GOAL_STATES or
                        not self.robot_stopped(name))]
                if not still_moving:
                    return False,'LOCAL_GOAL_HOLD',{
                        'robots':slot_holds,'reason':'slot_amcl_not_verified'}
            connector_holds=[name for name,state in states.items()
                            if state.get('connector_local_hold')]
            if connector_holds:
                still_moving=[name for name,state in states.items()
                    if name not in connector_holds and (state['running'] or
                        self.clients[name].get_state() in self.ACTIVE_GOAL_STATES or
                        not self.robot_stopped(name))]
                if not still_moving:
                    return False,'LOCAL_GOAL_HOLD',{
                        'robots':connector_holds,
                        'reason':'connector_block_timeout'}
            if time.time()-last_report>=1.0:
                self.publish_event('TRANSITION_PROGRESS',stage=stage['name'],waiting=waiting,
                    progress=dict((n,round(s['progress'],3)) for n,s in states.items()),
                    controllers=dict((n,transition_controller_name(s))
                                     for n,s in states.items()),
                    active_count=sum(s['running'] for s in states.values()),peak_concurrent=peak)
                last_report=time.time()
            if not any(has_reservation(s) or s.get('connector_admission')
                       for s in states.values()):
                idle_since=idle_since or time.time()
                if time.time()-idle_since>deadlock_timeout:
                    return False,'TRANSITION_DEPENDENCY_DEADLOCK',{'waiting':waiting}
            else:
                idle_since=None
            self.stop_requested.wait(0.05)
        return False,'GOAL_TIMEOUT',{'arrived':list(arrived)}

    def execute_current_stage(self):
        stage = self.stages[self.stage_index]
        moving = list(stage["goals"].keys())
        stage_mode = stage.get("mode", "move_base")
        slot_checkpoint_detail = None
        if not self.stage_motion_complete:
            occupancy = self.goal_occupancy(stage["goals"])
            if occupancy:
                with self.lock:
                    self.last_error = "GOAL_OCCUPIED_BY_TEAMMATE"
                    self.state = "GOAL_HOLD"
                    self.checkpoint_state = {
                        "state": "NOT_RUN",
                        "code": self.last_error,
                        "detail": occupancy,
                    }
                self.publish_event(
                    "STAGE_GOAL_REJECTED", stage=stage["name"],
                    code=self.last_error, detail=occupancy)
                return False
            with self.lock:
                self.state = "EXECUTING"
            self.publish_event(
                "STAGE_START", stage=stage["name"], moving=moving,
                mode=stage_mode)
            if stage_mode == "formation_line":
                ok, code, detail = self.run_formation_line(stage, moving)
            elif stage_mode == "transition":
                ok, code, detail = self.run_transition(stage)
            else:
                for name in moving:
                    self.clients[name].send_goal(
                        self.build_goal(stage["goals"][name]))
                ok, code, detail = self.wait_for_goals(
                    moving, stage["goals"], float(stage.get("timeout", 90.0)))
            if not ok:
                if transition_failure_scope(code)=='fleet':
                    self.cancel_all()
                with self.lock:
                    self.last_error = code
                    self.state = ("STOPPED" if code == "STOP_REQUESTED" else
                                  "LOCAL_GOAL_HOLD" if code == "LOCAL_GOAL_HOLD" else
                                  "GOAL_HOLD")
                    self.checkpoint_state = {"state": "NOT_RUN", "code": code,
                                             "detail": detail}
                self.publish_event("STAGE_GOAL_FAILED", stage=stage["name"],
                                   code=code, detail=detail)
                return False
            pending = dict((name, dict(value)) for name, value in self.expected.items())
            for name, goal in stage["goals"].items():
                pending[name] = {
                    "x": float(goal["x"]), "y": float(goal["y"]),
                    "yaw": math.radians(float(goal["yaw_deg"])),
                }
            with self.lock:
                self.pending_expected = pending
                self.stage_motion_complete = True
            if (stage_mode=='transition' and
                    detail.get('slot_amcl_verified')):
                slot_checkpoint_detail=dict((name,value['metrics'])
                    for name,value in detail['slot_certificates'].items())
            self.publish_event("STAGE_GOALS_SUCCEEDED", stage=stage["name"])

        if slot_checkpoint_detail is not None:
            ok,code,detail=True,'CHECKPOINT_PASSED',slot_checkpoint_detail
            with self.lock:
                self.checkpoint_state={"state":"VERIFIED_BY_SLOT_AMCL",
                                       "metrics":detail}
            self.publish_event('CHECKPOINT_REUSED_SLOT_AMCL',stage=stage['name'],
                robots=sorted(slot_checkpoint_detail),
                reason='each transition endpoint has a stable AMCL certificate')
        else:
            ok, code, detail = self.run_checkpoint(self.pending_expected, moving)
        if not ok:
            with self.lock:
                self.last_error = code
                self.state = ("STOPPED" if code == "STOP_REQUESTED"
                              else "LOCALIZATION_HOLD")
                self.checkpoint_state = {
                    "state": "FAILED", "code": code, "detail": detail,
                }
            self.publish_event("CHECKPOINT_FAILED", stage=stage["name"],
                               code=code, detail=detail)
            return False

        with self.lock:
            self.expected = self.pending_expected
            self.pending_expected = None
            self.stage_motion_complete = False
            self.stage_index += 1
            self.slot_certificates.pop(stage['name'], None)
            self.last_error = None
            self.checkpoint_state = {"state": "PASSED", "metrics": detail}
            self.state = ("SUCCEEDED" if self.stage_index >= len(self.stages)
                          else "READY")
        self.publish_event("STAGE_PASSED", stage=stage["name"], metrics=detail)
        self.publish_state()
        return True

    def run_worker(self, run_all):
        try:
            while not rospy.is_shutdown() and not self.stop_requested.is_set():
                if self.stage_index >= len(self.stages):
                    break
                if not self.execute_current_stage():
                    break
                if not run_all:
                    break
            with self.lock:
                self.active = False
                self.worker = None
        except Exception as exc:
            self.cancel_all()
            with self.lock:
                self.active = False
                self.worker = None
                self.state = "FAILED"
                self.last_error = "supervisor exception: %s" % exc
            self.publish_event("SUPERVISOR_FAILED", error=str(exc))
        self.publish_state()

    def begin(self, run_all):
        if not self.motion_enabled:
            return TriggerResponse(False, "motion_enabled is false")
        if run_all and not self.allow_full_run:
            return TriggerResponse(False, "allow_full_run is false; use start_next")
        with self.lock:
            if self.active:
                return TriggerResponse(False, "a stage is already active")
            if self.stage_index >= len(self.stages):
                return TriggerResponse(False, "mission complete; reset after returning to start")
            errors = self.cached_preflight_errors()
            if errors:
                self.last_error = "; ".join(errors)
                if self.state not in ("GOAL_HOLD", "LOCALIZATION_HOLD"):
                    self.state = "NOT_READY"
                message = self.last_error
            else:
                message = None
                retry_checkpoint = self.stage_motion_complete
                stage_name = self.stages[self.stage_index]["name"]
                self.active = True
                self.stop_requested.clear()
                self.last_error = None
                self.worker = threading.Thread(target=self.run_worker, args=(run_all,))
                self.worker.daemon = True
                self.worker.start()
        if message is not None:
            self.publish_state()
            return TriggerResponse(False, "preflight failed: %s" % message)
        if run_all:
            return TriggerResponse(True, "started remaining mission")
        if retry_checkpoint:
            return TriggerResponse(True, "retrying checkpoint for %s" % stage_name)
        return TriggerResponse(True, "started %s" % stage_name)

    def start_next(self, _request):
        return self.begin(False)

    def start_all(self, _request):
        return self.begin(True)

    def stop(self, _request):
        self.stop_requested.set()
        self.cancel_all()
        with self.lock:
            if not self.active:
                self.state = "STOPPED"
        self.publish_event("STOP_REQUESTED")
        self.publish_state()
        return TriggerResponse(True, "all fleet motion stopped")

    def reset(self, _request):
        with self.lock:
            if self.active:
                return TriggerResponse(False, "stop the active stage first")
            self.stage_index = 0
            self.slot_certificates = {}
            self.expected = self.initial_expected()
            self.pending_expected = None
            self.stage_motion_complete = False
            self.last_error = None
            self.checkpoint_state = {"state": "IDLE"}
            self.state = "STARTING"
            self.stop_requested.clear()
            self.static_preflight_complete = False
            self.startup_nomotion_attempts = dict(
                (name, 0) for name in self.robot_order)
            self.startup_nomotion_inflight = set()
        self.publish_event("MISSION_RESET")
        self.publish_state()
        return TriggerResponse(True, "reset to first stage; return cars to floor marks")

    def shutdown(self):
        self.stop_requested.set()
        self.cancel_all()


if __name__ == "__main__":
    try:
        FleetVendorOmniSupervisor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
