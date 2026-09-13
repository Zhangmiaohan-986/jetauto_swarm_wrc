#!/usr/bin/env python
"""Load one fleet profile and compose the shared real/sim ROS launch."""
from __future__ import print_function
import math
import hashlib
import os
import subprocess
import copy
import xml.etree.ElementTree as ET
import yaml

try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)

FIELDS = ('fuse_imu_yaw', 'imu_bias_estimation', 'max_vel_x', 'max_vel_y',
          'max_vel_x_backwards', 'max_vel_theta', 'acc_lim_x', 'acc_lim_y',
          'acc_lim_theta', 'amcl_update_min_d', 'amcl_update_min_a',
          'amcl_recovery_alpha_slow', 'amcl_recovery_alpha_fast')

def resolve(root, path):
    return path if os.path.isabs(path) else os.path.join(root, path)


def filter_active_fleet(mission, roster, excluded):
    """Return a mission/roster view with selected robots temporarily offline.

    The source ten-robot profile is never mutated.  Removing a robot also
    removes its stage goals/tasks and references from dependencies and gated
    reservations, so the supervisor receives one internally consistent fleet
    rather than a partial ten-robot configuration.
    """
    if isinstance(excluded, string_types):
        excluded = excluded.replace(',', ' ').split()
    excluded = set(name.strip() for name in (excluded or []) if name.strip())
    known = set(item['name'] for item in roster)
    unknown = excluded - known
    if unknown:
        raise ValueError('unknown excluded robot(s): %s' % ','.join(sorted(unknown)))
    if len(excluded) >= len(known):
        raise ValueError('at least one robot must remain active')

    result = copy.deepcopy(mission)
    names = [name for name in result['robot_order'] if name not in excluded]
    result['robot_order'] = names
    result['robots'] = dict((name, result['robots'][name]) for name in names)
    result['runtime_profiles'] = dict(
        (name, result['runtime_profiles'][name]) for name in names)

    for stage in result.get('stages', []):
        goals = stage.get('goals')
        if isinstance(goals, dict):
            stage['goals'] = dict((name, goal) for name, goal in goals.items()
                                  if name not in excluded)
        tasks = stage.get('tasks')
        if not isinstance(tasks, list):
            continue
        removed_ids = set(task.get('id') for task in tasks
                          if task.get('robot') in excluded)
        stage['tasks'] = [task for task in tasks
                          if task.get('robot') not in excluded]
        for task in stage['tasks']:
            if isinstance(task.get('after'), list):
                task['after'] = [item for item in task['after']
                                 if item not in removed_ids]
            if isinstance(task.get('gates'), list):
                task['gates'] = [dict((name, value) for name, value in gate.items()
                                      if name not in excluded)
                                 for gate in task['gates']]

    active_roster = [item for item in roster if item['name'] in names]
    return result, active_roster

def load_profile(root, profile_path):
    with open(resolve(root, profile_path)) as stream:
        profile = yaml.safe_load(stream)
    with open(resolve(root, profile['mission'])) as stream:
        mission = yaml.safe_load(stream)
    runtime = os.environ.get('FLEET_RUNTIME_CONFIG',resolve(root, profile['runtime']))
    # Runtime files are trusted, user-editable local bash configuration.
    script = 'source "$1"; printf "%s\\n" "${FLEET_RUNTIME_ROWS[@]}"; printf "FORMATION|%s|%s|%s|%s\\n" "$FLEET_FORMATION_SPEED" "$FLEET_FORMATION_MIN_SPEED" "$FLEET_FORMATION_MAX_LATERAL" "$FLEET_FORMATION_MAX_ANGULAR"; printf "TRANSITION|%s|%s\\n" "${FLEET_TRANSITION_VIA_WEIGHT:-10}" "${FLEET_TRANSITION_MOTION_WEIGHT:-20}"'
    script += '; printf "TRANSPORT|%s\\n" "${FLEET_LOCAL_TF:-false}"'
    lines = subprocess.check_output(['bash', '-eu', '-c', script, 'fleet-runtime', runtime]).decode('utf-8').splitlines()
    roster, settings = [], {}
    for line in lines:
        values = line.split('|')
        if values[0] == 'TRANSPORT':
            if values[1] not in ('true', 'false'):
                raise ValueError('FLEET_LOCAL_TF must be true or false')
            profile['per_robot_tf'] = values[1] == 'true'
            continue
        if values[0] == 'TRANSITION':
            via_weight=float(values[1])
            motion_weight=float(values[2])
            if any(math.isnan(v) or math.isinf(v) or v<=0 for v in (via_weight,motion_weight)):
                raise ValueError('Transition weights must be finite and positive')
            continue
        if values[0] == 'FORMATION':
            for name, value in zip(('speed', 'min_speed', 'max_lateral', 'max_angular'), values[1:]):
                mission.setdefault('formation_line', {})[name] = float(value)
            continue
        if len(values) != 16:
            raise ValueError('runtime row needs 16 fields: %s' % values[0])
        name, ip, hostname = values[:3]
        if name in settings:
            raise ValueError('duplicate robot %s' % name)
        row = {}
        for field, value in zip(FIELDS, values[3:]):
            if field in ('fuse_imu_yaw', 'imu_bias_estimation'):
                if value not in ('true', 'false'):
                    raise ValueError('%s must be true or false' % field)
                row[field] = value == 'true'
            else:
                row[field] = float(value)
                if math.isnan(row[field]) or math.isinf(row[field]):
                    raise ValueError('non-finite runtime value')
                if row[field] < 0:
                    raise ValueError('negative runtime value')
        roster.append({'name': name, 'ip': ip, 'factory_hostname': hostname})
        settings[name] = row
    if [r['name'] for r in roster] != mission['robot_order']:
        raise ValueError('runtime roster and mission robot_order differ')
    if len(set(r['ip'] for r in roster)) != len(roster):
        raise ValueError('duplicate robot IP')
    if set(mission['robots']) != set(mission['robot_order']):
        raise ValueError('initial poses and robot_order differ')
    profile['map_yaml'] = resolve(root, profile['map_yaml'])
    check_map_evidence(profile['map_yaml'], mission)
    formation = mission['formation_line']
    for key in ('speed','min_speed','max_lateral','max_angular'):
        value = formation[key]
        if math.isnan(value) or math.isinf(value) or value < 0:
            raise ValueError('invalid formation_line '+key)
    if formation['speed'] <= 0 or formation['min_speed'] > formation['speed']:
        raise ValueError('formation min_speed must not exceed positive speed')
    mission['runtime_profiles'] = settings
    if any(s.get('scheduler')=='progress_gates' for s in mission['stages']):
        mission['transition_via_weight']=via_weight
        mission['transition_motion_weight']=motion_weight
        mission['required_parameter_suffixes'].update({
            'move_base/TebLocalPlannerROS/global_plan_viapoint_sep': -0.1,
            'move_base/TebLocalPlannerROS/via_points_ordered': False,
            'move_base/TebLocalPlannerROS/weight_viapoint': via_weight})
        for key in ('weight_max_vel_x','weight_max_vel_y','weight_max_vel_theta',
                    'weight_acc_lim_x','weight_acc_lim_y','weight_acc_lim_theta'):
            mission['required_parameter_suffixes']['move_base/TebLocalPlannerROS/'+key]=motion_weight
    return profile, mission, roster

def check_map_evidence(path, mission):
    expected = mission.get('planning_evidence',{}).get('map_sha256')
    if expected:
        with open(path,'rb') as stream: metadata = stream.read()
        image = resolve(os.path.dirname(path), yaml.safe_load(metadata)['image'])
        with open(image,'rb') as stream: pixels = stream.read()
        if hashlib.sha256(metadata+pixels).hexdigest() != expected:
            raise ValueError('Map changed: regenerate this profile mission and validate simulation')

def text_value(value):
    return ('true' if value else 'false') if isinstance(value, bool) else str(value)

def parameter(parent, name, value):
    return ET.SubElement(parent, 'param', name=name, value=text_value(value))

def validate_initialization(initialization, robots):
    rate = float(initialization['save_pose_rate'])
    if math.isnan(rate) or math.isinf(rate) or rate == 0:
        raise ValueError('AMCL save_pose_rate must be finite and nonzero (negative disables saving)')
    for key in ('initial_cov_xx', 'initial_cov_yy', 'initial_cov_aa'):
        value = float(initialization[key])
        if math.isnan(value) or math.isinf(value) or value <= 0:
            raise ValueError('AMCL %s must be finite and positive' % key)
    for name, pose in robots.items():
        for key in ('initial_x', 'initial_y', 'initial_yaw_deg'):
            value = float(pose[key])
            if math.isnan(value) or math.isinf(value):
                raise ValueError('%s %s must be finite' % (name, key))

def build_launch(root, profile, mission, roster, simulation=False, gui=True, record=True):
    with open(os.path.join(root,'config/fleet_vendor_omni/amcl_initialization.yaml')) as stream:
        validate_initialization(yaml.safe_load(stream), mission['robots'])
    launch = ET.Element('launch')
    names = mission['robot_order']
    if profile.get('vm_sensor_relay', False):
        sensor_topics = ['/%s/%s' % (name, suffix)
                         for name in names for suffix in ('scan', 'odom', 'imu')]
        for topic in sensor_topics:
            ET.SubElement(launch, 'remap', **{'from': topic, 'to': '/fleet'+topic})
        if not simulation:
            sensor = ET.SubElement(launch, 'node', pkg='jetauto_multi',
                type='fleet_sensor_ingress', name='fleet_sensor_ingress',
                required='true', output='screen')
            ET.SubElement(sensor, 'rosparam', param='robots').text = yaml.safe_dump(names)
            parameter(sensor, 'per_robot_tf', profile.get('per_robot_tf', False))
            # This node alone reads physical streams; never remap its inputs.
            for topic in sensor_topics:
                ET.SubElement(sensor, 'remap', **{'from': topic, 'to': topic})
    if profile.get('vm_tf_relay', False):
        # Upper nodes share VM-local TF; only the two ingresses contact bottoms.
        for topic in ('/tf', '/tf_static'):
            ET.SubElement(launch, 'remap', **{'from': topic, 'to': '/fleet'+topic})
        if not simulation:
            ingresses = []
            if not profile.get('per_robot_tf', False):
                dynamic = ET.SubElement(launch, 'node', pkg='topic_tools',
                    type='relay', name='fleet_tf_dynamic_relay', args='/tf /fleet/tf',
                    required='true', output='screen')
                ingresses.append(dynamic)
            elif not profile.get('vm_sensor_relay', False):
                raise ValueError('per_robot_tf requires the native VM sensor ingress')
            ingress = ET.SubElement(launch, 'node', pkg='jetauto_multi',
                type='tf_ingress.py', name='fleet_tf_ingress', required='true', output='screen')
            ingresses.append(ingress)
            if profile.get('per_robot_tf', False):
                ET.SubElement(ingress, 'rosparam', param='robots').text = yaml.safe_dump(names)
            # Native relay handles high-rate dynamic traffic without Python
            # serialization; Python only merges the low-rate latched fixed links.
            # Override inherited remaps to prevent either ingress feeding itself.
            for node in ingresses:
                for topic in ('/tf', '/tf_static'):
                    ET.SubElement(node, 'remap', **{'from': topic, 'to': topic})
    map_frame = mission.get('map_frame', 'robot_1/map')
    map_topic = '/' + map_frame
    server = ET.SubElement(launch, 'node', pkg='map_server', type='map_server',
                           name='fleet_vendor_map_server', ns=map_frame.rsplit('/',1)[0],
                           args=profile['map_yaml'], output='screen', required='true')
    parameter(server, 'frame_id', map_frame)
    if simulation:
        adapter = ET.SubElement(launch, 'node', pkg='jetauto_multi', type='sim_adapter.py',
                                name='fleet_vendor_omni_sim_adapter', output='screen', required='true')
        ET.SubElement(adapter, 'rosparam').text = yaml.safe_dump(mission, default_flow_style=False)
        parameter(adapter, 'map_topic', map_topic)
        parameter(adapter, 'time_scale', 1.0)
        parameter(adapter, 'scan_rate', 10.0)
        parameter(adapter, 'scan_beams', 181)
    for index, name in enumerate(names, 1):
        include = ET.SubElement(launch, 'include', file=os.path.join(root,'launch/fleet_vendor_omni/include/robot_navigation.launch'))
        pose = mission['robots'][name]
        args = dict(mission['runtime_profiles'][name])
        args.update(robot_name=name, robot_id=index, robot_count=len(names),
                    use_transition_via_points=any(s.get('scheduler')=='progress_gates' for s in mission['stages']),
                    transition_via_weight=mission.get('transition_via_weight',10.0),
                    transition_motion_weight=mission.get('transition_motion_weight',20.0),
                    enable_amcl=not simulation, map_frame=map_frame, map_topic=map_topic,
                    global_costmap_sensor_topic='/%s/amcl_scan'%name,
                    initial_x=pose['initial_x'], initial_y=pose['initial_y'],
                    initial_yaw=math.radians(pose['initial_yaw_deg']))
        for key, value in sorted(args.items()):
            ET.SubElement(include, 'arg', name=key, value=text_value(value))
        if simulation:
            # The adapter owns map->odom->base_footprint only. Reuse the vendor
            # visual model and publish its internal links in each robot namespace.
            model = ET.SubElement(launch,'group',ns=name)
            ET.SubElement(model,'param',name='robot_description',command=
                '/usr/bin/env MACHINE_TYPE=JetAuto LIDAR_TYPE=A1 DEPTH_CAMERA_TYPE=AstraProPlus '
                '$(find xacro)/xacro "$(find jetauto_description)/urdf/jetauto.xacro"')
            state = ET.SubElement(model,'node',pkg='robot_state_publisher',
                                  type='robot_state_publisher',name='robot_state_publisher')
            parameter(state,'tf_prefix',name)
            joints = ET.SubElement(model,'node',pkg='joint_state_publisher',
                                   type='joint_state_publisher',name='joint_state_publisher')
            parameter(joints,'use_gui',False)
            parameter(joints,'rate',1)
            # Declarative simulation contract: no physical IMU filter or EKF runs.
            parameter(launch, '/%s/imu_filter/do_bias_estimation'%name, args['imu_bias_estimation'])
            parameter(launch, '/%s/amcl/odom_model_type'%name, 'omni')
            parameter(launch, '/%s/amcl/resample_interval'%name, 2)
            imu_config = [False]*15
            imu_config[5], imu_config[11] = args['fuse_imu_yaw'], True
            ET.SubElement(launch,'rosparam',param='/%s/ekf_localization/imu0_config'%name).text = yaml.safe_dump(imu_config)
    supervisor = ET.SubElement(launch,'node',pkg='jetauto_multi',type='supervisor.py',
                                name='fleet_vendor_omni',output='screen',required='true')
    ET.SubElement(supervisor,'rosparam').text = yaml.safe_dump(mission, default_flow_style=False)
    parameter(supervisor,'simulation',simulation)
    parameter(supervisor,'transition_map_yaml',profile['map_yaml'])
    if record:
        recorder = ET.SubElement(launch,'node',pkg='jetauto_multi',type='record.sh',name='fleet_vendor_omni_recorder',output='screen')
        ET.SubElement(recorder,'env',name='FLEET_ROBOT_COUNT',value=str(len(names)))
        ET.SubElement(recorder,'env',name='FLEET_MAP_TOPIC',value=map_topic)
        ET.SubElement(recorder,'env',name='FLEET_RECORD_BOTTOM_RAW',
                      value=text_value(profile.get('record_bottom_raw', True)))
    if gui:
        rviz_path = profile.get('rviz', 'rviz/fleet_vendor_omni_six.rviz')
        ET.SubElement(launch,'node',pkg='rviz',type='rviz',name='fleet_vendor_omni_rviz',
                      args='-d '+resolve(root,rviz_path),output='screen')
    return ET.tostring(launch, encoding='utf-8')
