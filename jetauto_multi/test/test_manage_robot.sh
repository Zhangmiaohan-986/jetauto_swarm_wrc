#!/usr/bin/env bash
# Bounded offline check: SSH/ping are mocked; no robot is contacted.
set -euo pipefail
admin="$(cd "$(dirname "$0")/../scripts/fleet_admin" && pwd)"
for file in manage_robot.sh manage_ten_bottom.sh start_fleet_bottom.sh; do
  bash -n "$admin/$file"
done
python3 - "$admin" <<'PY'
import os, pathlib, subprocess, sys, tempfile, re, xml.etree.ElementTree as ET
admin = pathlib.Path(sys.argv[1])
wrapper = ET.parse(str(admin/'../../launch/fleet_vendor_omni/include/robot_bottom_transport.launch')).getroot()
assert {(x.get('from'), x.get('to')) for x in wrapper.findall('remap')} == {
    ('/tf', '/$(arg robot_name)/tf'), ('/tf_static', '/$(arg robot_name)/tf_static')}
assert wrapper.find('include').get('file') == '$(find jetauto_slam)/launch/jetauto_robot.launch'
with tempfile.TemporaryDirectory() as tmp:
    root = pathlib.Path(tmp)
    mock = '''#!/usr/bin/env bash
case "$*" in
  *"bash -s"*)
    payload=$(</dev/stdin)
    bash -n <<< "$payload" || exit 90
    [[ "$payload" == *"pkill -INT"* && "$payload" != *"pkill -KILL"* ]] || exit 91
    printf '%s\\n' "$payload" >> "$TEST_LOG"
    exit 0 ;;
  *"pgrep -af"*)
    [[ -f "$TEST_STARTED" ]] && exit 0
    exit 1 ;;
  *"nohup bash"*) touch "$TEST_STARTED" ;;
esac
printf '%s\\n' "$*" >> "$TEST_LOG"
'''
    for name, content in [('sshpass', mock), ('ping', '#!/bin/sh\nexit 0\n'), ('sleep', '#!/bin/sh\nexit 0\n')]:
        path = root / name
        path.write_text(content)
        path.chmod(0o700)
    env = dict(os.environ, PATH=tmp + ':' + os.environ['PATH'], TEST_LOG=tmp+'/log', TEST_STARTED=tmp+'/started',
               FLEET_WIFI_POWER_SAVE='off',
               FLEET_LOCAL_TF='false',
               FLEET_RUNTIME_CONFIG=str(admin / '../../config/fleet_vendor_omni/profiles/fleet10_same_map/runtime.bash'))
    for action in ['stop', 'start', 'app-stop', 'status']:
        subprocess.run(['bash', str(admin/'manage_robot.sh'), '4', action], env=env, check=True)
    log = (root/'log').read_text()
    assert '192.168.1.114' in log and 'fuse_imu_yaw:=false' in log
    assert 'FLEET_IMU_BIAS_ESTIMATION=false' in log
    assert "iw dev wlan0 set power_save 'off' && iw dev wlan0 get power_save" in log
    assert 'exec roscore' not in log
    stop_pattern = re.search("pkill -INT -f '([^']+)'", log).group(1)
    assert re.match(stop_pattern, 'roslaunch jetauto_slam jetauto_robot.launch sim:=false')
    assert re.match(stop_pattern, 'roslaunch /home/jetauto/swarm_logs/robot_4_transport.launch robot_name:=robot_4')
    assert not re.match(stop_pattern, 'roslaunch /home/jetauto/swarm_logs/robot_5_transport.launch robot_name:=robot_5')
    assert not re.match(stop_pattern, 'roscore')
    invalid_wifi = dict(env, FLEET_WIFI_POWER_SAVE='invalid')
    assert subprocess.run(['bash', str(admin/'manage_robot.sh'), '4', 'start'],
                          env=invalid_wifi).returncode == 2
    assert (root/'log').read_text() == log, 'invalid Wi-Fi setting must reject before SSH'
    invalid_tf = dict(env, FLEET_LOCAL_TF='invalid')
    assert subprocess.run(['bash', str(admin/'manage_robot.sh'), '4', 'start'], env=invalid_tf).returncode == 2
    (root/'started').unlink()
    subprocess.run(['bash', str(admin/'manage_robot.sh'), '4', 'start'],
                   env=dict(env, FLEET_LOCAL_TF='true'), check=True)
    transport_log = (root/'log').read_text()[len(log):]
    assert 'scp ' in transport_log and 'robot_4_transport.launch.before_fleet4' in transport_log
    assert 'exec roslaunch /home/jetauto/swarm_logs/robot_4_transport.launch robot_name:=robot_4 fuse_imu_yaw:=false' in transport_log
    assert 'export FLEET_LOCAL_TF=true' in transport_log
    for args in [('0','stop'), ('11','start'), ('4','invalid')]:
        assert subprocess.run(['bash', str(admin/'manage_robot.sh'), *args], env=env).returncode != 0
print('PASS: syntax, mocked start/stop/app-stop/status, identity, Wi-Fi and local-TF switches, wrapper copy/backup, invalid inputs')
PY
