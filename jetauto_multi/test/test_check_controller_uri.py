"""Test actual checker block with mocked ROS output; no network or hardware."""
from pathlib import Path
import subprocess

script = Path(__file__).resolve().parents[1] / 'scripts/fleet_admin/check_fleet_bottom.sh'
source = script.read_text()
subprocess.check_call(['bash', '-n', str(script)])
start = source.index('  controller_uri=')
end = source.index('  if [[ "$actual_bias"', start)
block = source[start:end]
cases = [
    ('http://192.168.1.111:1234/\t/robot_1/ros_robot_controller', 'PASS'),
    ('http://192.168.1.112:1234/\t/robot_1/ros_robot_controller', 'FAIL namespace/IP mismatch'),
    ('http://192.168.1.111:1234/\t/robot_10/ros_robot_controller', 'FAIL controller URI lookup unavailable'),
    ('rosnode is a command-line tool for printing information about ROS Nodes.', 'FAIL controller URI lookup unavailable'),
    ('', 'FAIL controller URI lookup unavailable'),
]
for output, expected in cases:
    harness = '''ns=/robot_1; ip=192.168.1.111
pass() { echo "PASS $*"; }
fail() { echo "FAIL $*"; }
timeout() { shift; "$@"; }
rosnode() { [[ "$*" == 'list -a /robot_1/ros_robot_controller' ]] || exit 99; printf '%s\\n' "$1_OUTPUT"; }
'''.replace('"$1_OUTPUT"', '"$MOCK_OUTPUT"')
    import os
    result = subprocess.check_output(['bash', '-c', harness + block],
                                     env=dict(os.environ, MOCK_OUTPUT=output), universal_newlines=True)
    assert result.startswith(expected), result
print('PASS: Melodic URI lookup, wrong IP, exact namespace, help/empty output rejection')
