"""Offline test. Fake SSH records commands; never contacts a robot."""
import os
from pathlib import Path
import subprocess
import tempfile

script = Path(__file__).resolve().parents[1] / 'scripts/fleet_admin/poweroff_ten.sh'
subprocess.run(['bash', '-n', str(script)], check=True)
with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    mock = '''#!/usr/bin/env python3
import os, sys, shlex, subprocess
args = sys.argv
host = next(a for a in args if a.startswith('jetauto@'))
command = args[-1]
with open(os.environ['MOCK_LOG'], 'a') as f:
    f.write(host + (' probe' if command == 'true' else ' shutdown') + '\\n')
if host.endswith('.114'):
    sys.exit(255)
if command != 'true':
    payload = subprocess.check_output(['bash', '-c', 'sudo() { printf "%s" "${@: -1}"; }; ' + command], universal_newlines=True)
    subprocess.run(['bash', '-n'], input=payload, universal_newlines=True, check=True)
    assert payload.index('sync') < payload.index('pgrep') < payload.index('pkill -INT') < payload.rindex('sync') < payload.index('systemctl poweroff')
    assert 'pkill -KILL' not in payload
    assert '/home/jetauto/swarm_logs/robot_[1-9][0-9]*_transport[.]launch' in payload
    if host.endswith('.118'):
        sys.exit(8)
'''
    for name in ('sshpass', 'ssh'):
        path = root / name
        path.write_text(mock)
        path.chmod(0o700)
    env = dict(os.environ, PATH=directory + ':' + os.environ['PATH'],
               MOCK_LOG=str(root/'calls'), SSH_USER='jetauto')
    result = subprocess.run(['bash', str(script)], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    assert result.returncode == 1, result.stderr
    lines = (root/'calls').read_text().splitlines()
    probes = [line for line in lines if line.endswith('probe')]
    assert probes == ['jetauto@192.168.1.%d probe' % n for n in range(111, 121)]
    assert not any('.114 shutdown' in line for line in lines)
    assert lines[-1] == 'jetauto@192.168.1.120 shutdown'
    assert 'SKIP: robot_4' in result.stdout and 'CHECK: robot_8' in result.stderr
print('PASS: syntax, 111-120 order, offline skip, failure continuation, remote command order; no real SSH')
