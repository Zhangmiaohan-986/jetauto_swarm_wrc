#!/usr/bin/env bash
# Foreground VM master only. Never stops a master, including the old .111 one.
set -euo pipefail
[[ $# -eq 0 || ( $# -eq 1 && "$1" == --check ) ]] || {
  echo 'Usage: start_ten_master.sh [--check]' >&2; exit 2;
}
source "$(dirname "${BASH_SOURCE[0]}")/ten_environment.bash" real
if ! ip -o -4 addr show | awk '{print $4}' | cut -d/ -f1 | grep -Fxq "$FLEET_VM_IP"; then
  echo "ERROR: VM IP $FLEET_VM_IP is not assigned here; check the VM network/runtime table." >&2
  exit 2
fi
if python - "$ROS_MASTER_URI" <<'PY'
from __future__ import print_function
import errno
import socket
import sys
import xmlrpclib
from urlparse import urlparse
socket.setdefaulttimeout(3.0)
expected = sys.argv[1]
try:
    master = xmlrpclib.ServerProxy(expected)
    code, message, actual = master.getUri('/fleet_master_check')
    if code != 1 or (urlparse(actual).hostname, urlparse(actual).port) != (
            urlparse(expected).hostname, urlparse(expected).port):
        print('ERROR: Master identity mismatch: expected %s, got %r' % (expected, actual))
        sys.exit(2)
    print('ROS Master already online: %s' % actual)
except socket.error as exc:
    if getattr(exc, 'errno', None) == errno.ECONNREFUSED:
        sys.exit(1)
    print('ERROR: Master check failed; not starting a duplicate: %s' % exc)
    sys.exit(2)
except Exception as exc:
    print('ERROR: Master check failed; not starting a duplicate: %s' % exc)
    sys.exit(2)
PY
then
  exit 0
else
  result=$?
  [[ "$result" -eq 1 ]] || exit "$result"
fi
if [[ "${1:-}" == --check ]]; then
  echo "ERROR: $ROS_MASTER_URI is offline; run fleetmaster in a separate VM terminal." >&2
  exit 1
fi
if [[ "$FLEET_MASTER_IP" != "$FLEET_VM_IP" ]]; then
  echo "ERROR: selected master $FLEET_MASTER_IP is not this VM; start it explicitly on its host." >&2
  exit 2
fi
echo "Starting $ROS_MASTER_URI in this terminal. Existing masters elsewhere are untouched."
exec roscore -p 11311
