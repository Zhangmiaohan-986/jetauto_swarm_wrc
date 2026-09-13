# Ten-robot bottom-layer operations

Run on the VM. Robot N maps to 192.168.1.(110+N); physical Robot1 remains
.111 (and its factory/vendor identity is retained in the runtime table). The
ten-robot ROS Master is the VM at 192.168.1.104; .111 is not the ten-robot
Master.
Stop the VM mission/navigation command publishers before stopping/restarting
bottom layers. A live upper layer can send motion as soon as a bottom restarts.
These commands do not power off the computers and do not deliberately stop roscore.

```bash
cd /home/ubuntu/jetauto_real_dev_ws/src/jetauto_multi/scripts/fleet_admin
# Terminal 1, on the VM: start the ten-robot Master (shortcut is also available).
fleetmaster
# Or, from this directory:
source ./ten_environment.bash real
bash start_ten_master.sh

# Terminal 2, from this directory: restart the selected physical bottoms.
bash manage_ten_bottom.sh restart 1 2 3 4 5 6 7 8 9
```

The dispatcher submits selected robots concurrently and reports individual failures.
Offline robots cause a FAIL summary, but do not prevent other selected robots from
being processed. Success means a launch process was detected, NOT full ROS READY.
Use the existing staggered start_ten_bottom.sh if simultaneous startup overloads Wi-Fi.

```bash
# All ten: interfaces adapted; runtime readiness still requires checking.
# Run in the second terminal after fleetmaster is ready.
bash manage_ten_bottom.sh restart all
# Stop vendor APP service only (does not disable it across reboot).
bash manage_ten_bottom.sh app-stop all
# Stop vendor APP and send SIGINT (Ctrl+C equivalent) to fleet bottom roslaunch.
bash manage_ten_bottom.sh stop all
# Start stopped bottom layers; leaves existing fleet launches unchanged.
bash manage_ten_bottom.sh start all

# Single robot, same table and implementation.
bash manage_ten_bottom.sh stop 4
bash manage_ten_bottom.sh start 4
bash manage_ten_bottom.sh restart 4
bash manage_ten_bottom.sh app-stop 4
bash manage_ten_bottom.sh status 4
bash manage_ten_bottom.sh logs 4

# Full bottom readiness check, expects all ten online and the VM Master at .104.
bash check_ten_bottom.sh
```

`restart` means bottom-layer restart, not Ubuntu reboot. Ctrl+C in the dispatcher
terminal is NOT a remote bottom stop: the launch processes are detached. Use `stop`.
If graceful stop times out, inspect logs; no forced SIGKILL/relaunch is performed.
Robot1 is the physical `.111` vehicle, not the ten-robot Master. The VM
`fleetmaster` process is kept separate from vehicle bottom restarts. If a
legacy vendor APP on Robot1 owns a roscore, stopping that APP can still stop
its children; do not use that legacy roscore for the ten-robot mainline.

If a whole computer reboot is explicitly needed, first stop the mission and that
robot's bottom, then (example Robot4):

```bash
ssh -t jetauto@192.168.1.114 'sudo reboot'
```

After reboot, the vendor APP may autostart again. Use the fleet start/restart action
to stop it and apply the fleet environment. Rebooting Robot1 also loses its roscore.

## Interface adaptation on 2026-09-07

### Whole-fleet Ubuntu shutdown

After stopping the VM motion task, run `bash poweroff_ten.sh` from the admin
directory. No ROS sourcing is required. It follows .111 through .120 sequentially:
sync, inspect bottom launches, stop vendor APP, SIGINT remaining bottom launches,
wait up to 20 seconds, sync again, request system poweroff. Unreachable or failed
hosts are reported and processing continues. A bottom-stop failure skips that
host's poweroff. This intentionally shuts the ROS Master host down first, so the
upper task MUST already be stopped. SSH disconnection is reported as uncertain,
not proof of shutdown. Turn off physical power only after the OS has shut down.

Robot4/8/9: adapted only imu_base.launch, jetauto_robot.launch,
jetauto_controller.launch and ekf.launch. Each original is saved adjacent as
`<filename>.before_fleet4`, `.before_fleet8` or `.before_fleet9` respectively.
Robot1/2/3/5/6/7 already had the same interfaces and were not overwritten.
Robot10 was powered off and was not contacted or changed in this deployment.

Follow-up: Robot10's same four interfaces have now been backed up by SCP and
adapted, with adjacent `.before_fleet10` backups. Parameter parsing passed;
no runtime restart was performed. Its previous EKF exit -11 must be checked
after restart, not assumed fixed. The checker now uses Melodic `rosnode list -a`
instead of the unsupported `rosnode get-uri` that caused ten false IP failures.

SCP backups for Robot1-9:
`/home/ubuntu/experiment_records/20260907/bottom_interfaces/robot_N/`.
Windows copy: `F:/swarm_src/experiment_records/20260907/bottom_interfaces/robot_N/`.
The four top-level launch files there are BEFORE copies, not installation payloads.

Validation: roslaunch --dump-params on Robot1-9 confirmed dynamic bias false,
IMU absolute yaw fusion false, yaw-rate fusion true; no ROS nodes were launched.
Run-time readiness and real motion remain untested after this file deployment.
The switches still default to factory behavior unless passed by the fleet launcher.
Do not rely on editing a terminal variable to change an already-running node.
