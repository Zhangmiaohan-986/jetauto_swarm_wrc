# jetauto_multi

This package contains one shared six/ten-robot `fleet_vendor_omni` implementation. Old
multi-robot, measured-anchor, Robot2 diagnostic, three-robot, and ten-robot
implementations are kept outside this package under
`F:\swarm_src\archive\jetauto_multi_legacy_20260906`.

## Public entry points

```text
launch/real.launch                   # six physical robots
launch/sim.launch                    # six-robot controller-parity simulation
launch/ten_real.launch               # ten-robot candidate; see validation status
launch/ten_sim.launch                # ten-robot controller-parity simulation
```

Each entry uses `launch_fleet.py` to load one profile and instantiate the shared
`robot_navigation.launch`. No duplicated six/ten controller or per-robot launch list.
Ten-robot layout, network, commands and current validation status:
[FLEET10_RUNBOOK.md](docs/FLEET10_RUNBOOK.md).

Progress-gated concurrent transitions, TEB route guidance, parameters and rollback:
[PROGRESS_GATED_TRANSITIONS.md](docs/PROGRESS_GATED_TRANSITIONS.md).

New-map workflow and eight reserve formation templates (not automatically enabled):
[NEW_MAP_FORMATION_WORKFLOW.md](docs/NEW_MAP_FORMATION_WORKFLOW.md).

The six-robot baseline is unchanged:

```text
config/fleet_vendor_omni/six_robot_demo.yaml
config/fleet_vendor_omni/six_robot_runtime.bash
config/fleet_vendor_omni/teb_defaults.yaml
```

The real lower layer is the vendor workspace on each robot. The upper layer
uses vendor AMCL and `move_base + TEB` for transition routes, and the shared
supervisor's `formation_line` controller for straight formation routes. No
independent safety guard is included in this current release candidate.

On the VM, start the simulation with:

```bash
bash scripts/fleet_admin/start_vendor_omni_sim.sh \
  motion_enabled:=true allow_full_run:=true record_data:=false gui:=true
```

Run static contracts from the package root with Python 3. The real test must
be performed only after the six bottom-layer checker reports `READY`.
