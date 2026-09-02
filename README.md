# GO2 Mid-360 Autonomous Navigation

GO2 autonomous navigation on NVIDIA Jetson, ROS Noetic, Livox Mid-360, and
Unitree SDK2. The active workspace follows the same operating structure as the
AADCL Scout Mini project while retaining GO2-specific calibration, geometry,
safety limits, and low-level motion control.

## Active navigation chain

```text
Mid-360 + IMU -> FAST-LIO -> static map -> NDT localization
               -> move_base + TEB -> /cmd_vel_nav
               -> GO2 watchdog -> /cmd_vel_safe -> Unitree SDK2
```

The first TEB release is non-holonomic: lateral velocity is disabled. The real
SDK bridge starts disabled and must never be enabled before localization and
dry-run command checks pass.

## Build

```bash
source /opt/ros/noetic/setup.bash
cd ~/go2_mid360_nav
catkin_make -j1
source devel/setup.bash
```

## Canonical workflows

```bash
# Mapping
roslaunch go2_system_bringup go2_mapping.launch map_name:=site01

# Localization
roslaunch go2_system_bringup go2_localization.launch map_name:=site01

# Navigation (real Unitree bridge is separate and remains disabled)
roslaunch go2_system_bringup go2_navigation.launch map_name:=site01

# Static launch-graph validation; this starts no ROS nodes
python3 scripts/validate_launch_contracts.py

# Read-only command check after a navigation dry run
python3 scripts/check_cmd_vel.py --duration 10
```

These canonical launch files are introduced incrementally on the
`codex/scout-teb-migration` branch. Until the migration acceptance checks pass,
use `main` for the previous tracked implementation.

## Repository policy

- Active ROS packages live under `src/` in one Catkin workspace.
- Maps, bags, datasets, logs, build products, credentials, and backups are not
  committed.
- Third-party snapshots contain no nested `.git` directories; source and
  revisions are recorded in [`src/THIRD_PARTY.md`](src/THIRD_PARTY.md).
- The verified full pre-migration backup remains on the Jetson NVMe drive.

See `docs/superpowers/specs/2026-09-01-go2-scout-teb-migration-design.md` for
the approved design and `docs/superpowers/plans/2026-09-01-go2-scout-teb-migration.md`
for the staged implementation and validation plan.
